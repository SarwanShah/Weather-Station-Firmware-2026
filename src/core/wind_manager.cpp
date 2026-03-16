/**
 * @file wind_manager.cpp
 * @brief WindManager implementation — WMO-compliant wind acquisition
 *
 * Sampling strategy (WMO-No. 8 Chapter 5, §5.8.3):
 *   - 4 Hz ADC reads (every 250 ms) with 16x oversampling per read
 *   - Overlapping 3-s running average computed every 250 ms for gust detection
 *   - 2-minute running mean maintained via circular buffer
 *   - 10-minute window produces final WindRecord
 *
 * ADC → Voltage → Wind Speed conversion:
 *   adc_counts → voltage = counts * (Vref / 4095)
 *   sensor_voltage = voltage / DIVIDER_RATIO
 *   wind_speed_mps = sensor_voltage * MPS_PER_VOLT
 */

#include "wind_manager.h"
#include "Arduino.h"
#include <cstring>
#include <cmath>
#include <cfloat>
#include <driver/adc.h>
#include <esp_adc_cal.h>

// ADC calibration (legacy ESP-IDF driver)
static esp_adc_cal_characteristics_t _adcChars;
static bool                          _adcCalibrated = false;

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

WindManager::WindManager()
    : _gustBufIdx(0)
    , _gustBufCount(0)
    , _twoMinBufIdx(0)
    , _twoMinBufCount(0)
    , _tenMinSum(0.0)
    , _tenMinCount(0)
    , _peakGust(0.0f)
    , _minSpeed(FLT_MAX)
    , _windowStartMs(0)
    , _status(SensorStatus::NOT_PRESENT)
    , _totalWindows(0)
    , _consecutiveFails(0)
{
    memset(_gustBuf, 0, sizeof(_gustBuf));
    memset(_twoMinBuf, 0, sizeof(_twoMinBuf));
    memset(&_latestRecord, 0, sizeof(_latestRecord));
}

// ─────────────────────────────────────────────────────────────────────────────
// Start — configure ADC, create task and timer
// ─────────────────────────────────────────────────────────────────────────────

bool WindManager::start() {
    // Configure ADC1 — GPIO34 = ADC1_CH6 (legacy ESP-IDF driver)
    if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK) {
        Serial.println(F("[Wind] FATAL: ADC width config failed"));
        _status = SensorStatus::FAULTED;
        return false;
    }

    if (adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12) != ESP_OK) {
        Serial.println(F("[Wind] FATAL: ADC channel atten config failed"));
        _status = SensorStatus::FAULTED;
        return false;
    }

    // Characterise ADC for voltage calibration (uses eFuse Vref if burned)
    esp_adc_cal_value_t calType = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12,
        1100,   // default Vref (mV) if eFuse not programmed
        &_adcChars
    );
    _adcCalibrated = (calType == ESP_ADC_CAL_VAL_EFUSE_VREF
                   || calType == ESP_ADC_CAL_VAL_EFUSE_TP);
    Serial.printf("[Wind] ADC calibration: %s\n",
                  _adcCalibrated ? "eFuse/calibrated" : "default Vref");

    // Verify ADC with test read
    int testRead = adc1_get_raw(ADC1_CHANNEL_6);
    if (testRead < 0) {
        Serial.println(F("[Wind] FATAL: ADC read failed"));
        _status = SensorStatus::FAULTED;
        return false;
    }
    Serial.printf("[Wind] ADC test read: %d counts (GPIO%d)\n", testRead, WIND_ADC_PIN);

    // Create sync primitives
    _dataMutex = xSemaphoreCreateMutex();
    if (!_dataMutex) {
        Serial.println(F("[Wind] FATAL: Mutex creation failed"));
        return false;
    }

    _outputQueue = xQueueCreate(3, sizeof(WindRecord));
    if (!_outputQueue) {
        Serial.println(F("[Wind] FATAL: Queue creation failed"));
        return false;
    }

    // Create task on APP_CORE
    BaseType_t ret = xTaskCreatePinnedToCore(
        windTaskEntry, "WindSample", WIND_TASK_STACK,
        this, WIND_TASK_PRIO, &_taskHandle, APP_CORE
    );
    if (ret != pdPASS) {
        Serial.println(F("[Wind] FATAL: Task creation failed"));
        return false;
    }

    // Create 4 Hz software timer
    _timer = xTimerCreate(
        "WindTmr", pdMS_TO_TICKS(WIND_SAMPLE_INTERVAL_MS),
        pdTRUE, this, timerCallback
    );
    if (!_timer) {
        Serial.println(F("[Wind] FATAL: Timer creation failed"));
        return false;
    }

    resetWindow();
    _status = SensorStatus::OK;

    if (xTimerStart(_timer, pdMS_TO_TICKS(100)) != pdPASS) {
        Serial.println(F("[Wind] FATAL: Timer start failed"));
        return false;
    }

    Serial.println(F("[Wind] Acquisition started"));
    Serial.printf("[Wind] Sample: %lu Hz | Gust window: %lu s (%lu samples) | "
                  "Avg: 2 min (%lu) & 10 min (%lu)\n",
                  WIND_SAMPLE_FREQ_HZ, WIND_GUST_WINDOW_SEC,
                  WIND_GUST_WINDOW_SAMPLES, WIND_AVG_2MIN_SAMPLES,
                  WIND_AVG_10MIN_SAMPLES);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void WindManager::timerCallback(TimerHandle_t timer) {
    WindManager* mgr = static_cast<WindManager*>(pvTimerGetTimerID(timer));
    if (mgr && mgr->_taskHandle) {
        xTaskNotifyGive(mgr->_taskHandle);
    }
}

void WindManager::windTaskEntry(void* param) {
    static_cast<WindManager*>(param)->windTaskLoop();
}

void WindManager::windTaskLoop() {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float speed = readWindSpeedMps();

        if (speed >= 0.0f) {
            processSample(speed);
            _consecutiveFails = 0;
            _status = SensorStatus::OK;
        } else {
            _consecutiveFails++;
            if (_consecutiveFails >= MAX_CONSECUTIVE_FAILS) {
                _status = SensorStatus::FAULTED;
            } else {
                _status = SensorStatus::DEGRADED;
            }
        }

#if DEBUG_VERBOSE
        // Print every 20th sample (~5 s) to align with T/P/RH sample prints
        if (_tenMinCount > 0 && (_tenMinCount % (WIND_SAMPLE_FREQ_HZ * SAMPLE_INTERVAL_SEC)) == 0) {
            float gust3s = 0.0f;
            if (_gustBufCount >= WIND_GUST_WINDOW_SAMPLES) {
                float gs = 0.0f;
                for (uint16_t i = 0; i < WIND_GUST_WINDOW_SAMPLES; ++i) gs += _gustBuf[i];
                gust3s = gs / (float)WIND_GUST_WINDOW_SAMPLES;
            }
            Serial.printf("[Wind %4lu/%lu] Speed=%.1f m/s  3s-avg=%.1f m/s  Gust=%.1f m/s\n",
                          _tenMinCount, WIND_AVG_10MIN_SAMPLES,
                          speed >= 0.0f ? speed : 0.0f,
                          gust3s, _peakGust);
        }
#endif

        if (_tenMinCount >= WIND_AVG_10MIN_SAMPLES) {
            finaliseWindow();
            resetWindow();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Read wind speed from ADC with multi-sampling
// ─────────────────────────────────────────────────────────────────────────────

float WindManager::readWindSpeedMps() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < WIND_ADC_OVERSAMPLE; ++i) {
        int raw = adc1_get_raw(ADC1_CHANNEL_6);
        if (raw < 0) return -1.0f;
        sum += (uint32_t)raw;
    }
    uint32_t avgCounts = sum / WIND_ADC_OVERSAMPLE;

    // Convert to millivolts using esp_adc_cal (uses eFuse Vref if available)
    float voltage_mv = (float)esp_adc_cal_raw_to_voltage(avgCounts, &_adcChars);

    // Undo voltage divider, convert to m/s
    float sensorVoltage = (voltage_mv / 1000.0f) / WIND_DIVIDER_RATIO;
    float speed_mps = sensorVoltage * WIND_MPS_PER_VOLT;

    // Subtract zero-wind offset (ADC/divider bias measured at no-wind)
    speed_mps -= WIND_ZERO_OFFSET_MPS;

    // Clamp to sensor range
    if (speed_mps < 0.0f) speed_mps = 0.0f;
    if (speed_mps > WIND_SENSOR_MAX_MPS) speed_mps = WIND_SENSOR_MAX_MPS;

    return speed_mps;
}

// ─────────────────────────────────────────────────────────────────────────────

void WindManager::processSample(float speed_mps) {
    // 1. Push into 3-s circular buffer for gust detection
    _gustBuf[_gustBufIdx] = speed_mps;
    _gustBufIdx = (_gustBufIdx + 1) % WIND_GUST_WINDOW_SAMPLES;
    if (_gustBufCount < WIND_GUST_WINDOW_SAMPLES) _gustBufCount++;

    // Compute overlapping 3-s running average (only when buffer full)
    if (_gustBufCount >= WIND_GUST_WINDOW_SAMPLES) {
        float gustSum = 0.0f;
        for (uint16_t i = 0; i < WIND_GUST_WINDOW_SAMPLES; ++i) {
            gustSum += _gustBuf[i];
        }
        float gust3s = gustSum / (float)WIND_GUST_WINDOW_SAMPLES;
        if (gust3s > _peakGust) _peakGust = gust3s;
    }

    // 2. Push into 2-min circular buffer
    _twoMinBuf[_twoMinBufIdx] = speed_mps;
    _twoMinBufIdx = (_twoMinBufIdx + 1) % WIND_AVG_2MIN_SAMPLES;
    if (_twoMinBufCount < WIND_AVG_2MIN_SAMPLES) _twoMinBufCount++;

    // 3. Accumulate into 10-min sum
    _tenMinSum += (double)speed_mps;
    _tenMinCount++;

    // 4. Track minimum instantaneous speed
    if (speed_mps < _minSpeed) _minSpeed = speed_mps;
}

// ─────────────────────────────────────────────────────────────────────────────

void WindManager::finaliseWindow() {
    WindRecord rec;
    memset(&rec, 0, sizeof(rec));

    rec.window_start_ms = _windowStartMs;
    rec.window_end_ms   = millis();
    rec.total_samples   = _tenMinCount;
    rec.anem_status     = _status;

    // 10-min mean
    if (_tenMinCount > 0) {
        rec.speed_mean_10min = roundToResolution(
            (float)(_tenMinSum / _tenMinCount), WIND_SPEED_RESOLUTION);
    } else {
        rec.speed_mean_10min = NAN;
    }

    // 2-min mean (from circular buffer)
    if (_twoMinBufCount > 0) {
        float twoMinSum = 0.0f;
        for (uint16_t i = 0; i < _twoMinBufCount; ++i) {
            twoMinSum += _twoMinBuf[i];
        }
        rec.speed_mean_2min = roundToResolution(
            twoMinSum / (float)_twoMinBufCount, WIND_SPEED_RESOLUTION);
    } else {
        rec.speed_mean_2min = NAN;
    }

    rec.speed_max_3s = roundToResolution(_peakGust, WIND_GUST_RESOLUTION);
    rec.speed_min = (_minSpeed < FLT_MAX)
                  ? roundToResolution(_minSpeed, WIND_GUST_RESOLUTION) : NAN;

    // Publish to queue
    if (xQueueSend(_outputQueue, &rec, 0) != pdPASS) {
        WindRecord discard;
        xQueueReceive(_outputQueue, &discard, 0);
        xQueueSend(_outputQueue, &rec, 0);
    }

    // Update latest record
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _latestRecord = rec;
        _hasValidRecord = true;
        xSemaphoreGive(_dataMutex);
    }

    _totalWindows++;

    Serial.println(F("════════════ WIND 10-MIN SUMMARY ════════════"));
    Serial.printf("  Window #%lu  [%lu → %lu ms]  (%lu samples)\n",
                  _totalWindows, rec.window_start_ms, rec.window_end_ms, rec.total_samples);
    Serial.printf("  10-min mean : %.1f m/s\n", rec.speed_mean_10min);
    Serial.printf("  2-min mean  : %.1f m/s  (most recent 2 min)\n", rec.speed_mean_2min);
    Serial.printf("  Peak gust   : %.1f m/s  (max 3-s avg)\n", rec.speed_max_3s);
    Serial.printf("  Min speed   : %.1f m/s\n", rec.speed_min);
    Serial.println(F("═════════════════════════════════════════════"));
}

void WindManager::resetWindow() {
    _tenMinSum = 0.0;
    _tenMinCount = 0;
    _peakGust = 0.0f;
    _minSpeed = FLT_MAX;
    _windowStartMs = millis();

    // NOTE: Neither the 2-min nor the 3-s gust buffer is reset across
    // window boundaries.  WMO requires a continuous rolling 2-min mean,
    // and gusts straddling the boundary must still be detected.
}

bool WindManager::getLatestRecord(WindRecord& record) const {
    if (!_dataMutex) return false;
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (_hasValidRecord) {
            record = _latestRecord;
            xSemaphoreGive(_dataMutex);
            return true;
        }
        xSemaphoreGive(_dataMutex);
    }
    return false;
}
