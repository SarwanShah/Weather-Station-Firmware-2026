/**
 * @file sensor_manager.cpp
 * @brief SensorManager implementation — WMO-compliant acquisition engine
 */

#include "sensor_manager.h"
#include <cmath>
#include <cfloat>
#include <cstring>
#include "Arduino.h"
#include "../hal/i2c_bus.h"

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

SensorManager::SensorManager() {
    memset(&_accum, 0, sizeof(_accum));
    memset(&_latestRecord, 0, sizeof(_latestRecord));
}

void SensorManager::registerSensor(ISensor* sensor) {
    if (sensor) {
        _sensors.push_back(sensor);
        Serial.printf("[SensorMgr] Registered sensor: %s\n", sensor->name());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Start — create FreeRTOS primitives and launch the acquisition task
// ─────────────────────────────────────────────────────────────────────────────

bool SensorManager::start() {
    // Create synchronisation primitives
    _dataMutex = xSemaphoreCreateMutex();
    if (!_dataMutex) {
        Serial.println(F("[SensorMgr] FATAL: Mutex creation failed"));
        return false;
    }

    _outputQueue = xQueueCreate(5, sizeof(AveragedRecord));
    if (!_outputQueue) {
        Serial.println(F("[SensorMgr] FATAL: Queue creation failed"));
        return false;
    }

    // Initialise all registered sensors.
    // Re-apply I2C config after each attempt because some Adafruit
    // libraries call Wire.begin() internally, which can disrupt the
    // ESP32 I2C peripheral and cause subsequent sensors to fail.
    uint8_t ok_count = 0;
    for (auto* s : _sensors) {
        if (s->begin()) {
            ok_count++;
        } else {
            Serial.printf("[SensorMgr] WARNING: %s init failed\n", s->name());
        }
        g_i2c.reinit();
    }

    if (ok_count == 0) {
        Serial.println(F("[SensorMgr] FATAL: No sensors initialised"));
        return false;
    }

    Serial.printf("[SensorMgr] %u / %u sensors ready\n",
                  ok_count, (uint8_t)_sensors.size());

    // Create the sampling task on APP_CORE
    BaseType_t ret = xTaskCreatePinnedToCore(
        sampleTaskEntry,        // Entry function
        "SensorSample",         // Name
        SENSOR_TASK_STACK,      // Stack size
        this,                   // Parameter (this pointer)
        SENSOR_TASK_PRIO,       // Priority
        &_sampleTaskHandle,     // Handle
        APP_CORE                // Core affinity
    );

    if (ret != pdPASS) {
        Serial.println(F("[SensorMgr] FATAL: Task creation failed"));
        return false;
    }

    // Create the periodic software timer
    _sampleTimer = xTimerCreate(
        "SampleTmr",                            // Name
        pdMS_TO_TICKS(SAMPLE_INTERVAL_MS),       // Period
        pdTRUE,                                  // Auto-reload
        this,                                    // Timer ID
        timerCallback                            // Callback
    );

    if (!_sampleTimer) {
        Serial.println(F("[SensorMgr] FATAL: Timer creation failed"));
        return false;
    }

    // Reset the accumulator and start the timer
    resetWindow();

    if (xTimerStart(_sampleTimer, pdMS_TO_TICKS(100)) != pdPASS) {
        Serial.println(F("[SensorMgr] FATAL: Timer start failed"));
        return false;
    }

    Serial.println(F("[SensorMgr] Acquisition started"));
    Serial.printf("[SensorMgr] Sample interval: %lu s | Window: %lu s | Samples/window: %lu\n",
                  SAMPLE_INTERVAL_SEC, AVERAGING_WINDOW_SEC, SAMPLES_PER_WINDOW);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// FreeRTOS Timer Callback → notify the sample task
// ─────────────────────────────────────────────────────────────────────────────

void SensorManager::timerCallback(TimerHandle_t timer) {
    SensorManager* mgr = static_cast<SensorManager*>(pvTimerGetTimerID(timer));
    if (mgr && mgr->_sampleTaskHandle) {
        // Use task notification as a lightweight, zero-copy trigger.
        // eSetBits ensures we don't lose a notification if the task
        // hasn't consumed the previous one yet.
        xTaskNotifyGive(mgr->_sampleTaskHandle);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sample Task — waits for timer notifications, reads sensors, accumulates
// ─────────────────────────────────────────────────────────────────────────────

void SensorManager::sampleTaskEntry(void* param) {
    static_cast<SensorManager*>(param)->sampleTaskLoop();
}

void SensorManager::sampleTaskLoop() {
    for (;;) {
        // Block until the timer notifies us (no CPU burn while waiting)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        acquireSample();

        // Check if the window is complete
        if (_accum.total_samples >= SAMPLES_PER_WINDOW) {
            finaliseWindow();
            resetWindow();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Acquire a single sample from all registered sensors
// ─────────────────────────────────────────────────────────────────────────────

void SensorManager::acquireSample() {
    RawSample sample;
    memset(&sample, 0, sizeof(sample));
    sample.timestamp_ms = millis();

    // Read each sensor sequentially (they share the same I2C bus;
    // each driver acquires/releases the bus mutex internally)
    for (auto* s : _sensors) {
        if (s->status() == SensorStatus::FAULTED) {
            // Attempt recovery once per window
            if (_accum.total_samples == 0) {
                s->reset();
            }
            continue;
        }

        if (!s->read(sample)) {
            _totalFailures++;
        }
    }

    // Accumulate valid readings into the window
    if (sample.temp_valid) {
        _accum.temp_sum += sample.temperature_c;
        _accum.temp_count++;

        if (sample.temperature_c < _accum.temp_min) _accum.temp_min = sample.temperature_c;
        if (sample.temperature_c > _accum.temp_max) _accum.temp_max = sample.temperature_c;
    }

    if (sample.hum_valid) {
        _accum.hum_sum += sample.humidity_pct;
        _accum.hum_count++;
    }

    if (sample.press_valid) {
        _accum.press_sum += sample.pressure_hpa;
        _accum.press_count++;

        if (sample.pressure_hpa < _accum.press_min) _accum.press_min = sample.pressure_hpa;
        if (sample.pressure_hpa > _accum.press_max) _accum.press_max = sample.pressure_hpa;
    }

    _accum.total_samples++;

#if DEBUG_VERBOSE
    Serial.printf("[Sample %2u/%lu] T=%.1f°C  RH=%.0f%%  P=%.1f hPa  [%s %s %s]\n",
                  _accum.total_samples, SAMPLES_PER_WINDOW,
                  sample.temp_valid  ? sample.temperature_c : NAN,
                  sample.hum_valid   ? sample.humidity_pct  : NAN,
                  sample.press_valid ? sample.pressure_hpa  : NAN,
                  sample.temp_valid  ? "T:OK" : "T:--",
                  sample.hum_valid   ? "H:OK" : "H:--",
                  sample.press_valid ? "P:OK" : "P:--");
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// Finalise the 1-minute window: compute means, publish record
// ─────────────────────────────────────────────────────────────────────────────

void SensorManager::finaliseWindow() {
    AveragedRecord rec;
    memset(&rec, 0, sizeof(rec));

    rec.window_start_ms  = _accum.window_start_ms;
    rec.window_end_ms    = millis();
    rec.temp_sample_count  = _accum.temp_count;
    rec.hum_sample_count   = _accum.hum_count;
    rec.press_sample_count = _accum.press_count;

    // ── Compute arithmetic means ──
    if (_accum.temp_count > 0) {
        rec.temperature_c = roundToResolution(
            _accum.temp_sum / _accum.temp_count,
            TEMP_RESOLUTION
        );
        rec.temp_min = roundToResolution(_accum.temp_min, TEMP_RESOLUTION);
        rec.temp_max = roundToResolution(_accum.temp_max, TEMP_RESOLUTION);
    } else {
        rec.temperature_c = NAN;
        rec.temp_min = NAN;
        rec.temp_max = NAN;
    }

    if (_accum.hum_count > 0) {
        rec.humidity_pct = roundToResolution(
            _accum.hum_sum / _accum.hum_count,
            RH_RESOLUTION
        );
    } else {
        rec.humidity_pct = NAN;
    }

    if (_accum.press_count > 0) {
        rec.pressure_hpa = roundToResolution(
            _accum.press_sum / _accum.press_count,
            PRESS_RESOLUTION
        );
        rec.press_min = roundToResolution(_accum.press_min, PRESS_RESOLUTION);
        rec.press_max = roundToResolution(_accum.press_max, PRESS_RESOLUTION);
    } else {
        rec.pressure_hpa = NAN;
        rec.press_min = NAN;
        rec.press_max = NAN;
    }

    // Attach sensor status at window close
    for (auto* s : _sensors) {
        // Match by name — simple but effective for two sensors
        if (strcmp(s->name(), "HDC3022") == 0)  rec.hdc_status = s->status();
        if (strcmp(s->name(), "BMP585") == 0)   rec.bmp_status = s->status();
    }

    // ── Publish to output queue (non-blocking) ──
    if (xQueueSend(_outputQueue, &rec, 0) != pdPASS) {
        Serial.println(F("[SensorMgr] WARN: Output queue full — oldest record dropped"));
        // Remove oldest to make room
        AveragedRecord discard;
        xQueueReceive(_outputQueue, &discard, 0);
        xQueueSend(_outputQueue, &rec, 0);
    }

    // ── Update latest record (mutex-protected) ──
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _latestRecord = rec;
        _hasValidRecord = true;
        xSemaphoreGive(_dataMutex);
    }

    _totalWindows++;

    // ── Log the 1-minute summary ──
    Serial.println(F("════════════════════════════════════════════════════════════"));
    Serial.printf("  1-MIN AVERAGE #%lu  [%lu → %lu ms]\n",
                  _totalWindows, rec.window_start_ms, rec.window_end_ms);
    Serial.printf("  Temperature : %.1f °C  (min %.1f / max %.1f)  [%u samples]\n",
                  rec.temperature_c, rec.temp_min, rec.temp_max, rec.temp_sample_count);
    Serial.printf("  Humidity    : %.0f %%RH  [%u samples]\n",
                  rec.humidity_pct, rec.hum_sample_count);
    Serial.printf("  Pressure    : %.1f hPa  (min %.1f / max %.1f)  [%u samples]\n",
                  rec.pressure_hpa, rec.press_min, rec.press_max, rec.press_sample_count);
    Serial.printf("  HDC status  : %u | BMP status: %u\n",
                  (uint8_t)rec.hdc_status, (uint8_t)rec.bmp_status);
    Serial.println(F("════════════════════════════════════════════════════════════"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Reset the accumulator for a new window
// ─────────────────────────────────────────────────────────────────────────────

void SensorManager::resetWindow() {
    memset(&_accum, 0, sizeof(_accum));
    _accum.temp_min  =  FLT_MAX;
    _accum.temp_max  = -FLT_MAX;
    _accum.press_min =  FLT_MAX;
    _accum.press_max = -FLT_MAX;
    _accum.window_start_ms = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// Public accessors
// ─────────────────────────────────────────────────────────────────────────────

bool SensorManager::getLatestRecord(AveragedRecord& record) const {
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

SystemHealth SensorManager::getHealth() const {
SystemHealth h;
    memset(&h, 0, sizeof(h));
    h.uptime_sec = millis() / 1000;
    h.total_windows_completed = _totalWindows;
    h.total_wind_windows_completed = 0;  // Set by caller (main.cpp healthTask)
    h.total_sample_failures = _totalFailures;
    h.free_heap_bytes = esp_get_free_heap_size();
    h.cpu_temperature_c = 0.0f;

    // Find sensor statuses
    h.hdc_status  = SensorStatus::NOT_PRESENT;
    h.bmp_status  = SensorStatus::NOT_PRESENT;
    h.anem_status = SensorStatus::NOT_PRESENT;  // Set by caller
    for (auto* s : _sensors) {
        if (strcmp(s->name(), "HDC3022") == 0) h.hdc_status = s->status();
        if (strcmp(s->name(), "BMP585") == 0)  h.bmp_status = s->status();
    }

    return h;
}
