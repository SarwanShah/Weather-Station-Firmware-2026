/**
 * @file sensor_bmp585.cpp
 * @brief BMP585 driver implementation
 */

#include "sensor_bmp585.h"
#include "../config.h"

SensorBMP585::SensorBMP585(uint8_t addr)
    : _addr(addr)
    , _status(SensorStatus::NOT_PRESENT)
    , _consecutive_fails(0)
{}

bool SensorBMP585::begin() {
    if (!g_i2c.probe(_addr)) {
        Serial.printf("[BMP585] Device not found at 0x%02X\n", _addr);
        _status = SensorStatus::NOT_PRESENT;
        return false;
    }

    if (!g_i2c.lock()) return false;

    bool ok = _bmp.begin(_addr, &g_i2c.wire());

    g_i2c.unlock();

    if (!ok) {
        Serial.println(F("[BMP585] Library init failed"));
        _status = SensorStatus::NOT_PRESENT;
        return false;
    }

    configureForMeteorology();

    _status = SensorStatus::WARMING_UP;
    _consecutive_fails = 0;

    Serial.printf("[BMP585] Initialised at 0x%02X\n", _addr);

    vTaskDelay(pdMS_TO_TICKS(500));
    _status = SensorStatus::OK;

    return true;
}

void SensorBMP585::configureForMeteorology() {
    if (!g_i2c.lock()) return;

    /**
     * Oversampling settings for meteorological use:
     *   Pressure OSR = 128x  → ~1.3 Pa RMS noise (~0.013 hPa)
     *   Temperature OSR = 8x → sufficient for compensation
     *
     * IIR filter coefficient = 3 (moderate smoothing).
     * At 5 s sampling this adds negligible group delay while
     * suppressing pressure spikes from door slams, gusts, etc.
     *
     * The Adafruit BMP581 library provides setTemperatureOversampling()
     * and setPressureOversampling() methods.
     */
    _bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    _bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_128X);
    _bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

    // Set ODR to suit forced-mode triggering (we trigger on demand)
    _bmp.setOutputDataRate(BMP5XX_ODR_0_125_HZ);  // Low ODR; we use forced mode

    g_i2c.unlock();

    Serial.println(F("[BMP585] Configured: P_OSR=128x, T_OSR=8x, IIR=3"));
}

bool SensorBMP585::read(RawSample& sample) {
    sample.press_valid = false;

    if (_status == SensorStatus::NOT_PRESENT) return false;

    if (!g_i2c.lock()) {
        Serial.println(F("[BMP585] Bus lock timeout"));
        return false;
    }

    bool success = false;
    float pressure = 0.0f;
    float bmp_temp = 0.0f;

    for (uint8_t retry = 0; retry <= I2C_MAX_RETRIES; ++retry) {
        // performReading triggers a forced-mode measurement and
        // stores results in _bmp.pressure (hPa) and _bmp.temperature (°C)
        if (_bmp.performReading()) {
            pressure = _bmp.pressure;  // already in hPa
            bmp_temp = _bmp.temperature;         // °C
            success = true;
            break;
        }
        if (retry < I2C_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
        }
    }

    g_i2c.unlock();

    if (success) {
        // Plausibility check (WMO range: 500–1080 hPa)
        bool press_plausible = (pressure >= 300.0f && pressure <= 1100.0f);
        bool temp_plausible  = (bmp_temp >= -80.0f && bmp_temp <= 85.0f);

        if (press_plausible) {
            sample.pressure_hpa = pressure;
            sample.press_valid  = true;
        } else {
            Serial.printf("[BMP585] WARN: Pressure out of range: %.2f hPa\n", pressure);
        }

        if (temp_plausible) {
            sample.bmp_temperature_c = bmp_temp;
        }

        _consecutive_fails = 0;
        _status = SensorStatus::OK;
    } else {
        _consecutive_fails++;
        Serial.printf("[BMP585] Read failed (%u/%u)\n",
                      _consecutive_fails, MAX_CONSECUTIVE_FAILS);

        if (_consecutive_fails >= MAX_CONSECUTIVE_FAILS) {
            _status = SensorStatus::FAULTED;
        } else {
            _status = SensorStatus::DEGRADED;
        }
    }

    return success;
}

bool SensorBMP585::reset() {
    Serial.println(F("[BMP585] Performing soft reset..."));

    if (!g_i2c.lock()) return false;

    // BMP585 soft reset command: write 0xB6 to register 0x7E
    g_i2c.wire().beginTransmission(_addr);
    g_i2c.wire().write(0x7E);
    g_i2c.wire().write(0xB6);
    g_i2c.wire().endTransmission();

    g_i2c.unlock();

    vTaskDelay(pdMS_TO_TICKS(50));

    _consecutive_fails = 0;
    _status = SensorStatus::WARMING_UP;

    // Re-configure after reset
    configureForMeteorology();

    vTaskDelay(pdMS_TO_TICKS(500));
    _status = SensorStatus::OK;

    return true;
}
