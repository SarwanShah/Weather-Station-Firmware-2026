/**
 * @file sensor_hdc3022.cpp
 * @brief HDC3022 driver implementation
 */

#include "sensor_hdc3022.h"
#include "../config.h"

SensorHDC3022::SensorHDC3022(uint8_t addr)
    : _addr(addr)
    , _status(SensorStatus::NOT_PRESENT)
    , _consecutive_fails(0)
{}

bool SensorHDC3022::begin() {
    // Probe the device on the bus first
    if (!g_i2c.probe(_addr)) {
        Serial.printf("[HDC3022] Device not found at 0x%02X\n", _addr);
        _status = SensorStatus::NOT_PRESENT;
        return false;
    }

    // Initialise the Adafruit library
    if (!g_i2c.lock()) return false;

    bool ok = _hdc.begin(_addr, &g_i2c.wire());

    g_i2c.unlock();

    if (!ok) {
        Serial.println(F("[HDC3022] Library init failed"));
        _status = SensorStatus::NOT_PRESENT;
        return false;
    }

    _status = SensorStatus::WARMING_UP;
    _consecutive_fails = 0;

    Serial.printf("[HDC3022] Initialised at 0x%02X\n", _addr);

    // Allow sensor to stabilise (datasheet recommends ≥1 s after power-on)
    vTaskDelay(pdMS_TO_TICKS(1500));
    _status = SensorStatus::OK;

    return true;
}

bool SensorHDC3022::read(RawSample& sample) {
    sample.temp_valid = false;
    sample.hum_valid  = false;

    if (_status == SensorStatus::NOT_PRESENT) return false;

    if (!g_i2c.lock()) {
        Serial.println(F("[HDC3022] Bus lock timeout"));
        return false;
    }

    // Trigger a single-shot measurement in high-accuracy mode.
    // The Adafruit library handles the command, wait, and CRC check.
    double temp = 0.0, hum = 0.0;
    bool success = false;

    for (uint8_t retry = 0; retry <= I2C_MAX_RETRIES; ++retry) {
        // readTemperatureHumidityOnDemand triggers measurement and returns results
        if (_hdc.readTemperatureHumidityOnDemand(temp, hum, TRIGGERMODE_LP0)) {
            success = true;
            break;
        }
        if (retry < I2C_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
        }
    }

    g_i2c.unlock();

    if (success) {
        // Basic plausibility checks (WMO range: –80 to 60 °C, 0–100 %RH)
        bool temp_plausible = (temp >= -80.0f && temp <= 60.0f);
        bool hum_plausible  = (hum  >=   0.0f && hum  <= 100.0f);

        if (temp_plausible) {
            sample.temperature_c = temp;
            sample.temp_valid    = true;
        } else {
            Serial.printf("[HDC3022] WARN: Temp out of range: %.2f °C\n", temp);
        }

        if (hum_plausible) {
            sample.humidity_pct = hum;
            sample.hum_valid    = true;
        } else {
            Serial.printf("[HDC3022] WARN: RH out of range: %.2f %%\n", hum);
        }

        _consecutive_fails = 0;
        _status = SensorStatus::OK;
    } else {
        _consecutive_fails++;
        Serial.printf("[HDC3022] Read failed (%u/%u)\n",
                      _consecutive_fails, MAX_CONSECUTIVE_FAILS);

        if (_consecutive_fails >= MAX_CONSECUTIVE_FAILS) {
            _status = SensorStatus::FAULTED;
        } else {
            _status = SensorStatus::DEGRADED;
        }
    }

    return success;
}

bool SensorHDC3022::reset() {
    Serial.println(F("[HDC3022] Performing soft reset..."));

    if (!g_i2c.lock()) return false;

    // Send soft reset command (0x30A2 per HDC302x datasheet)
    g_i2c.wire().beginTransmission(_addr);
    g_i2c.wire().write(0x30);
    g_i2c.wire().write(0xA2);
    g_i2c.wire().endTransmission();

    g_i2c.unlock();

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete

    _consecutive_fails = 0;
    _status = SensorStatus::WARMING_UP;

    vTaskDelay(pdMS_TO_TICKS(1500));
    _status = SensorStatus::OK;

    return true;
}

void SensorHDC3022::activateHeater(uint32_t duration_ms) {
    if (!g_i2c.lock()) return;

    // Enable heater at medium power (command varies by library version)
    // Refer to HDC302x datasheet for heater control commands
    Serial.printf("[HDC3022] Heater ON for %lu ms\n", duration_ms);

    // The Adafruit library may provide heater control; if not,
    // send raw command: 0x306E (heater 200mA, 1s)
    g_i2c.wire().beginTransmission(_addr);
    g_i2c.wire().write(0x30);
    g_i2c.wire().write(0x6E);
    g_i2c.wire().endTransmission();

    g_i2c.unlock();

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Heater auto-disables after the programmed duration
    Serial.println(F("[HDC3022] Heater OFF"));
}
