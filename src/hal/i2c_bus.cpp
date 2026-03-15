/**
 * @file i2c_bus.cpp
 * @brief I2C bus management implementation
 */

#include "i2c_bus.h"
#include <Arduino.h>

// Global singleton instance
I2CBus g_i2c;

bool I2CBus::begin() {
    if (_initialised) return true;

    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) {
        Serial.println(F("[I2C] FATAL: Failed to create bus mutex"));
        return false;
    }

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);

    _initialised = true;
    Serial.printf("[I2C] Bus initialised — SDA=%d SCL=%d @ %lu Hz\n",
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    return true;
}

bool I2CBus::lock(uint32_t timeout_ms) {
    if (!_mutex) return false;
    return xSemaphoreTake(_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void I2CBus::unlock() {
    if (_mutex) {
        xSemaphoreGive(_mutex);
    }
}

TwoWire& I2CBus::wire() {
    return Wire;
}

bool I2CBus::probe(uint8_t addr) {
    if (!lock()) return false;

    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    unlock();
    return (err == 0);
}
