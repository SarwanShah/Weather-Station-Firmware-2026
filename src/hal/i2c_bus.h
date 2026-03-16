/**
 * @file i2c_bus.h
 * @brief Thread-safe I2C bus management
 *
 * Wraps Arduino Wire with a FreeRTOS mutex so that multiple tasks
 * (or future sensors on the same bus) cannot collide.
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "../config.h"

class I2CBus {
public:
    /**
     * @brief Initialise the I2C peripheral and create the bus mutex.
     * @return true if initialisation succeeded.
     */
    bool begin();

    /**
     * @brief Acquire exclusive access to the bus.
     * @param timeout_ms Maximum time to wait for the mutex.
     * @return true if lock acquired within timeout.
     */
    bool lock(uint32_t timeout_ms = 100);

    /**
     * @brief Release the bus mutex.
     */
    void unlock();

    /**
     * @brief Get the underlying TwoWire reference (use only while locked).
     */
    TwoWire& wire();

    /**
     * @brief Probe an I2C address.
     * @param addr 7-bit address to probe.
     * @return true if a device ACKs at addr.
     */
    bool probe(uint8_t addr);

    /**
     * @brief Re-apply I2C pin and clock configuration.
     *
     * Some Adafruit libraries call Wire.begin() internally, which can
     * reinitialise the ESP32 I2C peripheral and disrupt subsequent
     * transactions.  Call this after any library init to restore the
     * bus to a known-good state.
     */
    void reinit();

private:
    SemaphoreHandle_t _mutex = nullptr;
    bool              _initialised = false;
};

// Global singleton — defined in i2c_bus.cpp
extern I2CBus g_i2c;

#endif // I2C_BUS_H
