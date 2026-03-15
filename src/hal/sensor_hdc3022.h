/**
 * @file sensor_hdc3022.h
 * @brief Driver for the Adafruit HDC3022 temperature/humidity breakout
 *
 * The HDC3022 (Texas Instruments) is a high-accuracy digital humidity
 * and temperature sensor with integrated heater for defogging.
 *
 * Key specs:
 *   Temperature accuracy: ±0.2 °C (typical)
 *   Humidity accuracy:    ±1.5 %RH (typical)
 *   Interface: I2C up to 1 MHz
 *
 * This driver uses the Adafruit_HDC302x Arduino library which handles
 * CRC verification and command sequencing.
 */

#ifndef SENSOR_HDC3022_H
#define SENSOR_HDC3022_H

#include "sensor_interface.h"
#include "i2c_bus.h"
#include <Adafruit_HDC302x.h>

class SensorHDC3022 : public ISensor {
public:
    explicit SensorHDC3022(uint8_t addr = HDC3022_I2C_ADDR);

    const char*  name()   const override { return "HDC3022"; }
    bool         begin()        override;
    bool         read(RawSample& sample) override;
    SensorStatus status() const override { return _status; }
    bool         reset()        override;

    /**
     * @brief Activate the on-chip heater for condensation removal.
     * @param duration_ms Heater on-time (max ~5 s recommended).
     */
    void activateHeater(uint32_t duration_ms = 1000);

private:
    Adafruit_HDC302x _hdc;
    uint8_t          _addr;
    SensorStatus     _status;
    uint8_t          _consecutive_fails;
};

#endif // SENSOR_HDC3022_H
