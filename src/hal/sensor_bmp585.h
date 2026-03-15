/**
 * @file sensor_bmp585.h
 * @brief Driver for the Adafruit BMP585 pressure/temperature breakout
 *
 * The BMP585 (Bosch Sensortec) is a high-accuracy barometric pressure
 * sensor with on-chip temperature compensation.
 *
 * Key specs:
 *   Pressure accuracy:    ±0.5 hPa (typical, absolute)
 *   Pressure noise:       ~0.08 hPa RMS at highest oversampling
 *   Temperature accuracy: ±0.5 °C (typical)
 *   Interface: I2C / SPI
 *
 * The die temperature is logged as a diagnostic cross-check against
 * the HDC3022 primary air temperature.
 */

#ifndef SENSOR_BMP585_H
#define SENSOR_BMP585_H

#include "sensor_interface.h"
#include "i2c_bus.h"
#include <Adafruit_BMP5xx.h>

class SensorBMP585 : public ISensor {
public:
    explicit SensorBMP585(uint8_t addr = BMP585_I2C_ADDR);

    const char*  name()   const override { return "BMP585"; }
    bool         begin()        override;
    bool         read(RawSample& sample) override;
    SensorStatus status() const override { return _status; }
    bool         reset()        override;

private:
    /**
     * @brief Configure oversampling and IIR filter for WMO-grade readings.
     *
     * High oversampling reduces noise; IIR filter smooths short-term
     * fluctuations without adding significant lag for a 5 s sample rate.
     */
    void configureForMeteorology();

    Adafruit_BMP5xx _bmp;
    uint8_t         _addr;
    SensorStatus    _status;
    uint8_t         _consecutive_fails;
};

#endif // SENSOR_BMP585_H
