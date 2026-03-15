/**
 * @file power_service.h
 * @brief Power management service placeholder
 *
 * Future responsibilities:
 *   - Battery voltage / SoC monitoring
 *   - Solar charge controller interface
 *   - Sleep mode management (light sleep between samples)
 *   - Brown-out detection and graceful shutdown
 *   - Peripheral power gating (turn off sensors during deep sleep)
 *
 * ──────────────────────────────────────────────────────────────────
 *  INTEGRATION NOTES
 * ──────────────────────────────────────────────────────────────────
 *  The ESP32 can enter light sleep between the 5 s sample intervals,
 *  reducing average current from ~80 mA to ~0.8 mA.  The FreeRTOS
 *  tickless idle hook handles this automatically when configured.
 *
 *  For solar-powered deployments, monitor battery voltage via an ADC
 *  pin and reduce sampling frequency (e.g., 10 s intervals, still
 *  within the WMO 10-min acceptable window) when SoC is low.
 * ──────────────────────────────────────────────────────────────────
 */

#ifndef POWER_SERVICE_H
#define POWER_SERVICE_H

#include <cstdint>

class PowerService {
public:
    bool begin();

    /** Read battery voltage via ADC (returns volts, or -1 if not configured). */
    float getBatteryVoltage() const;

    /** Returns estimated state of charge [0.0 – 1.0]. */
    float getStateOfCharge() const;

    /** Enable ESP32 light sleep between sample intervals. */
    void enableLightSleep();

    /** Disable light sleep (e.g., during OTA update). */
    void disableLightSleep();

private:
    bool  _lightSleepEnabled = false;
    float _batteryVoltage    = -1.0f;
};

#endif // POWER_SERVICE_H
