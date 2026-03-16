/**
 * @file config.h
 * @brief System-wide configuration constants
 *
 * Central configuration for the meteorological data acquisition system.
 * Sampling and averaging parameters are derived from WMO-No. 8 (2024),
 * Guide to Instruments and Methods of Observation, Volume I, Annex 1.A.
 *
 * WMO Requirements (Annex 1.A):
 *   Temperature  — time constant 20 s,  output averaging 1 min, resolution 0.1 K
 *   Humidity SS  — time constant 40 s,  output averaging 1 min, resolution 1 %RH
 *   Pressure     — time constant  2 s,  output averaging 1 min, resolution 0.1 hPa
 *   Wind speed   — distance const 2-5m, output averaging 2 & 10 min, resolution 0.5 m/s
 *   Wind gust    — 3 s running avg,     sampled at 4 Hz, resolution 0.1 m/s
 *
 * Sampling at 5 s intervals yields 12 samples per 1-min window, satisfying
 * Nyquist for all three sensor time constants.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// FreeRTOS Core Affinity
// ─────────────────────────────────────────────────────────────────────────────
/** Core on which all acquisition / processing tasks run. */
static constexpr BaseType_t APP_CORE            = 0;
/** Core reserved for user application tasks (comms, control, etc.). */
static constexpr BaseType_t USER_CORE           = 1;

// ─────────────────────────────────────────────────────────────────────────────
// I2C Bus Configuration
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t  I2C_PORT              = 0;       // I2C peripheral 0
static constexpr int      I2C_SDA_PIN           = 21;      // ESP32-WROOM-32D default
static constexpr int      I2C_SCL_PIN           = 22;      // ESP32-WROOM-32D default
static constexpr uint32_t I2C_FREQ_HZ           = 100000;  // 100 kHz (standard mode)

// ─────────────────────────────────────────────────────────────────────────────
// Sensor I2C Addresses (Adafruit breakout defaults)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t  HDC3022_I2C_ADDR      = 0x44;    // Adafruit HDC3022 default
static constexpr uint8_t  BMP585_I2C_ADDR       = 0x47;    // Adafruit BMP585  default

// ─────────────────────────────────────────────────────────────────────────────
// WMO-Compliant Sampling Parameters
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Sampling interval in seconds.
 * 5 s provides 12 samples per 1-min averaging window.
 * This exceeds Nyquist for:
 *   - Pressure  (τ = 2 s  → Nyquist = 4 s)
 *   - Temperature (τ = 20 s → well oversampled)
 *   - Humidity    (τ = 40 s → well oversampled)
 */
static constexpr uint32_t SAMPLE_INTERVAL_SEC   = 5;
static constexpr uint32_t SAMPLE_INTERVAL_MS    = SAMPLE_INTERVAL_SEC * 1000;

/**
 * Output averaging window in seconds (WMO Annex 1.A: 1 min).
 */
static constexpr uint32_t AVERAGING_WINDOW_SEC  = 60;

/**
 * Number of samples per averaging window.
 */
static constexpr uint32_t SAMPLES_PER_WINDOW    = AVERAGING_WINDOW_SEC / SAMPLE_INTERVAL_SEC;  // 12

// ─────────────────────────────────────────────────────────────────────────────
// Reporting Resolution (WMO Annex 1.A)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr float    TEMP_RESOLUTION       = 0.1f;    // °C (0.1 K)
static constexpr float    RH_RESOLUTION         = 1.0f;    // %RH
static constexpr float    PRESS_RESOLUTION      = 0.1f;    // hPa
static constexpr float    WIND_SPEED_RESOLUTION = 0.5f;    // m/s (mean wind)  — WIND ADDITION
static constexpr float    WIND_GUST_RESOLUTION  = 0.1f;    // m/s (gusts)      — WIND ADDITION

// ─────────────────────────────────────────────────────────────────────────────
// Wind Sensor — PR-300-FSJT-V05 (Analog Voltage Output)            WIND ADDITION
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Sensor: PR-300-FSJT-V05
 *   Range:  0 – 30 m/s
 *   Output: 0 – 5 V (linear)
 *
 * Hardware interface:
 *   The ESP32 ADC accepts 0–3.1 V (with 11 dB attenuation).
 *   A resistive voltage divider scales 0–5 V → 0–2.5 V:
 *     R1 = 10 kΩ (sensor output to ADC input)
 *     R2 = 10 kΩ (ADC input to GND)
 *     Ratio = R2 / (R1 + R2) = 0.5
 *   Keeping below 2.5 V avoids the ESP32 ADC non-linearity zone.
 *
 * Calibration:
 *   wind_speed = (adc_voltage / DIVIDER_RATIO) * (MAX_SPEED / MAX_VOLTAGE)
 */
static constexpr int      WIND_ADC_PIN          = 34;      // GPIO34 (ADC1_CH6, input-only)
static constexpr float    WIND_DIVIDER_RATIO    = 0.5f;    // R2 / (R1 + R2)
static constexpr float    WIND_SENSOR_MAX_V     = 5.0f;    // Sensor full-scale voltage
static constexpr float    WIND_SENSOR_MAX_MPS   = 30.0f;   // Sensor full-scale m/s
static constexpr float    WIND_MPS_PER_VOLT     = WIND_SENSOR_MAX_MPS / WIND_SENSOR_MAX_V; // 6.0

/** Zero-wind offset in m/s — subtracted after conversion to correct ADC/divider bias. */
static constexpr float    WIND_ZERO_OFFSET_MPS  = 1.7f;

/** ESP32 ADC full-scale voltage with 11 dB attenuation (fallback if no eFuse cal). */
static constexpr float    ADC_REF_VOLTAGE       = 2.6f;    // ~2600 mV measurable range
static constexpr uint16_t ADC_MAX_COUNTS        = 4095;    // 12-bit ADC

// ─────────────────────────────────────────────────────────────────────────────
// WMO Wind Sampling Parameters (Chapter 5, §5.8.3)                 WIND ADDITION
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Sampling at 4 Hz (every 250 ms) is the WMO-recommended rate for optimal
 * gust detection with overlapping 3-second running averages.
 *
 * WMO §5.8.3: "it is desirable to sample the filtered wind signal every
 * 0.25 s (frequency 4 Hz)"
 */
static constexpr uint32_t WIND_SAMPLE_INTERVAL_MS  = 250;
static constexpr uint32_t WIND_SAMPLE_FREQ_HZ      = 4;

/**
 * Gust = maximum 3-second running average (WMO §5.8.2).
 * At 4 Hz, a 3-second window = 12 samples.
 */
static constexpr uint32_t WIND_GUST_WINDOW_SEC     = 3;
static constexpr uint32_t WIND_GUST_WINDOW_SAMPLES = WIND_GUST_WINDOW_SEC * WIND_SAMPLE_FREQ_HZ;

/** 2-minute averaging window. At 4 Hz = 480 samples. */
static constexpr uint32_t WIND_AVG_2MIN_SEC        = 120;
static constexpr uint32_t WIND_AVG_2MIN_SAMPLES    = WIND_AVG_2MIN_SEC * WIND_SAMPLE_FREQ_HZ;

/** 10-minute averaging window. At 4 Hz = 2400 samples. */
static constexpr uint32_t WIND_AVG_10MIN_SEC       = 600;
static constexpr uint32_t WIND_AVG_10MIN_SAMPLES   = WIND_AVG_10MIN_SEC * WIND_SAMPLE_FREQ_HZ;

/** ADC readings to average per single 250 ms sample (reduces noise). */
static constexpr uint8_t  WIND_ADC_OVERSAMPLE      = 16;

// ─────────────────────────────────────────────────────────────────────────────
// Task Stack Sizes & Priorities
// ─────────────────────────────────────────────────────────────────────────────
/** Wind acquisition task — highest priority on APP_CORE (4 Hz timing). */
static constexpr uint32_t WIND_TASK_STACK       = 8192;    // WIND ADDITION
static constexpr UBaseType_t WIND_TASK_PRIO     = 6;       // WIND ADDITION

/** Sensor sampling task — highest priority on APP_CORE for timing accuracy. */
static constexpr uint32_t SENSOR_TASK_STACK     = 4096;
static constexpr UBaseType_t SENSOR_TASK_PRIO   = 5;

/** Averaging / post-processing task — runs after each window completes. */
static constexpr uint32_t AVG_TASK_STACK        = 4096;
static constexpr UBaseType_t AVG_TASK_PRIO      = 4;

/** Comms service task (placeholder). */
static constexpr uint32_t COMMS_TASK_STACK      = 4096;
static constexpr UBaseType_t COMMS_TASK_PRIO    = 3;

/** System watchdog / health task. */
static constexpr uint32_t HEALTH_TASK_STACK     = 2048;
static constexpr UBaseType_t HEALTH_TASK_PRIO   = 1;

// ─────────────────────────────────────────────────────────────────────────────
// Sensor Fault Tolerance
// ─────────────────────────────────────────────────────────────────────────────
/** Maximum consecutive read failures before marking sensor as faulted. */
static constexpr uint8_t  MAX_CONSECUTIVE_FAILS = 3;

/** Retry delay after a failed I2C transaction (ms). */
static constexpr uint32_t I2C_RETRY_DELAY_MS    = 50;

/** Maximum I2C retries per single read attempt. */
static constexpr uint8_t  I2C_MAX_RETRIES       = 2;

// ─────────────────────────────────────────────────────────────────────────────
// Serial Debug
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint32_t SERIAL_BAUD           = 115200;

/** Set to 0 to disable verbose per-sample debug prints. */
#define DEBUG_VERBOSE  1

#endif // CONFIG_H