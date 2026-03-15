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

// ─────────────────────────────────────────────────────────────────────────────
// Task Stack Sizes & Priorities
// ─────────────────────────────────────────────────────────────────────────────
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
