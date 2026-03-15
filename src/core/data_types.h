/**
 * @file data_types.h
 * @brief Shared data structures for the meteorological acquisition system
 *
 * Defines the canonical data representations used across all modules.
 * All physical quantities use SI-compatible units matching WMO conventions:
 *   Temperature  → °C   (Kelvin offset applied at reporting layer if needed)
 *   Humidity     → %RH
 *   Pressure     → hPa  (= mbar, numerically identical to mbar)
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <cstdint>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Sensor Health Status
// ─────────────────────────────────────────────────────────────────────────────
enum class SensorStatus : uint8_t {
    OK          = 0,    ///< Sensor operating normally
    WARMING_UP  = 1,    ///< Initial stabilisation period
    DEGRADED    = 2,    ///< Intermittent failures (< MAX_CONSECUTIVE_FAILS)
    FAULTED     = 3,    ///< Persistent failure — data unreliable
    NOT_PRESENT = 4     ///< Sensor not detected on bus
};

// ─────────────────────────────────────────────────────────────────────────────
// Raw Instantaneous Sample (one I2C read cycle)
// ─────────────────────────────────────────────────────────────────────────────
struct RawSample {
    float    temperature_c;     ///< HDC3022 temperature [°C]
    float    humidity_pct;      ///< HDC3022 relative humidity [%RH]
    float    pressure_hpa;      ///< BMP585  pressure [hPa]
    float    bmp_temperature_c; ///< BMP585  die temperature [°C] (diagnostic)
    uint32_t timestamp_ms;      ///< millis() at acquisition
    bool     temp_valid;        ///< true if HDC3022 read succeeded
    bool     hum_valid;         ///< true if HDC3022 RH read succeeded
    bool     press_valid;       ///< true if BMP585 read succeeded
};

// ─────────────────────────────────────────────────────────────────────────────
// 1-Minute Averaged Record (WMO output)
// ─────────────────────────────────────────────────────────────────────────────
struct AveragedRecord {
    float    temperature_c;     ///< 1-min mean temperature [°C]
    float    humidity_pct;      ///< 1-min mean relative humidity [%RH]
    float    pressure_hpa;      ///< 1-min mean pressure [hPa]
    float    temp_min;          ///< Minimum temperature in window [°C]
    float    temp_max;          ///< Maximum temperature in window [°C]
    float    press_min;         ///< Minimum pressure in window [hPa]
    float    press_max;         ///< Maximum pressure in window [hPa]
    uint8_t  temp_sample_count; ///< Valid temperature samples in window
    uint8_t  hum_sample_count;  ///< Valid humidity samples in window
    uint8_t  press_sample_count;///< Valid pressure samples in window
    uint32_t window_start_ms;   ///< millis() at window start
    uint32_t window_end_ms;     ///< millis() at window close
    SensorStatus hdc_status;    ///< HDC3022 health at window close
    SensorStatus bmp_status;    ///< BMP585  health at window close
};

// ─────────────────────────────────────────────────────────────────────────────
// System Health Snapshot
// ─────────────────────────────────────────────────────────────────────────────
struct SystemHealth {
    uint32_t     uptime_sec;
    uint32_t     total_windows_completed;
    uint32_t     total_sample_failures;
    SensorStatus hdc_status;
    SensorStatus bmp_status;
    uint32_t     free_heap_bytes;
    float        cpu_temperature_c;  ///< ESP32 internal (if available)
};

// ─────────────────────────────────────────────────────────────────────────────
// Utility: Round to WMO reporting resolution
// ─────────────────────────────────────────────────────────────────────────────
inline float roundToResolution(float value, float resolution) {
    return roundf(value / resolution) * resolution;
}

#endif // DATA_TYPES_H
