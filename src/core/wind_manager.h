/**
 * @file wind_manager.h
 * @brief WMO-compliant wind speed acquisition and averaging
 *
 * Handles the PR-300-FSJT-V05 cup anemometer (0–30 m/s, 0–5 V analog).
 *
 * Architecture — runs independently from the I2C SensorManager:
 *
 *   ┌──────────────────────────────────────────────────────────┐
 *   │ FreeRTOS SW Timer (250 ms / 4 Hz)                       │
 *   │   └── Task Notification → windTask (Core 0, P:6)        │
 *   │         ├── ADC multi-sample → voltage → m/s            │
 *   │         ├── Push into 3-s circular buffer (12 samples)  │
 *   │         ├── Compute overlapping 3-s running average      │
 *   │         ├── Track peak gust (max 3-s avg in window)     │
 *   │         ├── Accumulate into 2-min & 10-min sums         │
 *   │         └── Every 10 min: produce WindRecord → Queue    │
 *   └──────────────────────────────────────────────────────────┘
 *
 * WMO Compliance (WMO-No. 8, 2024):
 *   Annex 1.A §5.1  — Mean speed: 2-min and 10-min averaging
 *   Annex 1.A §5.3  — Gust: max 3-s running average
 *   Chapter 5 §5.8.3 — 4 Hz sampling, overlapping 3-s windows every 0.25 s
 *
 * Hardware:
 *   ESP32 GPIO34 (ADC1_CH6) ← voltage divider (10k/10k) ← sensor 0–5V
 */

#ifndef WIND_MANAGER_H
#define WIND_MANAGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>

#include "../config.h"
#include "data_types.h"

class WindManager {
public:
    WindManager();

    bool start();

    QueueHandle_t getOutputQueue() const { return _outputQueue; }
    bool getLatestRecord(WindRecord& record) const;
    SensorStatus status() const { return _status; }
    uint32_t windowsCompleted() const { return _totalWindows; }

private:
    static void windTaskEntry(void* param);
    static void timerCallback(TimerHandle_t timer);

    void windTaskLoop();
    float readWindSpeedMps();
    void  processSample(float speed_mps);
    void  finaliseWindow();
    void  resetWindow();

    // FreeRTOS primitives
    TaskHandle_t      _taskHandle  = nullptr;
    TimerHandle_t     _timer       = nullptr;
    QueueHandle_t     _outputQueue = nullptr;
    SemaphoreHandle_t _dataMutex   = nullptr;

    // 3-second circular buffer for gust detection (12 samples at 4 Hz)
    float    _gustBuf[WIND_GUST_WINDOW_SAMPLES];
    uint16_t _gustBufIdx;
    uint16_t _gustBufCount;

    // 2-minute circular buffer for running 2-min mean
    float    _twoMinBuf[WIND_AVG_2MIN_SAMPLES];
    uint16_t _twoMinBufIdx;
    uint16_t _twoMinBufCount;

    // 10-minute window accumulators
    double   _tenMinSum;
    uint32_t _tenMinCount;
    float    _peakGust;
    float    _minSpeed;
    uint32_t _windowStartMs;

    // Latest completed record
    WindRecord _latestRecord;
    bool       _hasValidRecord = false;

    // Status
    SensorStatus _status;
    uint32_t     _totalWindows;
    uint8_t      _consecutiveFails;
};

#endif // WIND_MANAGER_H
