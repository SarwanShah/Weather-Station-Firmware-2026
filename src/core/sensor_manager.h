/**
 * @file sensor_manager.h
 * @brief Orchestrates WMO-compliant sensor sampling and 1-min averaging
 *
 * Architecture:
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ FreeRTOS Software Timer (5 s period)                    │
 *   │   └── Sends task notification → sampleTask              │
 *   │         └── Reads all registered ISensor instances       │
 *   │         └── Accumulates into running window              │
 *   │         └── Every 12th sample: compute 1-min average     │
 *   │               └── Sends AveragedRecord via queue         │
 *   │                     └── Downstream consumers (comms,     │
 *   │                         logging, control) receive copy   │
 *   └─────────────────────────────────────────────────────────┘
 *
 * All tasks are pinned to APP_CORE (Core 0).
 * The output queue is readable from any core.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <vector>

#include "../config.h"
#include "../hal/sensor_interface.h"
#include "data_types.h"

class SensorManager {
public:
    SensorManager();

    /**
     * @brief Register a sensor before calling start().
     * Sensors are read in registration order.
     */
    void registerSensor(ISensor* sensor);

    /**
     * @brief Initialise all registered sensors and start sampling.
     * Creates the FreeRTOS timer and sampling task on APP_CORE.
     * @return true if at least one sensor initialised successfully.
     */
    bool start();

    /**
     * @brief Get the output queue handle.
     * Consumers (comms, logging) receive AveragedRecord items from this queue.
     * Queue depth = 5 records (5 minutes of buffering if consumer stalls).
     */
    QueueHandle_t getOutputQueue() const { return _outputQueue; }

    /**
     * @brief Get the most recent averaged record (thread-safe copy).
     * @param[out] record  Destination for the latest record.
     * @return true if a valid record exists.
     */
    bool getLatestRecord(AveragedRecord& record) const;

    /**
     * @brief Get system health snapshot.
     */
    SystemHealth getHealth() const;

private:
    // ── FreeRTOS callbacks (static trampolines) ──
    static void sampleTaskEntry(void* param);
    static void timerCallback(TimerHandle_t timer);

    // ── Internal methods ──
    void sampleTaskLoop();
    void acquireSample();
    void finaliseWindow();
    void resetWindow();

    // ── Registered sensors ──
    std::vector<ISensor*> _sensors;

    // ── FreeRTOS primitives ──
    TaskHandle_t   _sampleTaskHandle  = nullptr;
    TimerHandle_t  _sampleTimer       = nullptr;
    QueueHandle_t  _outputQueue       = nullptr;
    SemaphoreHandle_t _dataMutex      = nullptr;

    // ── Accumulator for current 1-min window ──
    struct WindowAccumulator {
        float    temp_sum;
        float    hum_sum;
        float    press_sum;
        float    temp_min;
        float    temp_max;
        float    press_min;
        float    press_max;
        uint8_t  temp_count;
        uint8_t  hum_count;
        uint8_t  press_count;
        uint8_t  total_samples;
        uint32_t window_start_ms;
    };

    WindowAccumulator _accum;

    // ── Latest completed record (protected by _dataMutex) ──
    AveragedRecord _latestRecord;
    bool           _hasValidRecord = false;

    // ── Statistics ──
    uint32_t _totalWindows   = 0;
    uint32_t _totalFailures  = 0;
};

#endif // SENSOR_MANAGER_H
