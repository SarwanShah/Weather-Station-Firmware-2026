/**
 * @file comms_service.h
 * @brief Communications service placeholder
 *
 * This module will handle transmitting averaged meteorological records
 * to remote systems.  Potential transports include:
 *   - Wi-Fi (MQTT, HTTP POST, WebSocket)
 *   - LoRa / LoRaWAN
 *   - Cellular (LTE-M / NB-IoT)
 *   - SD card logging (local "comms")
 *
 * The service consumes AveragedRecord items from the SensorManager's
 * output queue and is designed to run on USER_CORE (Core 1) so that
 * network latency never blocks sensor acquisition.
 *
 * ──────────────────────────────────────────────────────────────────
 *  INTEGRATION GUIDE
 * ──────────────────────────────────────────────────────────────────
 *  1. Implement init() with your transport setup (WiFi.begin, etc.)
 *  2. Implement transmit() to serialise and send the record.
 *  3. Call start() from main.cpp after SensorManager::start().
 *  4. The task loop automatically dequeues records and calls transmit().
 * ──────────────────────────────────────────────────────────────────
 */

#ifndef COMMS_SERVICE_H
#define COMMS_SERVICE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "../config.h"
#include "../core/data_types.h"

class CommsService {
public:
    /**
     * @brief Initialise with the queue that supplies averaged records.
     * @param recordQueue  Queue handle from SensorManager::getOutputQueue().
     */
    explicit CommsService(QueueHandle_t recordQueue);

    /**
     * @brief Start the comms task on USER_CORE (Core 1).
     */
    bool start();

private:
    static void taskEntry(void* param);
    void taskLoop();

    /**
     * @brief Transmit a single averaged record.
     * Override / modify this for your chosen transport.
     * @return true if transmission succeeded.
     */
    bool transmit(const AveragedRecord& record);

    QueueHandle_t _recordQueue;
    TaskHandle_t  _taskHandle = nullptr;
};

#endif // COMMS_SERVICE_H
