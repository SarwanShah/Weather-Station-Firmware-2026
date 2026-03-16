/**
 * @file comms_service.h
 * @brief Communications service — consumes both T/P/RH and wind records
 *
 * Runs on USER_CORE (Core 1), polling both output queues without
 * blocking sensor acquisition on Core 0.
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
     * @brief Initialise with output queues.
     * @param recordQueue  AveragedRecord queue from SensorManager.
     * @param windQueue    WindRecord queue from WindManager (may be nullptr).
     */
    CommsService(QueueHandle_t recordQueue, QueueHandle_t windQueue = nullptr);

    bool start();

private:
    static void taskEntry(void* param);
    void taskLoop();

    bool transmit(const AveragedRecord& record);
    bool transmitWind(const WindRecord& record);

    QueueHandle_t _recordQueue;
    QueueHandle_t _windQueue;
    TaskHandle_t  _taskHandle = nullptr;
};

#endif // COMMS_SERVICE_H