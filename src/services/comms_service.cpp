/**
 * @file comms_service.cpp
 * @brief Communications service — polls both T/P/RH and wind queues
 *
 * Default transport: Serial JSON output.
 * Replace transmit() / transmitWind() with your production transport.
 */

#include "comms_service.h"
#include <Arduino.h>

CommsService::CommsService(QueueHandle_t recordQueue, QueueHandle_t windQueue)
    : _recordQueue(recordQueue)
    , _windQueue(windQueue)
{}

bool CommsService::start() {
    if (!_recordQueue) {
        Serial.println(F("[Comms] No record queue — cannot start"));
        return false;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        taskEntry,
        "CommsTask",
        COMMS_TASK_STACK,
        this,
        COMMS_TASK_PRIO,
        &_taskHandle,
        USER_CORE           // Runs on Core 1 — doesn't block sensor reads
    );

    if (ret != pdPASS) {
        Serial.println(F("[Comms] Task creation failed"));
        return false;
    }

    Serial.printf("[Comms] Service started on Core 1 (wind queue: %s)\n",
                  _windQueue ? "yes" : "no");
    return true;
}

void CommsService::taskEntry(void* param) {
    static_cast<CommsService*>(param)->taskLoop();
}

void CommsService::taskLoop() {
    AveragedRecord avgRecord;
    WindRecord     windRecord;

    for (;;) {
        // Poll the T/P/RH queue (non-blocking)
        if (xQueueReceive(_recordQueue, &avgRecord, 0) == pdPASS) {
            if (!transmit(avgRecord)) {
                Serial.println(F("[Comms] T/P/RH transmit failed"));
            }
        }

        // Poll the wind queue if available (non-blocking)
        if (_windQueue && xQueueReceive(_windQueue, &windRecord, 0) == pdPASS) {
            if (!transmitWind(windRecord)) {
                Serial.println(F("[Comms] Wind transmit failed"));
            }
        }

        // Sleep briefly to avoid busy-spinning (100 ms is fine —
        // records arrive at 1-min and 10-min intervals)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool CommsService::transmit(const AveragedRecord& rec) {
    char json[512];
    snprintf(json, sizeof(json),
        "{"
        "\"type\":\"tph\","
        "\"temp_c\":%.1f,"
        "\"temp_min\":%.1f,"
        "\"temp_max\":%.1f,"
        "\"rh_pct\":%.0f,"
        "\"press_hpa\":%.1f,"
        "\"press_min\":%.1f,"
        "\"press_max\":%.1f,"
        "\"samples\":{\"t\":%u,\"h\":%u,\"p\":%u},"
        "\"window_ms\":[%lu,%lu],"
        "\"status\":{\"hdc\":%u,\"bmp\":%u}"
        "}",
        rec.temperature_c,
        rec.temp_min,
        rec.temp_max,
        rec.humidity_pct,
        rec.pressure_hpa,
        rec.press_min,
        rec.press_max,
        rec.temp_sample_count,
        rec.hum_sample_count,
        rec.press_sample_count,
        rec.window_start_ms,
        rec.window_end_ms,
        (uint8_t)rec.hdc_status,
        (uint8_t)rec.bmp_status
    );

    Serial.println(F("[Comms] TX T/P/RH:"));
    Serial.println(json);
    return true;
}

bool CommsService::transmitWind(const WindRecord& rec) {
    char json[384];
    snprintf(json, sizeof(json),
        "{"
        "\"type\":\"wind\","
        "\"speed_10min\":%.1f,"
        "\"speed_2min\":%.1f,"
        "\"gust_3s\":%.1f,"
        "\"speed_min\":%.1f,"
        "\"samples\":%lu,"
        "\"window_ms\":[%lu,%lu],"
        "\"status\":{\"anem\":%u}"
        "}",
        rec.speed_mean_10min,
        rec.speed_mean_2min,
        rec.speed_max_3s,
        rec.speed_min,
        rec.total_samples,
        rec.window_start_ms,
        rec.window_end_ms,
        (uint8_t)rec.anem_status
    );

    Serial.println(F("[Comms] TX Wind:"));
    Serial.println(json);
    return true;
}