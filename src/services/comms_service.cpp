/**
 * @file comms_service.cpp
 * @brief Communications service implementation
 *
 * Default transport: Serial JSON output.
 * Replace transmit() with your production transport.
 */

#include "comms_service.h"
#include <Arduino.h>

CommsService::CommsService(QueueHandle_t recordQueue)
    : _recordQueue(recordQueue)
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

    Serial.println(F("[Comms] Service started on Core 1"));
    return true;
}

void CommsService::taskEntry(void* param) {
    static_cast<CommsService*>(param)->taskLoop();
}

void CommsService::taskLoop() {
    AveragedRecord record;

    for (;;) {
        // Block until a new averaged record is available (infinite wait)
        if (xQueueReceive(_recordQueue, &record, portMAX_DELAY) == pdPASS) {
            if (!transmit(record)) {
                Serial.println(F("[Comms] Transmit failed — record will be retried or lost"));
                // TODO: implement retry buffer / persistent queue
            }
        }
    }
}

bool CommsService::transmit(const AveragedRecord& rec) {
    /**
     * DEFAULT: Print a JSON-formatted record to Serial.
     *
     * Replace this method body with your transport logic:
     *
     *   WiFi/MQTT example:
     *     mqttClient.publish("weather/data", jsonPayload);
     *
     *   LoRaWAN example:
     *     lorawan.send(port, payload, payloadLen);
     *
     *   SD card example:
     *     file.println(csvLine);
     */

    // Build a simple JSON payload
    char json[512];
    snprintf(json, sizeof(json),
        "{"
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

    Serial.println(F("[Comms] TX:"));
    Serial.println(json);

    return true;  // Serial always "succeeds"
}
