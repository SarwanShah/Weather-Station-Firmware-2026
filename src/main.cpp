/**
 * @file main.cpp
 * @brief Entry point — WMO-compliant meteorological data acquisition system
 *
 * Target:  ESP32-WROOM-32D
 * Sensors: Adafruit HDC3022 (temperature / humidity)
 *          Adafruit BMP585  (pressure / temperature)
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║                    SYSTEM ARCHITECTURE                          ║
 * ╠═══════════════════════════════════════════════════════════════════╣
 * ║                                                                 ║
 * ║  Core 0 (APP_CORE) — Deterministic Acquisition                  ║
 * ║  ┌──────────────────────────────────────┐                       ║
 * ║  │ Software Timer (5 s)                 │                       ║
 * ║  │   └─► Task Notification              │                       ║
 * ║  │         └─► SensorSample Task (P:5)  │                       ║
 * ║  │               ├─ HDC3022::read()     │                       ║
 * ║  │               ├─ BMP585::read()      │                       ║
 * ║  │               ├─ Accumulate          │                       ║
 * ║  │               └─ Every 12th sample:  │                       ║
 * ║  │                    Average → Queue   │                       ║
 * ║  └──────────────────────────────────────┘                       ║
 * ║                          │                                      ║
 * ║                    OutputQueue (5 deep)                          ║
 * ║                          │                                      ║
 * ║  Core 1 (USER_CORE) — Non-Critical Services                     ║
 * ║  ┌──────────────────────────────────────┐                       ║
 * ║  │ CommsTask (P:3)                      │                       ║
 * ║  │   └─ Dequeue → JSON → Serial/MQTT   │                       ║
 * ║  │                                      │                       ║
 * ║  │ HealthTask (P:1)   [placeholder]     │                       ║
 * ║  │ ControlTask (P:2)  [placeholder]     │                       ║
 * ║  └──────────────────────────────────────┘                       ║
 * ║                                                                 ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 *
 * WMO Compliance (WMO-No. 8, 2024, Annex 1.A):
 *   • Sample interval:  5 s  (12 samples per window)
 *   • Averaging window: 60 s (1 min)
 *   • Output resolution: T=0.1°C, RH=1%, P=0.1 hPa
 *
 * Dependencies (PlatformIO / Arduino):
 *   - Adafruit_HDC302x
 *   - Adafruit_BMP581  (covers BMP585)
 *   - Adafruit_Sensor
 *   - Adafruit_BusIO
 */

#include <Arduino.h>
#include "config.h"
#include "hal/i2c_bus.h"
#include "hal/sensor_hdc3022.h"
#include "hal/sensor_bmp585.h"
#include "core/sensor_manager.h"
#include "services/comms_service.h"

// ─────────────────────────────────────────────────────────────────────────────
// Module Instances
// ─────────────────────────────────────────────────────────────────────────────
static SensorHDC3022  hdc3022(HDC3022_I2C_ADDR);
static SensorBMP585   bmp585(BMP585_I2C_ADDR);
static SensorManager  sensorMgr;
static CommsService*  comms = nullptr;   // Heap-allocated after queue is ready

// ─────────────────────────────────────────────────────────────────────────────
// Health Monitor Task (runs on USER_CORE, low priority)
// ─────────────────────────────────────────────────────────────────────────────
static void healthTask(void* param) {
    SensorManager* mgr = static_cast<SensorManager*>(param);
    const TickType_t interval = pdMS_TO_TICKS(30000);  // Every 30 s

    for (;;) {
        vTaskDelay(interval);

        SystemHealth h = mgr->getHealth();

        Serial.println(F("──── System Health ────"));
        Serial.printf("  Uptime       : %lu s\n", h.uptime_sec);
        Serial.printf("  Windows done : %lu\n",   h.total_windows_completed);
        Serial.printf("  Sample fails : %lu\n",   h.total_sample_failures);
        Serial.printf("  Free heap    : %lu bytes\n", h.free_heap_bytes);
        Serial.printf("  HDC status   : %u\n", (uint8_t)h.hdc_status);
        Serial.printf("  BMP status   : %u\n", (uint8_t)h.bmp_status);
        Serial.println(F("───────────────────────"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup() — called once on Core 1 by Arduino framework
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) { /* wait up to 3 s for USB-CDC */ }

    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════════════════╗"));
    Serial.println(F("║   Meteorological Data Acquisition System             ║"));
    Serial.println(F("║   WMO-No.8 Compliant | ESP32-WROOM-32D              ║"));
    Serial.println(F("╚═══════════════════════════════════════════════════════╝"));
    Serial.printf("  Build: %s %s\n", __DATE__, __TIME__);
    Serial.printf("  SDK  : %s\n", ESP.getSdkVersion());
    Serial.printf("  CPU  : %lu MHz  |  Flash: %lu bytes  |  Heap: %lu bytes\n",
                  ESP.getCpuFreqMHz(),
                  ESP.getFlashChipSize(),
                  ESP.getFreeHeap());
    Serial.println();

    // ── Step 1: Initialise I2C bus ──
    if (!g_i2c.begin()) {
        Serial.println(F("FATAL: I2C bus init failed. Halting."));
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    // ── I2C Bus Scan (diagnostic) ──
    Serial.println(F("[I2C] Scanning bus..."));
    uint8_t devicesFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (g_i2c.probe(addr)) {
            Serial.printf("[I2C]   Device found at 0x%02X\n", addr);
            devicesFound++;
        }
    }
    Serial.printf("[I2C] Scan complete — %u device(s) found\n\n", devicesFound);

    // ── Step 2: Register sensors ──
    sensorMgr.registerSensor(&hdc3022);
    sensorMgr.registerSensor(&bmp585);

    // Future sensors would be registered here:
    //   sensorMgr.registerSensor(&windSensor);
    //   sensorMgr.registerSensor(&rainGauge);
    //   sensorMgr.registerSensor(&solarRadiation);

    // ── Step 3: Start acquisition engine (Core 0) ──
    if (!sensorMgr.start()) {
        Serial.println(F("FATAL: SensorManager start failed. Halting."));
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    // ── Step 4: Start communications service (Core 1) ──
    comms = new CommsService(sensorMgr.getOutputQueue());
    if (!comms->start()) {
        Serial.println(F("WARNING: Comms service failed to start"));
        // Non-fatal — acquisition continues, data accumulates in queue
    }

    // ── Step 5: Start health monitor (Core 1) ──
    xTaskCreatePinnedToCore(
        healthTask,
        "HealthMon",
        HEALTH_TASK_STACK,
        &sensorMgr,
        HEALTH_TASK_PRIO,
        nullptr,
        USER_CORE
    );

    Serial.println(F("\n[Main] All systems started. Acquisition running.\n"));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop() — Arduino's default loop runs on Core 1
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    /**
     * The loop() task is deliberately idle.  All work is done in
     * dedicated FreeRTOS tasks with explicit priorities and core
     * affinity.
     *
     * This core (Core 1 / USER_CORE) is available for:
     *   - OTA firmware updates
     *   - Web server / configuration portal
     *   - User-defined control logic
     *   - Display driver (e.g., e-ink status screen)
     *
     * Yielding with a long delay ensures loop() consumes no CPU
     * while keeping the Arduino watchdog happy.
     */
    vTaskDelay(pdMS_TO_TICKS(10000));
}
