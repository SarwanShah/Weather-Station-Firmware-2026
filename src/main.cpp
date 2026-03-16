/**
 * @file main.cpp
 * @brief Entry point — WMO-compliant meteorological data acquisition system
 *
 * Target:  ESP32-WROOM-32D
 * Sensors: Adafruit HDC3022 (temperature / humidity)
 *          Adafruit BMP585  (pressure / temperature)
 *          PR-300-FSJT-V05  (wind speed, 0–5V analog)
 *
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║                    SYSTEM ARCHITECTURE                          ║
 * ╠═══════════════════════════════════════════════════════════════════╣
 * ║                                                                 ║
 * ║  Core 0 (APP_CORE) — Deterministic Acquisition                  ║
 * ║  ┌──────────────────────────────────────┐                       ║
 * ║  │ SW Timer (250 ms / 4 Hz)             │                       ║
 * ║  │   └─► WindSample Task (P:6)          │                       ║
 * ║  │         ├─ ADC 16x oversample        │                       ║
 * ║  │         ├─ 3-s running avg (gust)    │                       ║
 * ║  │         ├─ 2-min & 10-min accum      │                       ║
 * ║  │         └─ @10 min: WindRecord → Q   │                       ║
 * ║  │                                      │                       ║
 * ║  │ SW Timer (5 s)                       │                       ║
 * ║  │   └─► SensorSample Task (P:5)       │                       ║
 * ║  │         ├─ HDC3022::read()           │                       ║
 * ║  │         ├─ BMP585::read()            │                       ║
 * ║  │         └─ @12th: AveragedRecord → Q │                       ║
 * ║  └──────────────────────────────────────┘                       ║
 * ║                          │                                      ║
 * ║               Output Queues (separate)                           ║
 * ║                          │                                      ║
 * ║  Core 1 (USER_CORE) — Non-Critical Services                     ║
 * ║  ┌──────────────────────────────────────┐                       ║
 * ║  │ CommsTask (P:3)                      │                       ║
 * ║  │   └─ Dequeue → JSON → Serial/MQTT   │                       ║
 * ║  │                                      │                       ║
 * ║  │ HealthTask (P:1)                     │                       ║
 * ║  │ ControlTask (P:2)  [placeholder]     │                       ║
 * ║  └──────────────────────────────────────┘                       ║
 * ║                                                                 ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 *
 * WMO Compliance (WMO-No. 8, 2024, Annex 1.A + Chapter 5):
 *   • T/P/RH sample interval:  5 s  (12 samples per 1-min window)
 *   • T/P/RH averaging window: 60 s (1 min)
 *   • Output resolution: T=0.1°C, RH=1%, P=0.1 hPa
 *   • Wind sample interval:    250 ms (4 Hz)
 *   • Wind gust:               max 3-s running average (overlapping every 250 ms)
 *   • Wind averaging:          2-min and 10-min windows
 *   • Wind resolution:         mean 0.5 m/s, gust 0.1 m/s
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
#include "core/wind_manager.h"
#include "services/comms_service.h"

// ─────────────────────────────────────────────────────────────────────────────
// Module Instances
// ─────────────────────────────────────────────────────────────────────────────
static SensorHDC3022  hdc3022(HDC3022_I2C_ADDR);
static SensorBMP585   bmp585(BMP585_I2C_ADDR);
static SensorManager  sensorMgr;
static WindManager    windMgr;
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

        // Merge wind status from the global WindManager
        h.anem_status = windMgr.status();
        h.total_wind_windows_completed = windMgr.windowsCompleted();

        Serial.println(F("──── System Health ────"));
        Serial.printf("  Uptime       : %lu s\n", h.uptime_sec);
        Serial.printf("  T/P/RH wins  : %lu\n",   h.total_windows_completed);
        Serial.printf("  Wind wins    : %lu\n",   h.total_wind_windows_completed);
        Serial.printf("  Sample fails : %lu\n",   h.total_sample_failures);
        Serial.printf("  Free heap    : %lu bytes\n", h.free_heap_bytes);
        Serial.printf("  HDC status   : %u\n", (uint8_t)h.hdc_status);
        Serial.printf("  BMP status   : %u\n", (uint8_t)h.bmp_status);
        Serial.printf("  ANEM status  : %u\n", (uint8_t)h.anem_status);
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

    // ── Step 2: Register I2C sensors ──
    // BMP585 first: its address (0x47) was confirmed on-bus by the scan.
    // Some Adafruit libraries call Wire.begin() internally, which can
    // disrupt ESP32 I2C; initialising the known-good sensor first
    // ensures at least one sensor comes up even if the other fails.
    sensorMgr.registerSensor(&bmp585);
    sensorMgr.registerSensor(&hdc3022);

    // Future I2C sensors would be registered here:
    //   sensorMgr.registerSensor(&rainGauge);
    //   sensorMgr.registerSensor(&solarRadiation);

    // ── Step 3: Start I2C acquisition engine (Core 0, 5 s interval) ──
    if (!sensorMgr.start()) {
        Serial.println(F("FATAL: SensorManager start failed. Halting."));
        for (;;) vTaskDelay(portMAX_DELAY);
    }

    // ── Step 4: Start wind acquisition (Core 0, 4 Hz / 250 ms) ──
    if (!windMgr.start()) {
        Serial.println(F("WARNING: WindManager failed to start — continuing without wind"));
    }

    // ── Step 5: Start communications service (Core 1) ──
    comms = new CommsService(sensorMgr.getOutputQueue(), windMgr.getOutputQueue());
    if (!comms->start()) {
        Serial.println(F("WARNING: Comms service failed to start"));
    }

    // ── Step 6: Start health monitor (Core 1) ──
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