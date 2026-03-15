# Meteorological Data Acquisition System

**ESP32-WROOM-32D firmware for WMO-compliant weather observation**

## WMO Compliance

Sampling and averaging conform to **WMO-No. 8 (2024)**, *Guide to Instruments and Methods of Observation*, Volume I, Annex 1.A:

| Parameter     | Sensor   | Time Constant | Sample Rate | Averaging | Resolution |
|---------------|----------|---------------|-------------|-----------|------------|
| Temperature   | HDC3022  | 20 s          | 5 s         | 1 min     | 0.1 °C     |
| Humidity (RH) | HDC3022  | 40 s          | 5 s         | 1 min     | 1 %RH      |
| Pressure      | BMP585   | 2 s           | 5 s         | 1 min     | 0.1 hPa    |

The 5-second sampling interval yields 12 samples per 1-minute window, satisfying the Nyquist criterion for all sensor time constants. The 1-minute arithmetic mean is the WMO-recommended "instantaneous" output.

## Architecture

```
Core 0 (APP_CORE) — Deterministic Acquisition
┌──────────────────────────────────────┐
│ FreeRTOS Software Timer (5 s)        │
│   └─► Task Notification              │
│         └─► SensorSample Task (P:5)  │
│               ├── HDC3022::read()    │
│               ├── BMP585::read()     │
│               ├── Accumulate sample  │
│               └── @12th: Average→Q   │
└──────────────────────────────────────┘
                │ OutputQueue (5 deep)
Core 1 (USER_CORE) — Services & User Code
┌──────────────────────────────────────┐
│ CommsTask (P:3)  — JSON/MQTT output  │
│ HealthTask (P:1) — System monitor    │
│ loop()           — Free for user     │
└──────────────────────────────────────┘
```

## Project Structure

```
firmware/
├── platformio.ini                 # PlatformIO build config
├── README.md
└── src/
    ├── config.h                   # System-wide constants & WMO parameters
    ├── main.cpp                   # Entry point, task wiring
    ├── hal/                       # Hardware Abstraction Layer
    │   ├── i2c_bus.h/.cpp         # Mutex-protected I2C bus
    │   ├── sensor_interface.h     # Abstract sensor contract
    │   ├── sensor_hdc3022.h/.cpp  # HDC3022 temp/humidity driver
    │   └── sensor_bmp585.h/.cpp   # BMP585 pressure driver
    ├── core/                      # Application core logic
    │   ├── data_types.h           # Shared structs & enums
    │   └── sensor_manager.h/.cpp  # Sampling orchestration & averaging
    └── services/                  # Pluggable services
        ├── comms_service.h/.cpp   # Communications (Serial/MQTT/LoRa)
        └── power_service.h        # Power management (placeholder)
```

## Design Principles

1. **Core Isolation**: All time-critical I2C reads run on Core 0. Core 1 is entirely free for networking, UI, OTA, and user logic.

2. **Non-Blocking**: The sampling task sleeps via `ulTaskNotifyTake()` between 5-second intervals — zero CPU burn while waiting. A FreeRTOS software timer fires the notifications.

3. **Mutex-Protected I2C**: A single mutex governs bus access, preventing collisions if future sensors or tasks share the bus.

4. **Pluggable Sensors**: New sensors implement `ISensor` and register with `SensorManager::registerSensor()`. No changes to orchestration code.

5. **Queue-Decoupled Output**: Averaged records are pushed to a FreeRTOS queue. Downstream consumers (comms, logging, control) dequeue independently.

6. **Fault Tolerance**: Each sensor tracks consecutive failures and transitions through `OK → DEGRADED → FAULTED` states. Faulted sensors get a reset attempt at the start of each window.

## Wiring

| ESP32 Pin | Function | Connects To            |
|-----------|----------|------------------------|
| GPIO 21   | I2C SDA  | HDC3022 SDA, BMP585 SDA |
| GPIO 22   | I2C SCL  | HDC3022 SCL, BMP585 SCL |
| 3V3       | Power    | Both breakout VIN       |
| GND       | Ground   | Both breakout GND       |

Both Adafruit breakouts include pull-up resistors on SDA/SCL. If using long wires (>30 cm), add external 4.7 kΩ pull-ups.

## Building

```bash
# Install PlatformIO CLI
pip install platformio

# Build
cd firmware/
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

## Adding a New Sensor

1. Create `src/hal/sensor_mynew.h/.cpp` implementing `ISensor`.
2. Add the I2C address to `config.h`.
3. Instantiate in `main.cpp` and call `sensorMgr.registerSensor(&mySensor)`.
4. If the sensor contributes new data fields, extend `RawSample` and `AveragedRecord` in `data_types.h`.

## Adding a New Communication Transport

1. Modify `CommsService::transmit()` in `comms_service.cpp`.
2. Add transport init code to `CommsService::start()` or a separate `init()` method.
3. The task already runs on Core 1 — network latency won't affect sampling.

## License

Project-specific. WMO-No. 8 content referenced under WMO publication terms.
