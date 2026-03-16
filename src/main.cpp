#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setTimeOut(50);

    Serial.println("\n--- I2C Scanner ---");
    Serial.printf("SDA: GPIO%d, SCL: GPIO%d\n\n", SDA_PIN, SCL_PIN);

    int devicesFound = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("Device found at 0x%02X\n", addr);
            devicesFound++;
        }
    }

    Serial.printf("\nScan complete. %d device(s) found.\n", devicesFound);
}

void loop() {
    delay(5000);
}
