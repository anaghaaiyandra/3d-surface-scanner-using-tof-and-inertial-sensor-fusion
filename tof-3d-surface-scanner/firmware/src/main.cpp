// Milestone 1: I2C scanner.
// Confirms the VL53L5CX (0x29), MPU-6050 (0x68) and OLED (0x3C) are wired correctly.
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9);          // SDA, SCL (change if your board differs)
  Wire.setClock(400000);
  Serial.println("Scanning I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Device found at 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("Done. %d device(s) found.\n", found);
}

void loop() {}
