# Wiring

All three I2C devices share one bus. Check your exact board's pin labels, as pins differ between ESP32-S3 boards. The pins below are the common Arduino defaults for ESP32-S3.

| Signal | ESP32-S3 pin | Connects to |
|---|---|---|
| SDA | GPIO 8 | VL53L5CX, MPU-6050, OLED |
| SCL | GPIO 9 | VL53L5CX, MPU-6050, OLED |
| 3V3 | 3V3 | VCC of all modules |
| GND | GND | GND of all modules |

Expected I2C addresses: VL53L5CX `0x29`, MPU-6050 `0x68`, SSD1306 OLED `0x3C`.

Some VL53L5CX breakouts also expose `LPn`, `INT` and `RST` pins. Check the datasheet of your breakout and note your final wiring here, with a photo in `images/`.

Put your schematic (KiCad or a hand-drawn photo) in this folder.
