# Wiring

All three I2C devices share one bus (400 kHz).

| Signal | ESP32-S3 pin | Connects to |
|---|---|---|
| SDA | GPIO 8 | VL53L5CX, MPU, OLED |
| SCL | GPIO 9 | VL53L5CX, MPU, OLED |
| 3V3 | 3V3 | VCC of all modules |
| GND | GND | GND of all modules |

I2C addresses: VL53L5CX `0x29`, OLED `0x3C`, MPU `0x68` (the firmware also tries `0x69`).

Run `firmware/i2c_scanner` to confirm all three are detected. Add your schematic or a clear wiring photo to this folder.
