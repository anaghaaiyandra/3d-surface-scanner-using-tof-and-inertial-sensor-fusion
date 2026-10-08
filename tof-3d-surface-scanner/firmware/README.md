# Firmware (Arduino IDE, ESP32-S3)

| Sketch | Purpose |
|---|---|
| `tof_wifi_stream/` | **Main firmware.** Reads the VL53L5CX (8x8 @ 15 Hz) and the MPU accelerometer, then streams JSON over a WebSocket server on port 81. |
| `i2c_scanner/` | Wiring check. Run this first if a sensor is not detected. |

## Setup
1. Install the **esp32 board package** (Boards Manager) and select **ESP32S3 Dev Module**.
2. Install these libraries (Library Manager):
   - `WebSockets` (by Markus Sattler)
   - `SparkFun VL53L5CX Arduino Library`
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
3. Open `tof_wifi_stream/tof_wifi_stream.ino` and upload.

## What the firmware does
- Starts a Wi-Fi access point named `3D_Scanner_AP` (IP `192.168.4.1`).
- Wakes the MPU and computes **pitch and roll from the accelerometer** (`atan2`). The gyroscope is not read, and yaw is not computed.
- Sends one JSON packet per ToF frame: `{"pitch":..,"roll":..,"grid":[64 distances in mm]}` (0 means no valid reading).
- Shows AP name, IP, sensor status, client status, pitch, roll and frame count on the OLED once per second.

> The default Wi-Fi password is in the source. Change it before using the scanner in public places.
