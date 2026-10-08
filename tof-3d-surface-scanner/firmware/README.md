# Firmware

PlatformIO project for the ESP32-S3. Build in stages, committing after each one:

1. I2C scanner (current `main.cpp`)
2. Read the VL53L5CX 8x8 grid and print it over serial
3. Read the MPU-6050 and compute orientation
4. Combine both and stream over Wi-Fi WebSocket
