# Project Report

**Title:** A Handheld Camera-Free 3D Surface Scanner Using Multizone Optical Time-of-Flight (ToF) and Inertial Sensor Fusion

## Abstract
Traditional 3D surface profiling relies on cameras, costly LiDAR, or rigid mechanical gantries. Camera-based solutions raise privacy concerns, degrade in low light and need heavy image processing. This project presents a compact, camera-free handheld scanner that combines an 8x8 multizone ToF sensor (VL53L5CX, 940 nm) with an IMU (MPU-6050), orchestrated by an ESP32-S3. Depth zones are converted to 3D coordinates and rotated into a global frame using IMU orientation. The resulting point cloud is streamed over WebSockets to a PC, where a surface reconstruction step builds a polygon mesh.

## Problem statement
- Cameras raise privacy issues and fail in poor lighting.
- Pan-tilt scanners are bulky and mechanically rigid.
- Commercial LiDAR and depth cameras are expensive for embedded and educational use.

## Proposed solution
A handheld wand that measures depth with 64-zone ToF sensing, tracks its own orientation with an IMU, fuses both on an ESP32-S3, and renders the result on a PC.

## Methodology
1. Hardware assembly and I2C driver bring-up
2. Coordinate transformation and IMU fusion
3. Wireless streaming
4. Point cloud filtering and mesh reconstruction
5. Enclosure, calibration and testing

## Timeline
| Month | Goals |
|---|---|
| 1 | Hardware assembly, drivers, 8x8 readout, power and display |
| 2 | Coordinate maths, IMU fusion, Wi-Fi streaming |
| 3 | Visualiser, mesh reconstruction, enclosure and demo |

## Budget
See [`../hardware/BOM.md`](../hardware/BOM.md).

## Conclusion
*[Write after testing, using your real results.]*
