# Project Report (Engineering Clinics II)

**Title:** A Handheld Camera-Free 3D Surface Scanner Using Multizone Optical Time-of-Flight (ToF) Sensing and Inertial Orientation Compensation

## Abstract
Common 3D surface-profiling approaches rely on cameras, costly LiDAR or motorised gantries. Cameras raise privacy concerns and degrade in poor lighting. This project builds a compact, camera-free handheld scanner around an 8x8 multizone ToF sensor (VL53L5CX, 940 nm) and an inertial sensor, controlled by an ESP32-S3. The firmware streams 64-zone depth frames at 15 Hz together with accelerometer-derived pitch and roll over a Wi-Fi WebSocket link. A Python application converts each zone's range and viewing angle into 3D coordinates, rotates them using pitch and roll, and displays a live point cloud and a triangulated surface mesh. It also reports bounding-box dimensions and exports STL files.

## Problem statement
- Cameras raise privacy issues and fail in poor lighting.
- Pan-tilt scanners are bulky and mechanically rigid.
- Commercial LiDAR and depth cameras are expensive for embedded and educational use.

## System overview
1. **Sensing:** VL53L5CX 8x8 depth grid at 15 Hz; MPU accelerometer for tilt.
2. **Firmware (ESP32-S3):** Reads both sensors over I2C, computes pitch and roll, streams JSON over WebSocket, shows status on an OLED.
3. **PC software (Python):** Spherical-to-Cartesian conversion, rotation by pitch and roll, depth heatmap, point cloud, mesh generation (grid triangulation with depth-jump rejection, loop subdivision, Laplacian smoothing), STL export.

## Scope and limitations
The system is **orientation-aware**, not a full 6-DOF tracker: it estimates pitch and roll only, with no yaw and no translation estimate. Results are therefore best for single-pose surface capture. Multi-pose registration is future work.

## Status
See the roadmap in the main [README](../README.md).

## Conclusion
*[Write after final testing, using measured results.]*
