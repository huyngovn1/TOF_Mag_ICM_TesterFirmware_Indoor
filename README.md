# Drone Flight Controller (FC) Firmware

## Overview
<img width="651" height="257" alt="image" src="https://github.com/user-attachments/assets/2931754f-a28e-42d0-9da2-5e5a1d4957d4" />

Embedded firmware development for a **Drone Flight Controller (FC)**, integrating multiple sensors for motion monitoring, altitude measurement, distance detection, and flight control applications.

The project focuses on sensor communication, real-time data acquisition, debugging, and the development of a multi-sensor flight control system.

## Features

- **IMU Integration:** Motion sensor initialization and status monitoring.
- **4× Time-of-Flight (ToF) Sensors:** Multi-directional distance measurement and obstacle detection.
- **BMP3 Barometric Sensor:** Atmospheric pressure and temperature measurement.
- **Magnetometer:** Raw 3-axis magnetic field data acquisition.
- **Distance Monitoring:** Additional distance data through `MTF.distance`.
- **Real-Time Debugging:** Sensor monitoring using STM32CubeIDE Live Expressions / Debug Watch.

## Sensor Integration

| Sensor / Module | Function | Data |
|---|---|---|
| IMU | Motion sensing and initialization monitoring | `IMU_Status` |
| ToF Sensor 1 | Distance measurement | `tof[1].distance_mm` |
| ToF Sensor 2 | Distance measurement | `tof[2].distance_mm` |
| ToF Sensor 3 | Distance measurement | `tof[3].distance_mm` |
| ToF Sensor 4 | Distance measurement | `tof[4].distance_mm` |
| BMP3 | Barometric pressure | `bmp3_comp_data.pressure` |
| BMP3 | Temperature measurement | `bmp3_comp_data.temperature` |
| Magnetometer | 3-axis magnetic field data | `MagRaw[0..2]` |
| MTF | Additional distance measurement | `MTF.distance` |

## Debugging & Sensor Data

The following values were captured during a firmware debugging session.

| Variable | Sample Value | Unit |
|---|---:|---|
| `tof[1].distance_mm` | 79 | mm |
| `tof[2].distance_mm` | 275 | mm |
| `tof[3].distance_mm` | 20 | mm |
| `tof[4].distance_mm` | 251 | mm |
| `bmp3_comp_data.pressure` | 101627.73 | Pa |
| `bmp3_comp_data.temperature` | 38.09 | °C |
| `MagRaw[0]` | -5235 | Raw |
| `MagRaw[1]` | 116 | Raw |
| `MagRaw[2]` | -206 | Raw |
| `MTF.distance` | 20 | Firmware-defined |
| `IMU_Status` | 126 | Status value |

**Debug Notes:**
- ToF measurements are reported in millimeters.
- BMP3 pressure and temperature values are compensated sensor readings.
- Magnetometer values represent raw X, Y, and Z axis measurements.
- `IMU_Status` is an internal firmware status indicator.
- The displayed values are sample readings and may vary depending on sensor conditions.

## Development & Debugging

**Development Environment**
- STM32CubeIDE
- Embedded C
- STM32 HAL / LL drivers (depending on firmware configuration)
- SWD debugging interface

**Development Focus**
- Sensor initialization and communication
- Multi-sensor data acquisition
- Real-time monitoring and debugging
- Sensor data processing
- Flight controller firmware development

## Project Status

**Under Development**

The current debugging snapshot demonstrates sensor data acquisition from multiple modules. Further validation is required for complete flight control operation, sensor calibration, and system reliability.

## License

License information has not yet been specified.

---

**Drone Flight Controller Firmware — Embedded Systems Development**
