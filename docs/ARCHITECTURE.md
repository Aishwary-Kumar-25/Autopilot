# Autopilot Architecture Specification

## Overview

The **E-Club Autopilot** is a modular, high-reliability flight stack and robotics autopilot designed from scratch. It solves a primary vulnerability of traditional monolithic flight controller boards: **on-board soldered sensors that cannot be replaced when they fail or burn out**.

The codebase is engineered around three foundational pillars:
1. **Plug-and-Play Swappable Sensors**: Auto-discovered over standard serial buses (I2C/SPI), continuously monitored for health, with real-time failover and runtime hot-swap capability.
2. **Universal Sensor Fusion Engine**: A vehicle-agnostic state estimator combining 6-DOF/9-DOF Madgwick attitude estimation and 3rd-order baro-inertial complementary altitude/velocity estimation.
3. **Modular Vehicle Control & Mixing**: Distinct control pipelines for **UAVs (Multirotors)**, **UGVs (Ground Vehicles/Rovers)**, and **ROVs (Underwater Submarines)** running on top of identical sensor fusion states.

---

## High-Level Architecture Diagram

```
+---------------------------------------------------------------------------------+
|                                 APPLICATION LAYER                               |
|   +---------------------+   +---------------------+   +---------------------+   |
|   |    UAV Multicopter  |   |  UGV Ground Rover   |   |   ROV Submarine     |   |
|   |  (Quad-X PID Loop)  |   | (Ackermann/Skid-St) |   | (ArduSub 6-DOF ROV) |   |
|   +----------+----------+   +----------+----------+   +----------+----------+   |
|              |                         |                         |              |
|   +----------v-------------------------v-------------------------v----------+   |
|   |                       VehicleBase Abstract Interface                    |   |
|   |         - User Input Mapping    - Arming & Pre-Arm Safety Checks        |   |
|   +------------------------------------+------------------------------------+   |
+----------------------------------------|----------------------------------------+
                                         |
+----------------------------------------v----------------------------------------+
|                               ACTUATOR MIXER LAYER                              |
|   +-------------------------------------------------------------------------+   |
|   | UAVQuadMixer (4 ESCs) | UGVMixer (Steer+Throttle) | ROVThrusterMixer    |   |
|   | Normalized Demands    --> Physical PWM Microsecond Commands (1000-2000us)|   |
|   +------------------------------------+------------------------------------+   |
+----------------------------------------|----------------------------------------+
                                         |
+----------------------------------------v----------------------------------------+
|                      UNIVERSAL SENSOR FUSION ENGINE (AHRS)                      |
|   +-------------------------------------------------------------------------+   |
|   | - 6/9-DOF Quaternion Madgwick Filter (Attitude: Roll, Pitch, Yaw)       |   |
|   | - 3rd-Order Baro-Inertial Estimator (Altitude Z, Vertical Speed Vz)     |   |
|   | - Earth NED Coordinate Transformations & GPS Velocity Integration       |   |
|   |   ==> Outputs Unified VehicleState                                      |   |
|   +------------------------------------+------------------------------------+   |
+----------------------------------------|----------------------------------------+
                                         |
+----------------------------------------v----------------------------------------+
|                      PLUG-AND-PLAY SENSOR SUBSYSTEM                             |
|   +-------------------------------------------------------------------------+   |
|   | SensorManager: Dynamic Bus Scanner & Health Supervisor                  |   |
|   | - Auto-discovery on boot & runtime hot-swap recovery                    |   |
|   | - Active/Backup failover logic                                          |   |
|   | Drivers: MPU6050, ICM-20689, BMP280, QMC5883L, NMEA-GPS                 |   |
|   +------------------------------------+------------------------------------+   |
+----------------------------------------|----------------------------------------+
                                         |
+----------------------------------------v----------------------------------------+
|                    HARDWARE ABSTRACTION LAYER (HAL)                             |
|   +-------------------------------------------------------------------------+   |
|   | I2CDevice | SPIDevice | UARTDevice | PWMOutput | High-Resolution Time   |   |
|   | - STM32HAL: Direct STM32 Hardware Timers (TIM1/TIM4), I2C, SPI, UART    |   |
|   | - PosixHAL: Host Linux Simulation, Unit Tests, and Mock Bus Injection   |   |
|   +-------------------------------------------------------------------------+   |
+---------------------------------------------------------------------------------+
```

---

## Directory Organization

- `hal/`: Hardware Abstraction Layer interfaces and platform targets:
  - `hal.hpp`: Base peripheral interfaces.
  - `stm32/`: STM32 target implementations (STM32F4/F7/H7 timers, PWM channels, I2C/SPI/UART).
  - `posix/`: Linux/POSIX simulation platform for unit testing and SIL.
- `drivers/`: Modular plug-and-play sensor drivers:
  - `sensor_interface.hpp`: Unified abstract interfaces for IMUs, Barometers, Magnetometers, and GPS.
  - `sensor_manager.hpp/cpp`: Bus scanner, dynamic registry, health supervisor, and hot-swap recovery.
  - `imu/`: InvenSense MPU6050 and ICM-20689 drivers.
  - `baro/`: Bosch BMP280 / BME280 high-precision barometric pressure driver.
  - `mag/`: QMC5883L 3-axis digital compass driver.
  - `gps/`: NMEA / UBX GPS parser driver.
- `fusion/`: Universal, vehicle-agnostic state estimation:
  - `ahrs_interface.hpp/cpp`: Top-level sensor fusion interface.
  - `madgwick.hpp/cpp`: Quaternion-based gradient descent attitude filter.
  - `complementary.hpp/cpp`: 3rd-order baro-inertial altitude and climb-rate estimator.
  - `state_types.hpp`: Unified `VehicleState` struct.
- `math/`: Fast embedded 3D math (Vectors, Quaternions, Matrices, Low-Pass Filters, PID controllers).
- `control/`: Arming logic, pre-arm safety validators, and actuator mixing interfaces.
- `vehicles/`: Vehicle-specific flight/motion dynamics:
  - `uav/`: Multicopter Quad-X cascaded rate/angle PIDs, altitude hold, and motor mixer.
  - `ugv/`: Rover Ackermann and Skid-Steer steering PIDs, speed hold, and wheel mixers.
  - `rov/`: Underwater Submarine 6-DOF thruster matrix and depth hold inspired by ArduSub.
- `system/`: Real-time cyclic task scheduler and MAVLink-ready telemetry stream.
- `tests/`: Automated unit tests covering sensor hot-swap, attitude convergence, and actuator outputs.
