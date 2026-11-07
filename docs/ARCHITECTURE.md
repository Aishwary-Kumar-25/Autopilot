# Autopilot Architecture Specification

## Overview

The **E-Club Autopilot** is a modular, high-reliability flight stack and robotics autopilot designed from scratch. It solves key vulnerabilities found in traditional monolithic flight controller boards (like ArduPilot and early Pixhawks):
1. **Plug-and-Play Swappable Sensors**: Replaced noise-prone, lockup-vulnerable I2C with **CAN Bus (DroneCAN)** differential signaling as the primary external sensor bus, with I2C/SPI maintained for bench development.
2. **Zero-Allocation Pub/Sub Middleware**: Decouples data producers from consumers. Allows a **single unified firmware binary** to switch dynamically between UAV, UGV, and ROV vehicle profiles at runtime without reflashing.
3. **Modular Compile-Time Pruning**: Compile only the drivers needed for the target vehicle, eliminating multi-minute build times and binary bloat.
4. **Universal Sensor Fusion Engine**: A vehicle-agnostic state estimator combining 6-DOF/9-DOF Madgwick attitude estimation and 3rd-order baro-inertial complementary altitude/velocity estimation.

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
|   |                Zero-Allocation Pub/Sub Middleware (system/pubsub.hpp)   |   |
|   |   Topic<VehicleState> | Topic<ControlDemands> | Topic<ActuatorOutputs>  |   |
|   +------------------------------------+------------------------------------+   |
+----------------------------------------|----------------------------------------+
                                         |
+----------------------------------------v----------------------------------------+
|                               ACTUATOR MIXER LAYER                              |
|   +-------------------------------------------------------------------------+   |
|   | ActuatorMatrixMixer (Generalized Control Allocation Matrix B * tau)      |   |
|   | - 2 to 16 Actuators: 3 (Budget), 4 (Basic/Quad), 6 (Vectored/Hexa),     |   |
|   |   8 (Decoupled 6-DOF AUV/Octa), up to 16 channels                       |   |
|   | - Dynamic Interface: Choose number of thrusters/rotors at runtime       |   |
|   | - Desaturation & Dynamic Scaling (prevents rollover/loss of authority) |   |
|   | - Outputs: 16-ch PWM (1000-2000us) + DroneCAN ESC RawCommand (DSDL 1030)|   |
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
|   | - DroneCAN Node: Auto-discovery on CAN bus (RawIMU, Baro, Mag, GPS)     |   |
|   | - I2C/SPI Scanner: Auto-discovery on boot & runtime hot-swap recovery   |   |
|   | Drivers: DroneCAN Node, MPU6050, ICM-20689, BMP280, QMC5883L, NMEA-GPS  |   |
|   +------------------------------------+------------------------------------+   |
+----------------------------------------|----------------------------------------+
                                         |
+----------------------------------------v----------------------------------------+
|                    HARDWARE ABSTRACTION LAYER (HAL)                             |
|   +-------------------------------------------------------------------------+   |
|   | CANDevice | I2CDevice | SPIDevice | UARTDevice | PWMOutput | Time       |   |
|   | - STM32HAL: Direct STM32 Hardware Timers (TIM1/TIM4), bxCAN/FDCAN, I2C  |   |
|   | - PosixHAL: Host Linux Simulation, Unit Tests, and Mock Bus Injection   |   |
|   +-------------------------------------------------------------------------+   |
+---------------------------------------------------------------------------------+
```

---

## Directory Organization

- `hal/`: Hardware Abstraction Layer interfaces and platform targets:
  - `hal.hpp`: Base peripheral interfaces (including `CANDevice` and `CANFrame`).
  - `stm32/`: STM32 target implementations (TIM1/TIM4 PWM, bxCAN, I2C, SPI, UART).
  - `posix/`: Linux/POSIX simulation platform with frame injection for unit tests.
- `drivers/`: Modular plug-and-play sensor drivers:
  - `can/`: **DroneCAN** protocol engine (`dronecan_types.hpp`), node dispatcher (`dronecan_node.hpp/cpp`), and CAN sensor drivers (`dronecan_sensors.hpp/cpp`).
  - `sensor_interface.hpp`: Unified abstract interfaces for IMUs, Barometers, Magnetometers, and GPS.
  - `sensor_manager.hpp/cpp`: CAN and I2C/SPI bus scanner, dynamic registry, health supervisor, and hot-swap recovery.
  - `imu/`: InvenSense MPU6050 and ICM-20689 drivers.
  - `baro/`: Bosch BMP280 / BME280 barometric pressure driver.
  - `mag/`: QMC5883L 3-axis digital compass driver.
  - `gps/`: NMEA / UBX GPS parser driver.
- `fusion/`: Universal, vehicle-agnostic state estimation:
  - `ahrs_interface.hpp/cpp`: Top-level sensor fusion interface.
  - `madgwick.hpp/cpp`: Quaternion-based gradient descent attitude filter.
  - `complementary.hpp/cpp`: 3rd-order baro-inertial altitude and climb-rate estimator.
  - `state_types.hpp`: Unified `VehicleState` struct.
- `math/`: Fast embedded 3D math (Vectors, Quaternions, Matrices, Low-Pass Filters, PID controllers).
- `control/`: Arming logic, pre-arm safety validators, and `ActuatorMatrixMixer` (generalized 2-16 channel control allocation, dynamic thruster selection, and DroneCAN ESC support). See [ACTUATOR_MATRIX_MIXING.md](file:///home/aishwary/SRSahooProject/Autopilot_eclub/docs/ACTUATOR_MATRIX_MIXING.md) for full geometry mathematics.
- `vehicles/`: Vehicle-specific flight/motion dynamics (UAV Quad-X, UGV Rover, ROV Submarine).
- `system/`:
  - `pubsub.hpp`: **Zero-allocation Publish/Subscribe middleware**.
  - `scheduler.hpp/cpp`: Multi-rate cyclic executor.
  - `telemetry.hpp/cpp`: Serial/MAVLink telemetry reporting.
- `tests/`: Automated test suite (`test_pnp_sensors`, `test_sensor_fusion`, `test_vehicle_controls`, `test_pubsub`, `test_dronecan`).
