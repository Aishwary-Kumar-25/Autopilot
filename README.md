# Modular Plug-and-Play Autopilot Flight Stack

A high-reliability, modular autopilot designed from scratch for **UAVs (Multirotors)**, **UGVs (Ground Vehicles)**, and **ROVs (Underwater Submarines)** running on the **STM32** processor family.

---

## Key Features

1. **CAN Bus (DroneCAN) & Swappable Sensors**:
   - Replaces noisy, lockup-prone I2C with **DroneCAN (CAN 2.0B differential signaling)** as the primary external sensor bus, eliminating I2C bus lockup vulnerabilities.
   - Dynamic auto-discovery detects connected CAN sensor nodes (`RawIMU`, `StaticPressure`, `MagneticFieldStrength`, `Fix`) and I2C/SPI sensors on boot or during runtime.
   - Continuous health monitoring detects sensor failures and supports **live field hot-swapping** without rebooting the flight controller.

2. **Zero-Allocation Pub/Sub Middleware**:
   - High-throughput, lock-free `Topic<T>` message broker inspired by PX4 uORB.
   - Decouples sensor estimators from flight controllers, allowing a **single firmware binary** to switch dynamically between UAV, UGV, and ROV vehicle profiles at runtime without reflashing.

3. **Modular Driver Compilation**:
   - Granular CMake feature flags (`ENABLE_DRIVER_MPU6050`, `ENABLE_CAN_DRONECAN`, etc.) allow compiling only required drivers, dropping compile times to sub-second levels and minimizing flash usage.

4. **Universal Sensor Fusion Engine**:
   - The exact same state estimation pipeline runs across all vehicles.
   - 6-DOF / 9-DOF Quaternion **Madgwick Filter** provides fast, drift-compensated attitude (Roll, Pitch, Yaw).
   - 3rd-order **Baro-Inertial Complementary Filter** computes lag-free vertical velocity ($v_z$) and altitude ($z$).
   - Outputs a unified `VehicleState` struct (quaternion, Euler angles, body angular rates, NED accelerations, NED velocities, and positions).

5. **Multi-Vehicle Abstraction & Generalized Actuator Matrix Mixer**:
   - **ActuatorMatrixMixer (2 to 16 Channels)**: Dynamic control allocation matrix ($u = B \cdot \tau$) replacing hardcoded mixers. Features dynamic desaturation and scaling to preserve attitude authority without clipping.
   - **Interface Option for Thruster Selection**: Select thruster/motor count dynamically at runtime:
     - **ROV Submarine / AUV**: Supports 3 thrusters (simple budget 2-horiz/1-vert), 4 thrusters (basic 2-horiz/2-vert), 6 thrusters (standard vectored BlueROV2 6-DOF), 8 thrusters (decoupled 6-DOF BlueROV2 Heavy & RoboSub AUV with independent pitch and roll moments), and custom configurations up to 16 thrusters.
     - **UAV Multicopter**: Scales dynamically from Tricopters (3), Quad-X (4), Hexa-X (6), Octa-X (8), up to 16-rotor heavy-lift platforms.
     - **DroneCAN ESC Integration**: Generates native `uavcan.equipment.esc.RawCommand` (DSDL 1030) packets for direct CAN bus motor control alongside standard 16-channel PWM.
   - **UGV (Ground Rover)**: Heading hold and speed hold controllers with selectable Ackermann steering or Skid-Steer differential mixing.

6. **STM32 Hardware Abstraction Layer (HAL)**:
   - Targets the standard class of high-performance 32-bit ARM Cortex-M processors (STM32F405, STM32F745/765, STM32H743/753).
   - Hardware PWM output driver with configurable timer mapping (`TIM1`, `TIM4`, `TIM3`, `TIM8`) delivering standard 50 Hz to 400 Hz microsecond pulses (1000–2000 µs).
   - Dual-target design: runs natively on STM32 hardware and includes a POSIX emulation HAL for 100% test coverage and software-in-the-loop (SIL) testing on Linux host machines.

---

## Directory Structure

```
Autopilot_eclub/
├── CMakeLists.txt              # Unified build system (Host simulation + STM32)
├── README.md                   # Project overview & quickstart
├── docs/                       # Technical specifications
│   ├── ARCHITECTURE.md         # System architecture & dataflow diagrams
│   ├── SENSOR_PLUG_AND_PLAY.md # PnP sensor pinouts, DroneCAN, & hot-swap
│   ├── HARDWARE_STM32_HAL.md   # STM32 class pinouts, timers, & cross-compile
│   └── ACTUATOR_MATRIX_MIXING.md # 2-16 channel matrix mixer & geometry presets
├── hal/                        # Hardware Abstraction Layer
│   ├── hal.hpp                 # Bus, Timer, PWM, & Peripheral interfaces
│   ├── posix/                  # Linux/POSIX host simulation & mock hardware
│   └── stm32/                  # STM32 hardware timers, PWM channels, bxCAN, I2C
├── drivers/                    # Modular Plug-and-Play Sensor Drivers
│   ├── can/                    # DroneCAN node dispatcher, parser, and CAN drivers
│   ├── sensor_interface.hpp    # IMU, Barometer, Magnetometer, GPS interfaces
│   ├── sensor_manager.hpp/cpp  # Dynamic discovery, failover, & health supervisor
│   ├── imu/                    # MPU6050 and ICM-20689 drivers
│   ├── baro/                   # BMP280 / BME280 barometer driver
│   ├── mag/                    # QMC5883L 3-axis digital compass driver
│   └── gps/                    # NMEA GPS serial stream driver
├── fusion/                     # Universal Vehicle-Agnostic Sensor Fusion
│   ├── ahrs_interface.hpp/cpp  # Unified AHRS state estimator
│   ├── madgwick.hpp/cpp        # 6-DOF / 9-DOF Quaternion Madgwick filter
│   ├── complementary.hpp/cpp   # 3rd-order baro-inertial altitude/climb filter
│   └── state_types.hpp         # VehicleState definition
├── math/                       # Embedded 3D math & controllers
│   ├── vector3.hpp             # Vector3f
│   ├── quaternion.hpp          # Quaternion math & Euler conversions
│   ├── matrix3x3.hpp           # Matrix3f
│   ├── low_pass_filter.hpp     # 1st-order Butterworth low-pass filter
│   └── pid.hpp                 # Anti-windup, filtered derivative PID
├── control/                    # Control & Actuator Interfaces
│   ├── arming_handler.hpp/cpp  # Pre-arm validators & safety interlock
│   └── vehicle_mixer.hpp       # Actuator PWM mapper
├── vehicles/                   # Vehicle-Specific Dynamic Models
│   ├── vehicle_base.hpp/cpp    # Base vehicle class
│   ├── uav/                    # Quad-X Multicopter & altitude hold
│   ├── ugv/                    # Ground Rover (Ackermann / Skid-Steer)
│   └── rov/                    # ArduSub-inspired 6-DOF Submarine
├── system/                     # Real-Time Scheduling & Telemetry
│   ├── pubsub.hpp              # Zero-allocation Pub/Sub messaging middleware
│   ├── scheduler.hpp/cpp       # AP_Scheduler-style multi-rate cyclic executor
│   ├── telemetry.hpp/cpp       # Serial/MAVLink telemetry reporting
│   └── main.cpp                # Main system entrypoint & flight loop
└── tests/                      # Automated Test Suite
    ├── test_pnp_sensors.cpp    # PnP discovery, fault injection, & hot-swap
    ├── test_sensor_fusion.cpp  # Level & 30-deg tilt convergence & climb
    ├── test_vehicle_controls.cpp # UAV, UGV, & ROV PWM actuator validation
    ├── test_pubsub.cpp         # Zero-allocation Pub/Sub verification
    └── test_dronecan.cpp       # DroneCAN protocol, sensors, & node dispatcher
```

---

## Quickstart: Building and Running

### 1. Build the Project
```bash
cd Autopilot_eclub
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 2. Run the Automated Test Suite
```bash
ctest --output-on-failure
```
All five automated test suites will execute:
- `test_pnp_sensors`: Validates auto-discovery of I2C/SPI sensors and dynamic recovery when a sensor is hot-plugged.
- `test_sensor_fusion`: Validates that the universal Madgwick & baro-inertial filters converge to level orientation, track roll tilt up to 30°, and estimate altitude accurately.
- `test_vehicle_controls`: Validates that UAV (Quad-X), UGV (Rover), and ROV (ArduSub 6-DOF) compute correct PWM outputs across all actuator channels while using the same state estimator.
- `test_pubsub`: Validates the lock-free, zero-allocation Publish/Subscribe message broker and dynamic vehicle profile routing.
- `test_dronecan`: Validates the 29-bit DroneCAN protocol, float16 conversions, dynamic node status discovery, and high-reliability CAN sensor reception.

### 3. Run the Flight Loop Simulation
```bash
# Run UAV Multicopter
./autopilot_app uav

# Run UGV Ground Rover
./autopilot_app ugv

# Run ROV Underwater Submarine
./autopilot_app rov
```
