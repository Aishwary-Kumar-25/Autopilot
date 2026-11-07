# Modular Plug-and-Play Autopilot Flight Stack

A high-reliability, modular autopilot designed from scratch for **UAVs (Multirotors)**, **UGVs (Ground Vehicles)**, and **ROVs (Underwater Submarines)** running on the **STM32** processor family.

---

## Key Features

1. **Plug-and-Play Swappable Sensors**:
   - Eliminates the vulnerability of traditional flight controllers where soldered on-board sensors fail or burn out, requiring replacement of the entire board.
   - External sensors plug in over standard 4-pin/6-pin JST-GH I2C, SPI, and UART ports.
   - Dynamic auto-discovery detects plugged-in sensors (MPU6050, ICM-20689, BMP280, QMC5883L, NMEA-GPS) on boot or during runtime.
   - Continuous health monitoring detects sensor failures and supports **live field hot-swapping** without rebooting the flight controller.

2. **Universal Sensor Fusion Engine**:
   - The exact same state estimation pipeline runs across all vehicles.
   - 6-DOF / 9-DOF Quaternion **Madgwick Filter** provides fast, drift-compensated attitude (Roll, Pitch, Yaw).
   - 3rd-order **Baro-Inertial Complementary Filter** computes lag-free vertical velocity ($v_z$) and altitude ($z$).
   - Outputs a unified `VehicleState` struct (quaternion, Euler angles, body angular rates, NED accelerations, NED velocities, and positions).

3. **Multi-Vehicle Abstraction & Specialized Controls**:
   - **UAV (Multicopter - Quad-X)**: Cascaded Angle P -> Rate PID loops, altitude hold, and Quad-X motor mixer (Channels 0–3).
   - **UGV (Ground Rover)**: Heading hold and speed hold controllers with selectable Ackermann steering (Steering Servo + ESC) or Skid-Steer differential mixing (Channels 0–1).
   - **ROV (Underwater Submarine - ArduSub-Inspired)**: 6-DOF vectored thruster matrix mixer (Channels 0–5) and depth-hold controller using common sensor fusion depth estimation.

4. **STM32 Hardware Abstraction Layer (HAL)**:
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
│   ├── SENSOR_PLUG_AND_PLAY.md # PnP sensor pinouts, protocols, & hot-swap
│   └── HARDWARE_STM32_HAL.md   # STM32 class pinouts, timers, & cross-compile
├── hal/                        # Hardware Abstraction Layer
│   ├── hal.hpp                 # Bus, Timer, PWM, & Peripheral interfaces
│   ├── posix/                  # Linux/POSIX host simulation & mock hardware
│   └── stm32/                  # STM32 hardware timers, PWM channels, I2C, SPI
├── drivers/                    # Modular Plug-and-Play Sensor Drivers
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
│   ├── scheduler.hpp/cpp       # AP_Scheduler-style multi-rate cyclic executor
│   ├── telemetry.hpp/cpp       # Serial/MAVLink telemetry reporting
│   └── main.cpp                # Main system entrypoint & flight loop
└── tests/                      # Automated Test Suite
    ├── test_pnp_sensors.cpp    # PnP discovery, fault injection, & hot-swap
    ├── test_sensor_fusion.cpp  # Level & 30-deg tilt convergence & climb
    └── test_vehicle_controls.cpp # UAV, UGV, & ROV PWM actuator validation
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
All three test suites will execute:
- `test_pnp_sensors`: Validates auto-discovery of I2C/SPI sensors, simulates an in-flight hardware burnout, and verifies dynamic recovery when a replacement sensor is hot-plugged.
- `test_sensor_fusion`: Validates that the universal Madgwick & baro-inertial filters converge to level orientation, track roll tilt up to 30°, and estimate altitude accurately.
- `test_vehicle_controls`: Validates that UAV (Quad-X), UGV (Rover), and ROV (ArduSub 6-DOF) compute correct PWM outputs across all actuator channels while using the same state estimator.

### 3. Run the Flight Loop Simulation
```bash
# Run UAV Multicopter
./autopilot_app uav

# Run UGV Ground Rover
./autopilot_app ugv

# Run ROV Underwater Submarine
./autopilot_app rov
```
