# Plug-and-Play Sensor Specification & Hot-Swap Architecture

## Motivation: Why CAN Bus (DroneCAN) Replaces I2C

Traditional autopilots (including older Pixhawks and many ArduPilot builds) rely on I2C for external sensors (compass, barometer, airspeed). In actual flight vehicles, **I2C is a critical failure point**:
1. **Single-Ended Vulnerability**: I2C uses 3.3V single-ended lines (`SDA`/`SCL`) with high-impedance pull-ups. Running cables next to 50A–150A brushless motor wires induces high electromagnetic interference (EMI), causing corrupted frames.
2. **I2C Bus Lockups**: If any slave sensor hangs or drops power holding `SDA` low, the entire bus halts, freezing all other sensors sharing the bus.
3. **Cable Length Limits**: Signal integrity degrades rapidly beyond 15–20 cm.

### The DroneCAN Solution
The **E-Club Autopilot** establishes **CAN Bus (DroneCAN protocol)** as the primary external sensor bus:
- **Differential Signaling** (`CAN_H` / `CAN_L`): Rejects severe motor noise and ground bounce.
- **Hardware Error Frames & Auto-Retransmission**: Guaranteed frame delivery without CPU lockups.
- **Dynamic Node Discovery**: Nodes broadcast their presence and health autonomously.
- **I2C/SPI Retained as Secondary**: I2C and SPI remain fully supported for bench development and low-cost prototypes.

---

## Hardware Connectors & Pinout Standard

The system follows the **Pixhawk Connector Standard** (JST-GH 1.25mm locking connectors):

### 1. Primary CAN Sensor Port (JST-GH 4-Pin) — Recommended
| Pin # | Signal | Description |
|:-----:|:------:|:------------|
| 1 | **VCC** | +5.0V Clean LDO Power |
| 2 | **CAN_H** | CAN High Differential Signal (120Ω terminated bus) |
| 3 | **CAN_L** | CAN Low Differential Signal (120Ω terminated bus) |
| 4 | **GND** | System Ground |

### 2. Secondary I2C Sensor Port (JST-GH 4-Pin) — Bench / Prototyping
| Pin # | Signal | Description |
|:-----:|:------:|:------------|
| 1 | **VCC** | +5.0V / +3.3V Power |
| 2 | **SCL** | I2C Clock Line (4.7kΩ pull-up) |
| 3 | **SDA** | I2C Data Line (4.7kΩ pull-up) |
| 4 | **GND** | System Ground |

### 3. GPS & External Magnetometer Port (JST-GH 6-Pin)
| Pin # | Signal | Description |
|:-----:|:------:|:------------|
| 1 | **VCC** | +5.0V Power |
| 2 | **TX** | Microcontroller UART TX -> GPS RX |
| 3 | **RX** | Microcontroller UART RX <- GPS TX |
| 4 | **SCL** / **CAN_H** | I2C Clock or CAN High |
| 5 | **SDA** / **CAN_L** | I2C Data or CAN Low |
| 6 | **GND** | Ground |

---

## DroneCAN Message Protocol & Supported Data Types

The system's `DroneCANNode` listens on CAN 2.0B (29-bit identifier) at 1 Mbps, decoding the following standard DSDL message IDs:

| Data Type ID | DroneCAN Message Name | Target Driver | Description |
|:------------:|:----------------------|:--------------|:------------|
| `341` (`0x0155`) | `uavcan.protocol.NodeStatus` | `DroneCANNode` | Node presence, uptime, and health monitor (1 Hz) |
| `1000` (`0x03E8`) | `uavcan.equipment.ahrs.RawIMU` | `DroneCAN_IMU` | 3-axis gyro rates (float16) + 3-axis accel (float16) |
| `1001` (`0x03E9`) | `uavcan.equipment.ahrs.MagneticFieldStrength` | `DroneCAN_Compass` | 3-axis magnetic field in Gauss (float16) |
| `1020` (`0x03FC`) | `uavcan.equipment.air_data.StaticPressure` | `DroneCAN_Baro` | Ambient pressure in Pascal (float32) |
| `1021` (`0x03FD`) | `uavcan.equipment.air_data.StaticTemperature` | `DroneCAN_Baro` | Temperature in Kelvin (float16) |
| `1060` (`0x0424`) | `uavcan.equipment.gnss.Fix` | `DroneCAN_GPS` | Position (lat, lon, alt), speed, heading, and 3D fix |

---

## Dynamic Auto-Discovery Mechanism (I2C/SPI Fallback)

During system startup, `SensorManager` queries known I2C slave address spaces for legacy or bench sensors:

| Address | Probe Register | Expected Value | Detected Device | Driver Instantiated |
|:-------:|:--------------:|:--------------:|:---------------:|:-------------------:|
| `0x68` / `0x69` | `0x75` (WHO_AM_I) | `0x68` / `0x70` | InvenSense MPU6050 / MPU6500 | `MPU6050` |
| `0x68` / `0x69` | `0x75` (WHO_AM_I) | `0x98` | InvenSense ICM-20689 | `ICM20689` |
| `0x76` / `0x77` | `0xD0` (CHIP_ID) | `0x58` | Bosch BMP280 | `BMP280` |
| `0x76` / `0x77` | `0xD0` (CHIP_ID) | `0x60` | Bosch BME280 | `BMP280` |
| `0x0D` | `0x0D` (CHIP_ID) | `0xFF` | QST QMC5883L Compass | `QMC5883L` |
| UART Port | NMEA Stream | `$GPGGA` / `$GPRMC` | NMEA GNSS Module | `NMEAGPS` |

---

## Health Monitoring & Dynamic Replacement (Hot-Swap)

1. **Continuous Supervision**: Sensors report `HEALTHY`, `DEGRADED`, `FAILED`, or `DISCONNECTED`.
2. **Automatic Failover**: If the primary CAN or I2C IMU fails, `SensorManager` instantly switches to an available backup sensor or triggers safe descent.
3. **Live Hot-Swap**: When a replacement CAN node or I2C sensor is plugged in during runtime, `probe_and_recover_sensors()` detects the new hardware, initializes and recalibrates it, and seamlessly restores system health **without rebooting the autopilot**.
