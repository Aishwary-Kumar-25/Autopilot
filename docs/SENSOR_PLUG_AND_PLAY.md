# Plug-and-Play Sensor Specification & Hot-Swap Architecture

## Motivation

Traditional commercial and open-source flight controllers (e.g. Pixhawk 2.4.8, Matek, Omnibus) solder the IMU, barometer, and flash directly to the main microcontroller PCB. This creates severe single points of failure:
- High-vibration fatigue or crash impact shears internal MEMS structures inside the IMU.
- Over-voltage spikes on sensor VDD rails fry the I2C/SPI tranceivers.
- Moisture or water ingress destroys exposed barometer diaphragms (especially critical in marine/ROV operations).

When an on-board sensor fails, the entire autopilot board is rendered useless. Desoldering LGA/QFN packages requires specialized hot-air stations and risks delaminating PCB traces.

**The Solution:** All primary sensing is decoupled from the main processor PCB. Sensors are connected as modular, plug-and-play modules over standard external buses. If a sensor fails, a field technician unplugs the bad sensor and plugs in a replacement—the autopilot detects, calibrates, and incorporates the new sensor **without requiring a power cycle or reboot**.

---

## Hardware Connectors & Pinout Standard

The system follows the **Pixhawk Connector Standard** (JST-GH 1.25mm locking connectors) and SparkFun Qwiic / Adafruit STEMMA QT 4-pin standards:

### 1. I2C Sensor Port (JST-GH 4-Pin)
| Pin # | Signal | Description |
|:-----:|:------:|:------------|
| 1 | **VCC** | +5.0V / +3.3V Clean LDO Power |
| 2 | **SCL** | I2C Clock Line (with 4.7kΩ pull-up on motherboard) |
| 3 | **SDA** | I2C Data Line (with 4.7kΩ pull-up on motherboard) |
| 4 | **GND** | System Ground |

### 2. External SPI Sensor Port (JST-GH 6-Pin)
| Pin # | Signal | Description |
|:-----:|:------:|:------------|
| 1 | **VCC** | +3.3V Power |
| 2 | **SCK** | SPI Serial Clock |
| 3 | **MISO** | SPI Master In / Slave Out |
| 4 | **MOSI** | SPI Master Out / Slave In |
| 5 | **CS** | Chip Select (Active Low) |
| 6 | **GND** | Ground |

### 3. GPS & External Magnetometer Port (JST-GH 6-Pin)
| Pin # | Signal | Description |
|:-----:|:------:|:------------|
| 1 | **VCC** | +5.0V Power |
| 2 | **TX** | Microcontroller UART TX -> GPS RX |
| 3 | **RX** | Microcontroller UART RX <- GPS TX |
| 4 | **SCL** | External Compass I2C Clock |
| 5 | **SDA** | External Compass I2C Data |
| 6 | **GND** | Ground |

---

## Dynamic Auto-Discovery Mechanism

During system startup, or when `auto_discover_sensors()` runs, `SensorManager` queries known I2C slave address spaces and reads standard identification registers:

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

### Health State Machine
Each sensor module maintains an autonomous health state:
1. `UNINITIALIZED`: Driver created but hardware not yet configured.
2. `HEALTHY`: Sensor passing periodic reads within timeout limits without checksum/communication errors.
3. `DEGRADED`: Occasional dropped frames or high bus noise (1 to 4 consecutive errors).
4. `FAILED`: 5 consecutive read timeouts, I2C bus NACKs, or frozen data streams.
5. `DISCONNECTED`: Physical device no longer acknowledges I2C address.

### Runtime Failover
If the active primary IMU or Barometer enters `FAILED` or `DISCONNECTED`:
- The active pointer is immediately invalidated to prevent polluted data from feeding the AHRS / flight controllers.
- If a secondary/redundant sensor was discovered, `SensorManager` instantly switches to the secondary unit.
- If no backup is available, the `ArmingHandler` triggers a safe descent or failsafe disarm.

### Field Replacement Flow (Hot-Swap)
1. **Sensor Dies**: Primary IMU burns out mid-test or on ground. `SensorManager` flags `FAILED`.
2. **Technician Action**: Technician disconnects the faulty sensor module and plugs in a new compatible sensor (e.g. replacing an MPU6050 with an ICM-20689 or new MPU6050).
3. **Periodic Recovery Probe**: The `probe_and_recover_sensors()` routine runs periodically (1 Hz):
   - Probes the I2C bus for unresponsive/failed addresses.
   - Detects the new physical device.
   - Reads WHO_AM_I, constructs the driver, initializes registers, and runs a zero-motion gyro bias calibration.
   - Reinstates the new sensor as active primary.
   - System returns to `HEALTHY` without requiring an STM32 power cycle.
