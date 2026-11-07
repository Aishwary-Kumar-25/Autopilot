#include "sensor_manager.hpp"
#include "drivers/can/dronecan_node.hpp"

#ifdef ENABLE_DRIVER_MPU6050
#include "drivers/imu/mpu6050.hpp"
#endif

#ifdef ENABLE_DRIVER_ICM20689
#include "drivers/imu/icm20689.hpp"
#endif

#ifdef ENABLE_DRIVER_BMP280
#include "drivers/baro/bmp280.hpp"
#endif

#ifdef ENABLE_DRIVER_QMC5883L
#include "drivers/mag/qmc5883l.hpp"
#endif

#ifdef ENABLE_DRIVER_NMEA_GPS
#include "drivers/gps/nmea.hpp"
#endif

#include <iostream>

namespace autopilot {
namespace drivers {

SensorManager::SensorManager(hal::HAL* hal) : _hal(hal) {}

SensorManager::~SensorManager() = default;

void SensorManager::notify_event(const char* name, SensorHealth health) {
    if (_event_callback) {
        _event_callback(name, health);
    }
}

bool SensorManager::register_sensor(std::shared_ptr<Sensor> sensor) {
    if (!sensor) return false;

    _all_sensors.push_back(sensor);

    // Auto-assign as primary if none active or current is unhealthy
    switch (sensor->type()) {
        case SensorType::IMU:
            if (!_active_imu || !_active_imu->is_healthy()) {
                _active_imu = static_cast<IMUSensor*>(sensor.get());
            }
            break;
        case SensorType::BAROMETER:
            if (!_active_baro || !_active_baro->is_healthy()) {
                _active_baro = static_cast<BarometerSensor*>(sensor.get());
            }
            break;
        case SensorType::MAGNETOMETER:
            if (!_active_compass || !_active_compass->is_healthy()) {
                _active_compass = static_cast<CompassSensor*>(sensor.get());
            }
            break;
        case SensorType::GPS:
            if (!_active_gps || !_active_gps->is_healthy()) {
                _active_gps = static_cast<GPSSensor*>(sensor.get());
            }
            break;
        default:
            break;
    }

    return true;
}

void SensorManager::auto_discover_sensors() {
    if (!_hal) return;
    auto* i2c = _hal->get_i2c();
    auto* time = _hal->get_time();

    if (i2c && time) {
        // 1. Probe for IMUs at 0x68 and 0x69
        const uint8_t imu_addrs[] = {0x68, 0x69};
        for (uint8_t addr : imu_addrs) {
            if (i2c->probe(addr)) {
                uint8_t who = 0;
                if (i2c->read_byte(addr, 0x75, who)) {
#ifdef ENABLE_DRIVER_ICM20689
                    if (who == 0x98) {
                        auto icm = std::make_shared<ICM20689>(i2c, time, addr);
                        if (icm->init()) {
                            register_sensor(icm);
                            notify_event(icm->name(), SensorHealth::HEALTHY);
                        }
                    }
#endif
#ifdef ENABLE_DRIVER_MPU6050
                    if (who == 0x68 || who == 0x70 || who == 0x72) {
                        auto mpu = std::make_shared<MPU6050>(i2c, time, addr);
                        if (mpu->init()) {
                            register_sensor(mpu);
                            notify_event(mpu->name(), SensorHealth::HEALTHY);
                        }
                    }
#endif
                }
            }
        }

        // 2. Probe for Barometer at 0x76 and 0x77
#ifdef ENABLE_DRIVER_BMP280
        const uint8_t baro_addrs[] = {0x76, 0x77};
        for (uint8_t addr : baro_addrs) {
            if (i2c->probe(addr)) {
                uint8_t chip_id = 0;
                if (i2c->read_byte(addr, 0xD0, chip_id)) {
                    if (chip_id == 0x58 || chip_id == 0x60) {
                        auto bmp = std::make_shared<BMP280>(i2c, time, addr);
                        if (bmp->init()) {
                            register_sensor(bmp);
                            notify_event(bmp->name(), SensorHealth::HEALTHY);
                        }
                    }
                }
            }
        }
#endif

        // 3. Probe for Magnetometer at 0x0D
#ifdef ENABLE_DRIVER_QMC5883L
        if (i2c->probe(0x0D)) {
            auto qmc = std::make_shared<QMC5883L>(i2c, time, 0x0D);
            if (qmc->init()) {
                register_sensor(qmc);
                notify_event(qmc->name(), SensorHealth::HEALTHY);
            }
        }
#endif
    }

    // 4. GPS on UART
#ifdef ENABLE_DRIVER_NMEA_GPS
    auto* uart = _hal->get_uart();
    if (uart && time) {
        auto gps = std::make_shared<NMEAGPS>(uart, time);
        if (gps->init()) {
            register_sensor(gps);
            notify_event(gps->name(), SensorHealth::HEALTHY);
        }
    }
#endif

    // 5. DroneCAN on CAN Bus
    auto* can = _hal->get_can();
    if (can && time) {
        _dronecan_node = std::make_unique<dronecan::DroneCANNode>(can, time, 1);
        _dronecan_node->init(1000000);
    }
}

void SensorManager::update() {
    if (_dronecan_node) {
        _dronecan_node->process_rx();
        for (const auto& s : _dronecan_node->get_discovered_sensors()) {
            register_sensor(s);
        }
    }

    for (auto& s : _all_sensors) {
        if (s && s->health() != SensorHealth::DISCONNECTED) {
            s->update();
        }
    }
    check_health_and_failover();
}

void SensorManager::check_health_and_failover() {
    // IMU Failover
    if (_active_imu && !_active_imu->is_healthy()) {
        notify_event(_active_imu->name(), _active_imu->health());
        _active_imu = nullptr;
        // Search for healthy backup IMU
        for (auto& s : _all_sensors) {
            if (s->type() == SensorType::IMU && s->is_healthy()) {
                _active_imu = static_cast<IMUSensor*>(s.get());
                notify_event(_active_imu->name(), SensorHealth::HEALTHY);
                break;
            }
        }
    }

    // Barometer Failover
    if (_active_baro && !_active_baro->is_healthy()) {
        notify_event(_active_baro->name(), _active_baro->health());
        _active_baro = nullptr;
        for (auto& s : _all_sensors) {
            if (s->type() == SensorType::BAROMETER && s->is_healthy()) {
                _active_baro = static_cast<BarometerSensor*>(s.get());
                notify_event(_active_baro->name(), SensorHealth::HEALTHY);
                break;
            }
        }
    }

    // Compass Failover
    if (_active_compass && !_active_compass->is_healthy()) {
        notify_event(_active_compass->name(), _active_compass->health());
        _active_compass = nullptr;
        for (auto& s : _all_sensors) {
            if (s->type() == SensorType::MAGNETOMETER && s->is_healthy()) {
                _active_compass = static_cast<CompassSensor*>(s.get());
                notify_event(_active_compass->name(), SensorHealth::HEALTHY);
                break;
            }
        }
    }
}

void SensorManager::probe_and_recover_sensors() {
    if (!_hal) return;
    auto* time = _hal->get_time();
    if (!time) return;

    uint32_t now = time->millis();
    if (now - _last_probe_time_ms < 1000) { // Probe every 1000 ms
        return;
    }
    _last_probe_time_ms = now;

    // Check if any registered sensor is currently failed/disconnected and see if it can be re-initialized
    for (auto& s : _all_sensors) {
        if (!s->is_healthy()) {
            if (s->init()) {
                notify_event(s->name(), SensorHealth::HEALTHY);
                // If we lacked an active sensor of this type, reinstate it immediately
                if (s->type() == SensorType::IMU && !_active_imu) {
                    _active_imu = static_cast<IMUSensor*>(s.get());
                } else if (s->type() == SensorType::BAROMETER && !_active_baro) {
                    _active_baro = static_cast<BarometerSensor*>(s.get());
                } else if (s->type() == SensorType::MAGNETOMETER && !_active_compass) {
                    _active_compass = static_cast<CompassSensor*>(s.get());
                }
            }
        }
    }

    // Also probe for newly plugged-in hardware not previously known
    auto* i2c = _hal->get_i2c();
    if (i2c) {
        const uint8_t probe_addrs[] = {0x68, 0x69, 0x76, 0x77, 0x0D};
        for (uint8_t addr : probe_addrs) {
            if (i2c->probe(addr)) {
                // If it's 0x68/0x69 and we have no active IMU, try creating
                if ((addr == 0x68 || addr == 0x69) && (!_active_imu || !_active_imu->is_healthy())) {
                    uint8_t who = 0;
                    if (i2c->read_byte(addr, 0x75, who)) {
#ifdef ENABLE_DRIVER_ICM20689
                        if (who == 0x98) {
                            auto icm = std::make_shared<ICM20689>(i2c, time, addr);
                            if (icm->init()) {
                                register_sensor(icm);
                                notify_event(icm->name(), SensorHealth::HEALTHY);
                            }
                        }
#endif
#ifdef ENABLE_DRIVER_MPU6050
                        if (who == 0x68 || who == 0x70 || who == 0x72) {
                            auto mpu = std::make_shared<MPU6050>(i2c, time, addr);
                            if (mpu->init()) {
                                register_sensor(mpu);
                                notify_event(mpu->name(), SensorHealth::HEALTHY);
                            }
                        }
#endif
                    }
                }
            }
        }
    }

    if (_dronecan_node) {
        _dronecan_node->broadcast_node_status();
    }
}

bool SensorManager::has_healthy_imu() const {
    return _active_imu != nullptr && _active_imu->is_healthy();
}

bool SensorManager::has_healthy_baro() const {
    return _active_baro != nullptr && _active_baro->is_healthy();
}

bool SensorManager::has_healthy_compass() const {
    return _active_compass != nullptr && _active_compass->is_healthy();
}

bool SensorManager::has_healthy_gps() const {
    return _active_gps != nullptr && _active_gps->is_healthy();
}

} // namespace drivers
} // namespace autopilot
