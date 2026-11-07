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

#include <algorithm>
#include <cmath>

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
    if (monitor_for(sensor.get())) return true; // already known: never add the same sensor twice

    _all_sensors.push_back(sensor);
    Monitor m;
    m.sensor = sensor.get();
    m.verdict = sensor->health();
    _monitors.push_back(m);

    // Become primary if there is none yet (health is re-evaluated every update()).
    switch (sensor->type()) {
        case SensorType::IMU:
            if (!_active_imu) _active_imu = static_cast<IMUSensor*>(sensor.get());
            break;
        case SensorType::BAROMETER:
            if (!_active_baro) _active_baro = static_cast<BarometerSensor*>(sensor.get());
            break;
        case SensorType::MAGNETOMETER:
            if (!_active_compass) _active_compass = static_cast<CompassSensor*>(sensor.get());
            break;
        case SensorType::GPS:
            if (!_active_gps) _active_gps = static_cast<GPSSensor*>(sensor.get());
            break;
        default:
            break;
    }
    return true;
}

SensorManager::Monitor* SensorManager::monitor_for(const Sensor* s) {
    for (auto& m : _monitors) if (m.sensor == s) return &m;
    return nullptr;
}

const SensorManager::Monitor* SensorManager::monitor_for(const Sensor* s) const {
    for (const auto& m : _monitors) if (m.sensor == s) return &m;
    return nullptr;
}

SensorHealth SensorManager::verdict(const Sensor* s) const {
    const Monitor* m = s ? monitor_for(s) : nullptr;
    return m ? m->verdict : SensorHealth::UNINITIALIZED;
}

static uint32_t timeout_for(SensorType t) {
    switch (t) {
        case SensorType::IMU:          return SensorManager::IMU_TIMEOUT_MS;
        case SensorType::BAROMETER:    return SensorManager::BARO_TIMEOUT_MS;
        case SensorType::MAGNETOMETER: return SensorManager::COMPASS_TIMEOUT_MS;
        case SensorType::GPS:          return SensorManager::GPS_TIMEOUT_MS;
        default:                       return 1000;
    }
}

static bool finite3(const math::Vector3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void SensorManager::evaluate(Monitor& m, uint32_t now_ms) {
    Sensor* s = m.sensor;
    const SensorHealth driver = s->health();
    SensorHealth v = driver;

    // New sample since last check?
    const uint32_t count = s->sample_count();
    const bool fresh = count != m.last_sample_count;
    if (fresh) {
        m.last_sample_count = count;
        m.last_sample_ms = now_ms;
        m.seen_sample = true;
    }

    if (v == SensorHealth::HEALTHY) {
        // timeout (only once the sensor has ever produced a timestamped sample)
        if (m.seen_sample && now_ms - m.last_sample_ms > timeout_for(s->type())) v = SensorHealth::FAILED;

        // range + stuck checks on the data itself
        if (s->type() == SensorType::IMU) {
            auto* imu = static_cast<IMUSensor*>(s);
            const auto& a = imu->get_accel();
            const auto& g = imu->get_gyro();
            if (!finite3(a) || !finite3(g) || a.length() > 16.0f * 9.80665f || g.length() > 35.0f) {
                v = SensorHealth::FAILED;
            } else if (fresh) {
                const bool same = a.x == m.last_a.x && a.y == m.last_a.y && a.z == m.last_a.z &&
                                  g.x == m.last_b.x && g.y == m.last_b.y && g.z == m.last_b.z;
                m.stuck_run = same ? m.stuck_run + 1 : 0;
                m.last_a = a;
                m.last_b = g;
            }
            if (m.stuck_run >= STUCK_SAMPLES) v = SensorHealth::FAILED;
            if (v == SensorHealth::HEALTHY && m.consistency_err > 1.0f) v = SensorHealth::DEGRADED;
        } else if (s->type() == SensorType::MAGNETOMETER) {
            const auto& f = static_cast<CompassSensor*>(s)->get_field_gauss();
            if (!finite3(f) || f.length() > 8.0f) v = SensorHealth::FAILED;
        } else if (s->type() == SensorType::BAROMETER) {
            auto* b = static_cast<BarometerSensor*>(s);
            if (!std::isfinite(b->get_altitude_m()) || b->get_pressure_pa() < 10000.0f || b->get_pressure_pa() > 120000.0f) {
                v = SensorHealth::FAILED;
            }
        }
    }

    if (v == SensorHealth::FAILED && _armed) m.latched_failed = true;
    if (m.latched_failed) v = SensorHealth::FAILED;

    if (v != m.verdict) {
        m.verdict = v;
        notify_event(s->name(), v);
    }
}

void SensorManager::update_consistency() {
    // Nearest-neighbour gyro disagreement: with 3 IMUs one bad unit is far from
    // both others, while each good one has a close neighbour.
    std::vector<Monitor*> imus;
    for (auto& m : _monitors) {
        if (m.sensor->type() == SensorType::IMU && m.verdict != SensorHealth::FAILED &&
            m.sensor->health() == SensorHealth::HEALTHY) {
            imus.push_back(&m);
        }
    }
    if (imus.size() < 2) {
        for (auto* m : imus) m->consistency_err = 0.0f;
        return;
    }
    for (auto* a : imus) {
        const auto& ga = static_cast<IMUSensor*>(a->sensor)->get_gyro();
        float best = 1e9f;
        for (auto* b : imus) {
            if (a == b) continue;
            const auto d = ga - static_cast<IMUSensor*>(b->sensor)->get_gyro();
            best = std::min(best, d.length());
        }
        const float err = best / GYRO_CONSISTENCY_RAD_S;
        a->consistency_err += 0.05f * (err - a->consistency_err); // low-pass so noise doesn't trip it
    }
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
                            _bound_i2c.emplace_back(addr, icm.get());
                            notify_event(icm->name(), SensorHealth::HEALTHY);
                        }
                    }
#endif
#ifdef ENABLE_DRIVER_MPU6050
                    if (who == 0x68 || who == 0x70 || who == 0x72) {
                        auto mpu = std::make_shared<MPU6050>(i2c, time, addr);
                        if (mpu->init()) {
                            register_sensor(mpu);
                            _bound_i2c.emplace_back(addr, mpu.get());
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
            register_sensor(s); // no-op for sensors we already know
        }
    }

    for (auto& s : _all_sensors) {
        if (s && s->health() != SensorHealth::DISCONNECTED) {
            s->update();
        }
    }

    const uint32_t now = (_hal && _hal->get_time()) ? _hal->get_time()->millis() : 0;
    update_consistency();
    for (auto& m : _monitors) evaluate(m, now);
    check_health_and_failover();
}

template <typename T>
void SensorManager::select_primary(T*& active, SensorType type) {
    const SensorHealth cur = active ? verdict(active) : SensorHealth::FAILED;
    if (cur == SensorHealth::HEALTHY) return;

    // Best alternative: OK sensors first (lowest consistency error), then degraded ones.
    T* best = nullptr;
    SensorHealth best_h = SensorHealth::FAILED;
    float best_err = 1e9f;
    for (auto& m : _monitors) {
        if (m.sensor->type() != type || m.sensor == active) continue;
        const bool better = (m.verdict == SensorHealth::HEALTHY && (best_h != SensorHealth::HEALTHY || m.consistency_err < best_err)) ||
                            (m.verdict == SensorHealth::DEGRADED && best_h == SensorHealth::FAILED);
        if (better) {
            best = static_cast<T*>(m.sensor);
            best_h = m.verdict;
            best_err = m.consistency_err;
        }
    }
    const bool switch_needed = (cur == SensorHealth::FAILED || cur == SensorHealth::UNINITIALIZED ||
                                cur == SensorHealth::DISCONNECTED) ? best != nullptr
                                                                   : (best && best_h == SensorHealth::HEALTHY);
    if (switch_needed) {
        if (active) notify_event(active->name(), cur);
        active = best;
        ++_failovers;
        notify_event(active->name(), best_h);
    }
}

void SensorManager::check_health_and_failover() {
    select_primary(_active_imu, SensorType::IMU);
    select_primary(_active_baro, SensorType::BAROMETER);
    select_primary(_active_compass, SensorType::MAGNETOMETER);
    select_primary(_active_gps, SensorType::GPS);
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

    // Hot-swap recovery is a ground-only operation: re-initialising a chip in flight
    // (gyro bias calibration needs the vehicle still) is more dangerous than flying on the backup.
    if (_armed) {
        if (_dronecan_node) _dronecan_node->broadcast_node_status();
        return;
    }

    // Check if any registered sensor is currently failed/disconnected and see if it can be re-initialized
    for (auto& s : _all_sensors) {
        if (verdict(s.get()) != SensorHealth::HEALTHY) {
            if (Monitor* m = monitor_for(s.get())) {
                m->latched_failed = false;
                m->stuck_run = 0;
                m->seen_sample = false;
                m->last_sample_count = s->sample_count();
                m->consistency_err = 0.0f;
            }
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
                if ((addr == 0x68 || addr == 0x69) && !has_healthy_imu() && i2c_addr_free_for_new_driver(addr)) {
                    uint8_t who = 0;
                    if (i2c->read_byte(addr, 0x75, who)) {
#ifdef ENABLE_DRIVER_ICM20689
                        if (who == 0x98) {
                            auto icm = std::make_shared<ICM20689>(i2c, time, addr);
                            if (icm->init()) {
                                register_sensor(icm);
                                _bound_i2c.emplace_back(addr, icm.get());
                                notify_event(icm->name(), SensorHealth::HEALTHY);
                            }
                        }
#endif
#ifdef ENABLE_DRIVER_MPU6050
                        if (who == 0x68 || who == 0x70 || who == 0x72) {
                            auto mpu = std::make_shared<MPU6050>(i2c, time, addr);
                            if (mpu->init()) {
                                register_sensor(mpu);
                                _bound_i2c.emplace_back(addr, mpu.get());
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

void SensorManager::remove_sensor(Sensor* s) {
    if (_active_imu == s) _active_imu = nullptr;
    if (_active_baro == s) _active_baro = nullptr;
    if (_active_compass == s) _active_compass = nullptr;
    if (_active_gps == s) _active_gps = nullptr;
    _monitors.erase(std::remove_if(_monitors.begin(), _monitors.end(), [s](const Monitor& m) { return m.sensor == s; }),
                    _monitors.end());
    _bound_i2c.erase(std::remove_if(_bound_i2c.begin(), _bound_i2c.end(), [s](const auto& b) { return b.second == s; }),
                     _bound_i2c.end());
    _all_sensors.erase(std::remove_if(_all_sensors.begin(), _all_sensors.end(), [s](const auto& p) { return p.get() == s; }),
                       _all_sensors.end());
}

// True if no working driver owns this address. A dead driver that failed to
// re-initialise (e.g. the chip was swapped for a different model) is dropped.
bool SensorManager::i2c_addr_free_for_new_driver(uint8_t addr) {
    for (auto& b : _bound_i2c) {
        if (b.first != addr) continue;
        if (verdict(b.second) == SensorHealth::HEALTHY) return false;
        remove_sensor(b.second);
        return true;
    }
    return true;
}

bool SensorManager::has_healthy_imu() const { return _active_imu && is_usable(_active_imu); }
bool SensorManager::has_healthy_baro() const { return _active_baro && is_usable(_active_baro); }
bool SensorManager::has_healthy_compass() const { return _active_compass && is_usable(_active_compass); }
bool SensorManager::has_healthy_gps() const { return _active_gps && is_usable(_active_gps); }

} // namespace drivers
} // namespace autopilot
