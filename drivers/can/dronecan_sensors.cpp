#include "dronecan_sensors.hpp"
#include <cmath>

namespace autopilot {
namespace drivers {
namespace dronecan {

// ---------------- DroneCAN_IMU ----------------

DroneCAN_IMU::DroneCAN_IMU(uint8_t node_id) : _node_id(node_id) {
    _health = SensorHealth::UNINITIALIZED;
}

bool DroneCAN_IMU::init() {
    _health = SensorHealth::HEALTHY;
    return true;
}

bool DroneCAN_IMU::update() {
    // Health check based on packet timeout (e.g. 500ms timeout)
    return is_healthy();
}

void DroneCAN_IMU::ingest_raw_imu(const math::Vector3f& rate_gyro, const math::Vector3f& accel, uint32_t timestamp_ms) {
    _gyro = rate_gyro - _gyro_bias;
    _accel = accel;
    _last_packet_ms = timestamp_ms;
    record_success(timestamp_ms);
}

void DroneCAN_IMU::calibrate_gyro_bias(uint16_t) {
    // Typically calibrated on node or zeroed on flat ground
    _gyro_bias.zero();
}

// ---------------- DroneCAN_Baro ----------------

DroneCAN_Baro::DroneCAN_Baro(uint8_t node_id) : _node_id(node_id) {
    _health = SensorHealth::UNINITIALIZED;
}

bool DroneCAN_Baro::init() {
    _health = SensorHealth::HEALTHY;
    return true;
}

bool DroneCAN_Baro::update() {
    return is_healthy();
}

void DroneCAN_Baro::set_ground_reference(float ground_pressure_pa) {
    if (ground_pressure_pa > 50000.0f && ground_pressure_pa < 115000.0f) {
        _ground_pressure_pa = ground_pressure_pa;
    }
}

void DroneCAN_Baro::ingest_pressure(float pressure_pa, uint32_t timestamp_ms) {
    _pressure_pa = pressure_pa;
    _last_packet_ms = timestamp_ms;

    // Standard barometric formula: h = 44330 * (1 - (P / P0)^0.190295)
    float raw_alt = 44330.0f * (1.0f - std::pow(_pressure_pa / _ground_pressure_pa, 0.190295f));
    _altitude_m = _alt_filter.apply(raw_alt);
    record_success(timestamp_ms);
}

void DroneCAN_Baro::ingest_temperature(float temp_kelvin) {
    _temperature_c = temp_kelvin - 273.15f;
}

// ---------------- DroneCAN_Compass ----------------

DroneCAN_Compass::DroneCAN_Compass(uint8_t node_id) : _node_id(node_id) {
    _health = SensorHealth::UNINITIALIZED;
}

bool DroneCAN_Compass::init() {
    _health = SensorHealth::HEALTHY;
    return true;
}

bool DroneCAN_Compass::update() {
    return is_healthy();
}

void DroneCAN_Compass::ingest_mag_field(const math::Vector3f& field_gauss, uint32_t timestamp_ms) {
    _field_gauss = field_gauss;
    _last_packet_ms = timestamp_ms;

    _heading_rad = std::atan2(_field_gauss.y, _field_gauss.x);
    if (_heading_rad < 0.0f) {
        _heading_rad += 2.0f * static_cast<float>(M_PI);
    }
    record_success(timestamp_ms);
}

// ---------------- DroneCAN_GPS ----------------

DroneCAN_GPS::DroneCAN_GPS(uint8_t node_id) : _node_id(node_id) {
    _health = SensorHealth::UNINITIALIZED;
}

bool DroneCAN_GPS::init() {
    _health = SensorHealth::HEALTHY;
    return true;
}

bool DroneCAN_GPS::update() {
    return is_healthy();
}

void DroneCAN_GPS::ingest_gnss_fix(double lat_deg, double lon_deg, float alt_m, float speed_m_s,
                                   float heading_deg, uint8_t fix_type, uint8_t sats, uint32_t timestamp_ms) {
    _data.latitude_deg = lat_deg;
    _data.longitude_deg = lon_deg;
    _data.altitude_msl_m = alt_m;
    _data.speed_m_s = speed_m_s;
    _data.ground_course_deg = heading_deg;
    _data.fix_type = fix_type;
    _data.satellites = sats;
    _data.has_fix = (fix_type >= 3);
    _data.timestamp_ms = timestamp_ms;
    _last_packet_ms = timestamp_ms;

    record_success(timestamp_ms);
}

} // namespace dronecan
} // namespace drivers
} // namespace autopilot

