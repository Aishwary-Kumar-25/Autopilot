#include "ahrs_interface.hpp"

namespace autopilot {
namespace fusion {

SensorFusion::SensorFusion(drivers::SensorManager* sensor_manager)
    : _sensors(sensor_manager) {}

void SensorFusion::reset() {
    _attitude_filter.reset();
    _alt_estimator.reset(0.0f);
    _state = VehicleState{};
}

math::Vector3f SensorFusion::body_to_ned(const math::Vector3f& v_body) const {
    return _state.attitude_q.rotate_vector(v_body);
}

math::Vector3f SensorFusion::ned_to_body(const math::Vector3f& v_ned) const {
    return _state.attitude_q.conjugate().rotate_vector(v_ned);
}

void SensorFusion::update_attitude(float dt) {
    if (!_sensors) return;

    auto* imu = _sensors->get_active_imu();
    if (!imu || !imu->is_healthy()) {
        _state.attitude_valid = false;
        return;
    }

    const auto& gyro = imu->get_gyro();
    const auto& accel = imu->get_accel();

    _state.angular_velocity = gyro;
    _state.linear_accel_body = accel;

    auto* compass = _sensors->get_active_compass();
    if (compass && compass->is_healthy()) {
        const auto& mag = compass->get_field_gauss();
        _attitude_filter.update_9dof(gyro, accel, mag, dt);
    } else {
        _attitude_filter.update_imu(gyro, accel, dt);
    }

    _state.attitude_q = _attitude_filter.get_quaternion();
    _attitude_filter.get_euler_angles(_state.euler_rad.x, _state.euler_rad.y, _state.euler_rad.z);
    _state.heading_rad = _state.euler_rad.z;
    if (_state.heading_rad < 0.0f) {
        _state.heading_rad += 2.0f * static_cast<float>(M_PI);
    }

    // Transform body acceleration to Earth NED frame (subtract 1g gravity)
    math::Vector3f earth_accel = body_to_ned(accel);
    earth_accel.z -= 9.80665f; // Cancel static gravity vector
    _state.linear_accel_ned = earth_accel;

    _state.attitude_valid = true;
}

void SensorFusion::update_altitude(float dt) {
    if (!_sensors) return;

    // Upward acceleration = -linear_accel_ned.z
    float accel_z_up = -_state.linear_accel_ned.z;
    _alt_estimator.predict(accel_z_up, dt);

    auto* baro = _sensors->get_active_baro();
    if (baro && baro->is_healthy()) {
        float measured_alt = baro->get_altitude_m();
        _alt_estimator.correct(measured_alt, dt);
        _state.altitude_valid = true;
    }

    _state.altitude_m = _alt_estimator.get_altitude();
    _state.vertical_velocity_m_s = _alt_estimator.get_vertical_velocity();
    _state.position_ned.z = -_state.altitude_m;
    _state.velocity_ned.z = -_state.vertical_velocity_m_s;
}

void SensorFusion::update_position(float dt) {
    if (!_sensors) return;

    auto* gps = _sensors->get_active_gps();
    if (gps && gps->has_3d_fix()) {
        const auto& data = gps->get_data();
        float course_rad = data.ground_course_deg * (static_cast<float>(M_PI) / 180.0f);
        _state.velocity_ned.x = data.speed_m_s * std::cos(course_rad);
        _state.velocity_ned.y = data.speed_m_s * std::sin(course_rad);

        _state.position_ned.x += _state.velocity_ned.x * dt;
        _state.position_ned.y += _state.velocity_ned.y * dt;
        _state.position_valid = true;
    }
}

void SensorFusion::update(float dt) {
    update_attitude(dt);
    update_altitude(dt);
    update_position(dt);
}

} // namespace fusion
} // namespace autopilot
