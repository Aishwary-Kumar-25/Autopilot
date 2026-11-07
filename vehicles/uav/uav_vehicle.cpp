#include "uav_vehicle.hpp"

namespace autopilot {
namespace vehicles {

UAVVehicle::UAVVehicle(hal::HAL* hal)
    : VehicleBase(hal, VehicleType::UAV_MULTICOPTER) {
    _mixer = std::make_unique<UAVQuadMixer>();
    setup_pids();
}

void UAVVehicle::setup_pids() {
    // Angle Loop (P-only or PI)
    _roll_angle_pid.set_gains(4.5f, 0.0f, 0.0f);
    _roll_angle_pid.set_limits(-4.0f, 4.0f, 0.0f); // Max rate 4 rad/s (~230 deg/s)

    _pitch_angle_pid.set_gains(4.5f, 0.0f, 0.0f);
    _pitch_angle_pid.set_limits(-4.0f, 4.0f, 0.0f);

    // Rate Loop (PID)
    _roll_rate_pid.set_gains(0.15f, 0.10f, 0.004f);
    _roll_rate_pid.set_limits(-0.5f, 0.5f, 0.2f);

    _pitch_rate_pid.set_gains(0.15f, 0.10f, 0.004f);
    _pitch_rate_pid.set_limits(-0.5f, 0.5f, 0.2f);

    _yaw_rate_pid.set_gains(0.20f, 0.05f, 0.0f);
    _yaw_rate_pid.set_limits(-0.5f, 0.5f, 0.2f);

    // Altitude and Climb Rate Loop
    _altitude_pid.set_gains(1.0f, 0.0f, 0.0f);
    _altitude_pid.set_limits(-2.0f, 2.0f, 0.0f); // Max climb/descend speed: 2 m/s

    _velocity_z_pid.set_gains(0.3f, 0.1f, 0.02f);
    _velocity_z_pid.set_limits(-0.3f, 0.4f, 0.2f); // Throttle adjustment
}

void UAVVehicle::update_controller(float dt) {
    const auto& state = get_state();

    if (_mode == UAVFlightMode::MANUAL) {
        _demands = _user_input;
        return;
    }

    // 1. Attitude stabilization
    // Map user input [-1.0, 1.0] to target angles [-30 deg, +30 deg]
    constexpr float MAX_ANGLE_RAD = 30.0f * (static_cast<float>(M_PI) / 180.0f);
    float target_roll = _user_input.roll * MAX_ANGLE_RAD;
    float target_pitch = _user_input.pitch * MAX_ANGLE_RAD;
    float target_yaw_rate = _user_input.yaw * 3.0f; // up to 3 rad/s

    // Outer Loop: Angle -> Desired Rate
    float desired_roll_rate = _roll_angle_pid.update(target_roll, state.euler_rad.x, dt);
    float desired_pitch_rate = _pitch_angle_pid.update(target_pitch, state.euler_rad.y, dt);

    // Inner Loop: Rate -> Torque Demand
    _demands.roll = _roll_rate_pid.update(desired_roll_rate, state.angular_velocity.x, dt);
    _demands.pitch = _pitch_rate_pid.update(desired_pitch_rate, state.angular_velocity.y, dt);
    _demands.yaw = _yaw_rate_pid.update(target_yaw_rate, state.angular_velocity.z, dt);

    // 2. Throttle & Altitude
    if (_mode == UAVFlightMode::ALT_HOLD && state.altitude_valid) {
        float desired_vel_z = _altitude_pid.update(_target_altitude_m, state.altitude_m, dt);
        float throttle_adj = _velocity_z_pid.update(desired_vel_z, state.vertical_velocity_m_s, dt);
        float base_hover_throttle = 0.5f; // Estimated hover throttle
        _demands.throttle = std::clamp(base_hover_throttle + throttle_adj, 0.1f, 0.9f);
    } else {
        _demands.throttle = _user_input.throttle;
    }
}

} // namespace vehicles
} // namespace autopilot
