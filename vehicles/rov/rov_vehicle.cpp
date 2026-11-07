#include "rov_vehicle.hpp"

namespace autopilot {
namespace vehicles {

ROVVehicle::ROVVehicle(hal::HAL* hal)
    : VehicleBase(hal, VehicleType::ROV_SUBMARINE) {
    _mixer = std::make_unique<ROVThrusterMixer>();
    setup_pids();
}

void ROVVehicle::setup_pids() {
    // Attitude stabilization (Roll & Pitch self-leveling)
    _roll_pid.set_gains(2.0f, 0.1f, 0.05f);
    _roll_pid.set_limits(-0.6f, 0.6f, 0.2f);

    _pitch_pid.set_gains(2.0f, 0.1f, 0.05f);
    _pitch_pid.set_limits(-0.6f, 0.6f, 0.2f);

    _yaw_pid.set_gains(1.5f, 0.05f, 0.05f);
    _yaw_pid.set_limits(-0.6f, 0.6f, 0.2f);

    // Depth hold
    _depth_pid.set_gains(1.2f, 0.0f, 0.0f);
    _depth_pid.set_limits(-1.0f, 1.0f, 0.0f); // Max 1 m/s climb/sink rate

    _depth_rate_pid.set_gains(0.4f, 0.15f, 0.02f);
    _depth_rate_pid.set_limits(-0.7f, 0.7f, 0.3f);
}

void ROVVehicle::update_controller(float dt) {
    const auto& state = get_state();

    if (_mode == ROVMode::MANUAL) {
        _demands = _user_input;
        return;
    }

    _demands = _user_input;

    // Stabilize attitude (keep ROV level in water)
    if (_mode == ROVMode::STABILIZE || _mode == ROVMode::DEPTH_HOLD) {
        // Roll & Pitch self leveling toward 0
        _demands.roll = _roll_pid.update(0.0f, state.euler_rad.x, dt);
        _demands.pitch = _pitch_pid.update(0.0f, state.euler_rad.y, dt);
    }

    // Depth Hold (ArduSub style)
    if (_mode == ROVMode::DEPTH_HOLD && state.altitude_valid) {
        // In water, depth = -altitude_m (or depth sensor)
        float current_depth = -state.altitude_m;
        float desired_sink_rate = _depth_pid.update(_target_depth_m, current_depth, dt);
        float current_sink_rate = -state.vertical_velocity_m_s;
        _demands.throttle = -_depth_rate_pid.update(desired_sink_rate, current_sink_rate, dt);
    }
}

} // namespace vehicles
} // namespace autopilot
