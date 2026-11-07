#include "ugv_vehicle.hpp"
#include <cmath>

namespace autopilot {
namespace vehicles {

UGVVehicle::UGVVehicle(hal::HAL* hal, UGVSteeringType steering_type)
    : VehicleBase(hal, VehicleType::UGV_GROUND_ROVER) {
    _mixer = std::make_unique<UGVMixer>(steering_type);
    setup_pids();
}

void UGVVehicle::setup_pids() {
    // Heading PID: outputs yaw steering demand [-1.0, 1.0]
    _heading_pid.set_gains(1.2f, 0.05f, 0.08f);
    _heading_pid.set_limits(-1.0f, 1.0f, 0.3f);

    // Speed PID: outputs forward throttle demand [-1.0, 1.0]
    _speed_pid.set_gains(0.5f, 0.1f, 0.02f);
    _speed_pid.set_limits(-1.0f, 1.0f, 0.4f);
}

void UGVVehicle::update_controller(float dt) {
    const auto& state = get_state();

    if (_mode == UGVMode::MANUAL) {
        _demands = _user_input;
        return;
    }

    _demands = _user_input;

    // Heading hold
    if (_mode == UGVMode::HEADING_HOLD || _mode == UGVMode::SPEED_HOLD) {
        float heading_err = _target_heading_rad - state.heading_rad;
        // Wrap heading error between -PI and +PI
        while (heading_err > static_cast<float>(M_PI)) heading_err -= 2.0f * static_cast<float>(M_PI);
        while (heading_err < -static_cast<float>(M_PI)) heading_err += 2.0f * static_cast<float>(M_PI);

        _demands.yaw = _heading_pid.update(0.0f, -heading_err, dt);
    }

    // Speed hold
    if (_mode == UGVMode::SPEED_HOLD && state.position_valid) {
        float current_speed = std::sqrt(state.velocity_ned.x * state.velocity_ned.x +
                                        state.velocity_ned.y * state.velocity_ned.y);
        _demands.forward = _speed_pid.update(_target_speed_m_s, current_speed, dt);
    }
}

} // namespace vehicles
} // namespace autopilot
