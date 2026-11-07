#include "ugv_vehicle.hpp"
#include <algorithm>
#include <cmath>

namespace autopilot {
namespace vehicles {

UGVVehicle::UGVVehicle(hal::HAL* hal, UGVSteeringType steering_type)
    : UGVVehicle(hal, steering_type, VehicleType::UGV_GROUND_ROVER) {}

UGVVehicle::UGVVehicle(hal::HAL* hal, UGVSteeringType steering_type, VehicleType type)
    : VehicleBase(hal, type) {
    _mixer = std::make_unique<UGVMixer>(steering_type);
    setup_pids();
}

void UGVVehicle::setup_pids() {
    // Heading PI: outputs yaw steering demand [-1.0, 1.0]. The damping (D) term
    // comes from the gyro yaw rate in update_controller(), not from differentiating
    // the wrapped heading error (which spikes at the +-180 deg wrap and on every
    // new target heading).
    _heading_pid.set_gains(1.2f, 0.05f, 0.0f);
    _heading_pid.set_limits(-1.0f, 1.0f, 0.3f);

    // Speed PID: outputs forward throttle demand [-1.0, 1.0]
    _speed_pid.set_gains(0.5f, 0.1f, 0.02f);
    _speed_pid.set_limits(-1.0f, 1.0f, 0.4f);
}

void UGVVehicle::set_mode(UGVMode mode) {
    if (mode == _mode) return;
    if (mode != UGVMode::MANUAL && _mode == UGVMode::MANUAL) {
        _target_heading_rad = get_state().heading_rad;
        _heading_pid.reset();
        _speed_pid.reset();
    }
    _mode = mode;
}

bool UGVVehicle::check_arming_inputs(std::string& failure_reason) const {
    if (std::fabs(_user_input.forward) > 0.1f || std::fabs(_user_input.throttle) > 0.1f) {
        failure_reason = "PreArm: drive stick not neutral";
        return false;
    }
    return true;
}

void UGVVehicle::on_arm() {
    _heading_pid.reset();
    _speed_pid.reset();
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

        const float steer = _heading_pid.update(heading_err, 0.0f, dt) - HEADING_KD * state.angular_velocity.z;
        _demands.yaw = std::clamp(steer, -1.0f, 1.0f);
    }

    // Speed hold
    if (_mode == UGVMode::SPEED_HOLD && state.position_valid) {
        // Signed speed along the heading, so reversing (negative target) works.
        const float current_speed = state.velocity_ned.x * std::cos(state.heading_rad) +
                                    state.velocity_ned.y * std::sin(state.heading_rad);
        _demands.forward = _speed_pid.update(_target_speed_m_s, current_speed, dt);
    }
}

} // namespace vehicles
} // namespace autopilot
