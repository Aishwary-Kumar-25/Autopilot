#include "arming_handler.hpp"

namespace autopilot {
namespace control {

ArmingHandler::ArmingHandler(drivers::SensorManager* sensors) : _sensors(sensors) {}

bool ArmingHandler::check_prearm(std::string& failure_reason) const {
    if (!_sensors) {
        failure_reason = "SensorManager not initialized";
        return false;
    }

    if (!_sensors->has_healthy_imu()) {
        failure_reason = "PreArm: No healthy IMU detected";
        return false;
    }

    auto* imu = _sensors->get_active_imu();
    if (imu && imu->get_accel().length() < 5.0f) {
        failure_reason = "PreArm: Accelerometer reading invalid (< 0.5g)";
        return false;
    }

    return true;
}

bool ArmingHandler::arm(std::string& failure_reason) {
    if (_state == ArmingState::ARMED) {
        return true;
    }

    if (!check_prearm(failure_reason)) {
        _last_error = failure_reason;
        return false;
    }

    _state = ArmingState::ARMED;
    _last_error.clear();
    return true;
}

void ArmingHandler::disarm() {
    _state = ArmingState::DISARMED;
}

void ArmingHandler::force_failsafe(const std::string& reason) {
    _state = ArmingState::FAILS_SAFE;
    _last_error = reason;
}

} // namespace control
} // namespace autopilot
