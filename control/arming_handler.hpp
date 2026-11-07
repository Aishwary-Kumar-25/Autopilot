#pragma once

#include "drivers/sensor_manager.hpp"
#include <string>

namespace autopilot {
namespace control {

enum class ArmingState {
    DISARMED = 0,
    ARMING,
    ARMED,
    FAILS_SAFE
};

class ArmingHandler {
public:
    ArmingHandler(drivers::SensorManager* sensors);

    bool arm(std::string& failure_reason);
    void disarm();
    void force_failsafe(const std::string& reason);

    bool is_armed() const { return _state == ArmingState::ARMED; }
    ArmingState get_state() const { return _state; }
    const std::string& get_last_error() const { return _last_error; }

    /**
     * @brief Run pre-arm safety checks.
     */
    bool check_prearm(std::string& failure_reason) const;

private:
    drivers::SensorManager* _sensors{nullptr};
    ArmingState _state{ArmingState::DISARMED};
    std::string _last_error;
};

} // namespace control
} // namespace autopilot
