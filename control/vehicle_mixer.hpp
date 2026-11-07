#pragma once

#include "hal/hal.hpp"
#include <algorithm>
#include <cstdint>

namespace autopilot {
namespace control {

struct ControlDemands {
    float roll{0.0f};      // [-1.0, +1.0] Roll torque
    float pitch{0.0f};     // [-1.0, +1.0] Pitch torque
    float yaw{0.0f};       // [-1.0, +1.0] Yaw torque
    float throttle{0.0f};  // [0.0, 1.0] for UAV, [-1.0, 1.0] for UGV/ROV
    float forward{0.0f};   // [-1.0, +1.0] Surge
    float lateral{0.0f};   // [-1.0, +1.0] Sway
};

class VehicleMixer {
public:
    virtual ~VehicleMixer() = default;

    /**
     * @brief Mix control demands into physical PWM outputs on hardware pins.
     * @param demands Desired normalized torques/forces.
     * @param pwm Hardware PWM driver interface.
     * @param armed Whether the vehicle is armed.
     */
    virtual void output_to_actuators(const ControlDemands& demands, hal::PWMOutput* pwm, bool armed) = 0;

    /**
     * @brief Number of active PWM output channels used by this mixer.
     */
    virtual uint8_t get_num_channels() const = 0;

protected:
    static uint16_t map_unidirectional_pwm(float demand) {
        float clamped = std::clamp(demand, 0.0f, 1.0f);
        return static_cast<uint16_t>(1000.0f + (clamped * 1000.0f));
    }

    static uint16_t map_bidirectional_pwm(float demand) {
        float clamped = std::clamp(demand, -1.0f, 1.0f);
        return static_cast<uint16_t>(1500.0f + (clamped * 500.0f));
    }
};

} // namespace control
} // namespace autopilot
