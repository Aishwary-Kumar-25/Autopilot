#pragma once

#include "control/vehicle_mixer.hpp"

namespace autopilot {
namespace vehicles {

/**
 * @brief Multirotor Quad-X Motor Mixer.
 * Motors:
 * 1: Front-Right (CCW)
 * 2: Rear-Left   (CCW)
 * 3: Front-Left  (CW)
 * 4: Rear-Right  (CW)
 */
class UAVQuadMixer : public control::VehicleMixer {
public:
    uint8_t get_num_channels() const override { return 4; }

    void output_to_actuators(const control::ControlDemands& demands, hal::PWMOutput* pwm, bool armed) override {
        if (!pwm) return;

        if (!armed) {
            for (uint8_t i = 0; i < 4; ++i) {
                pwm->set_channel(i, 1000); // Motor off
            }
            return;
        }

        float throttle = demands.throttle;
        float roll = demands.roll;
        float pitch = demands.pitch;
        float yaw = demands.yaw;

        // Standard Quad-X mixing matrix
        float m1 = throttle - roll + pitch + yaw; // Front-Right
        float m2 = throttle + roll - pitch + yaw; // Rear-Left
        float m3 = throttle + roll + pitch - yaw; // Front-Left
        float m4 = throttle - roll - pitch - yaw; // Rear-Right

        pwm->set_channel(0, map_unidirectional_pwm(m1));
        pwm->set_channel(1, map_unidirectional_pwm(m2));
        pwm->set_channel(2, map_unidirectional_pwm(m3));
        pwm->set_channel(3, map_unidirectional_pwm(m4));
    }
};

} // namespace vehicles
} // namespace autopilot
