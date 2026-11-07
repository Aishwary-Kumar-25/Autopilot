#pragma once

#include "control/vehicle_mixer.hpp"

namespace autopilot {
namespace vehicles {

/**
 * @brief 6-DOF Underwater Submarine Thruster Matrix Mixer.
 * Implements standard Vectored 6-Thruster ROV configuration (as used in ArduSub BlueROV2):
 * Thruster 1: Forward-Right (Vectored 45 deg)
 * Thruster 2: Forward-Left  (Vectored 45 deg)
 * Thruster 3: Rear-Right     (Vectored 45 deg)
 * Thruster 4: Rear-Left      (Vectored 45 deg)
 * Thruster 5: Vertical-Right
 * Thruster 6: Vertical-Left
 */
class ROVThrusterMixer : public control::VehicleMixer {
public:
    uint8_t get_num_channels() const override { return 6; }

    void output_to_actuators(const control::ControlDemands& demands, hal::PWMOutput* pwm, bool armed) override {
        if (!pwm) return;

        if (!armed) {
            for (uint8_t i = 0; i < 6; ++i) {
                pwm->set_channel(i, 1500); // 1500 us = Stopped / neutral
            }
            return;
        }

        float fwd = demands.forward;
        float lat = demands.lateral;
        float yaw = demands.yaw;
        float heave = demands.throttle; // Vertical thrust
        float roll = demands.roll;
        float pitch = demands.pitch;

        // ArduSub Vectored 6-DOF Thruster Matrix
        float t1 =  fwd - lat + yaw; // Forward-Right
        float t2 =  fwd + lat - yaw; // Forward-Left
        float t3 = -fwd - lat + yaw; // Rear-Right
        float t4 = -fwd + lat - yaw; // Rear-Left
        float t5 =  heave - roll + pitch; // Vertical-Right
        float t6 =  heave + roll + pitch; // Vertical-Left

        pwm->set_channel(0, map_bidirectional_pwm(t1));
        pwm->set_channel(1, map_bidirectional_pwm(t2));
        pwm->set_channel(2, map_bidirectional_pwm(t3));
        pwm->set_channel(3, map_bidirectional_pwm(t4));
        pwm->set_channel(4, map_bidirectional_pwm(t5));
        pwm->set_channel(5, map_bidirectional_pwm(t6));
    }
};

} // namespace vehicles
} // namespace autopilot
