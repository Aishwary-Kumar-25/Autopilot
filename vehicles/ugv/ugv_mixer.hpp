#pragma once

#include "control/vehicle_mixer.hpp"

namespace autopilot {
namespace vehicles {

enum class UGVSteeringType {
    ACKERMANN,  // Channel 0: Steering Servo, Channel 1: Throttle Motor
    SKID_STEER  // Channel 0: Left Track/Wheel, Channel 1: Right Track/Wheel
};

class UGVMixer : public control::VehicleMixer {
public:
    UGVMixer(UGVSteeringType type = UGVSteeringType::ACKERMANN) : _steering_type(type) {}

    uint8_t get_num_channels() const override { return 2; }

    void output_to_actuators(const control::ControlDemands& demands, hal::PWMOutput* pwm, bool armed) override {
        if (!pwm) return;

        if (!armed) {
            pwm->set_channel(0, 1500); // Neutral / center
            pwm->set_channel(1, 1500); // Neutral / stop
            return;
        }

        if (_steering_type == UGVSteeringType::ACKERMANN) {
            // Channel 0: Steering (demands.yaw)
            pwm->set_channel(0, map_bidirectional_pwm(demands.yaw));
            // Channel 1: Throttle / Forward (demands.throttle or demands.forward)
            pwm->set_channel(1, map_bidirectional_pwm(demands.forward != 0.0f ? demands.forward : demands.throttle));
        } else {
            // Skid Steer differential mixing
            float forward = demands.forward != 0.0f ? demands.forward : demands.throttle;
            float turn = demands.yaw;

            // Positive yaw = turn right (FRD): the left side must drive faster.
            // (Was inverted, so heading hold steered away from the target.)
            float left_motor = forward + turn;
            float right_motor = forward - turn;

            pwm->set_channel(0, map_bidirectional_pwm(left_motor));
            pwm->set_channel(1, map_bidirectional_pwm(right_motor));
        }
    }

private:
    UGVSteeringType _steering_type{UGVSteeringType::ACKERMANN};
};

} // namespace vehicles
} // namespace autopilot
