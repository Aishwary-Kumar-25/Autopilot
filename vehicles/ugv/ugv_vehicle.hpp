#pragma once

#include "vehicles/vehicle_base.hpp"
#include "ugv_mixer.hpp"
#include "math/pid.hpp"

namespace autopilot {
namespace vehicles {

enum class UGVMode {
    MANUAL,
    HEADING_HOLD,
    SPEED_HOLD
};

class UGVVehicle : public VehicleBase {
public:
    UGVVehicle(hal::HAL* hal, UGVSteeringType steering_type = UGVSteeringType::ACKERMANN);

    const char* get_type_name() const override { return "UGV Ground Vehicle (Rover)"; }

    /** Entering HEADING_HOLD/SPEED_HOLD captures the current heading as the target. */
    void set_mode(UGVMode mode);
    UGVMode get_mode() const { return _mode; }

    void set_target_heading(float heading_rad) { _target_heading_rad = heading_rad; }
    void set_target_speed(float speed_m_s) { _target_speed_m_s = speed_m_s; }

    /** Heading-rate damping gain (steering per rad/s of yaw rate). */
    static constexpr float HEADING_KD = 0.08f;
    static constexpr float STEER_DEADBAND = 0.05f; // yaw stick beyond this steers directly in HEADING/SPEED_HOLD

protected:
    /** For vehicles that reuse the rover controller (the boat). */
    UGVVehicle(hal::HAL* hal, UGVSteeringType steering_type, VehicleType type);

    void update_controller(float dt) override;
    bool check_arming_inputs(std::string& failure_reason) const override;
    void on_arm() override;

private:
    UGVMode _mode{UGVMode::MANUAL};
    float _target_heading_rad{0.0f};
    float _target_speed_m_s{0.0f};

    math::PID _heading_pid;
    math::PID _speed_pid;

    void setup_pids();
};

} // namespace vehicles
} // namespace autopilot
