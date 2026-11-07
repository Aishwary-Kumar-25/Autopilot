#pragma once

#include "vehicles/vehicle_base.hpp"
#include "rov_mixer.hpp"
#include "math/pid.hpp"

namespace autopilot {
namespace vehicles {

enum class ROVMode {
    MANUAL,
    STABILIZE,
    DEPTH_HOLD
};

class ROVVehicle : public VehicleBase {
public:
    ROVVehicle(hal::HAL* hal);

    const char* get_type_name() const override { return "ROV Submarine (ArduSub-Inspired 6-DOF)"; }

    void set_mode(ROVMode mode) { _mode = mode; }
    ROVMode get_mode() const { return _mode; }

    void set_target_depth(float depth_m) { _target_depth_m = depth_m; }

protected:
    void update_controller(float dt) override;

private:
    ROVMode _mode{ROVMode::STABILIZE};
    float _target_depth_m{0.0f};

    // Attitude stabilization
    math::PID _roll_pid;
    math::PID _pitch_pid;
    math::PID _yaw_pid;

    // Depth Controller (from ArduSub depth hold)
    math::PID _depth_pid;
    math::PID _depth_rate_pid;

    void setup_pids();
};

} // namespace vehicles
} // namespace autopilot
