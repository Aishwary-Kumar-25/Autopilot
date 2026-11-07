#pragma once

#include "vehicles/vehicle_base.hpp"
#include "uav_mixer.hpp"
#include "math/pid.hpp"

namespace autopilot {
namespace vehicles {

enum class UAVFlightMode {
    MANUAL,
    STABILIZE,
    ALT_HOLD
};

class UAVVehicle : public VehicleBase {
public:
    UAVVehicle(hal::HAL* hal);

    const char* get_type_name() const override { return "UAV Multicopter (Quad-X)"; }

    void set_mode(UAVFlightMode mode) { _mode = mode; }
    UAVFlightMode get_mode() const { return _mode; }

    void set_target_altitude(float target_alt_m) { _target_altitude_m = target_alt_m; }

protected:
    void update_controller(float dt) override;

private:
    UAVFlightMode _mode{UAVFlightMode::STABILIZE};
    float _target_altitude_m{1.0f};

    // Cascaded Attitude Controllers
    math::PID _roll_angle_pid;
    math::PID _pitch_angle_pid;
    math::PID _roll_rate_pid;
    math::PID _pitch_rate_pid;
    math::PID _yaw_rate_pid;

    // Altitude Controller
    math::PID _altitude_pid;
    math::PID _velocity_z_pid;

    void setup_pids();
};

} // namespace vehicles
} // namespace autopilot
