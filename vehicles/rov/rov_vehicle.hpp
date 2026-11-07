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

enum class ROVLayout {
    AUTO,          // Selected automatically from thruster count
    SIMPLE_3,      // 3 Thrusters: 2 Differential Horiz + 1 Vert Heave
    BASIC_4,       // 4 Thrusters: 2 Differential Horiz + 2 Vert (Roll + Heave)
    VECTORED_6,    // 6 Thrusters: 4 Vectored 45-deg Horiz + 2 Vert (BlueROV2)
    HEAVY_8,       // 8 Thrusters: 4 Vectored Horiz + 4 Vert (Decoupled 6-DOF BlueROV2 Heavy & RoboSub)
    CUSTOM_N       // Custom user-defined allocation matrix (up to 16)
};

class ROVVehicle : public VehicleBase {
public:
    explicit ROVVehicle(hal::HAL* hal, uint8_t initial_thrusters = 6);

    const char* get_type_name() const override;

    void set_mode(ROVMode mode) { _mode = mode; }
    ROVMode get_mode() const { return _mode; }

    void set_target_depth(float depth_m) { _target_depth_m = depth_m; }

    /**
     * @brief Runtime interface option to choose the number of thrusters (2 to 16).
     * @param count Number of thrusters (e.g. 3, 4, 6, 8, up to 16).
     * @param layout Optional layout preset (defaults to AUTO).
     */
    bool set_thruster_count(uint8_t count, ROVLayout layout = ROVLayout::AUTO);
    uint8_t get_thruster_count() const { return _thruster_count; }
    ROVLayout get_thruster_layout() const { return _layout; }
    const char* get_thruster_layout_name() const;

    /**
     * @brief Configure custom allocation matrix row for a specific thruster channel.
     */
    bool set_custom_thruster(uint8_t channel, float roll, float pitch, float yaw, float heave, float surge, float sway);

    control::ActuatorMatrixMixer* get_matrix_mixer() { return _matrix_mixer; }

protected:
    void update_controller(float dt) override;

private:
    ROVMode _mode{ROVMode::STABILIZE};
    float _target_depth_m{0.0f};

    uint8_t _thruster_count{6};
    ROVLayout _layout{ROVLayout::VECTORED_6};
    control::ActuatorMatrixMixer* _matrix_mixer{nullptr};

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
