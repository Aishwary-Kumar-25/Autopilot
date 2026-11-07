#pragma once

#include "hal/hal.hpp"
#include "drivers/sensor_manager.hpp"
#include "fusion/ahrs_interface.hpp"
#include "control/arming_handler.hpp"
#include "control/vehicle_mixer.hpp"
#include <string>

namespace autopilot {
namespace vehicles {

enum class VehicleType {
    UAV_MULTICOPTER,
    UGV_GROUND_ROVER,
    ROV_SUBMARINE
};

class VehicleBase {
public:
    VehicleBase(hal::HAL* hal, VehicleType type);
    virtual ~VehicleBase() = default;

    virtual bool init();

    /**
     * @brief Main periodic flight loop step (called at 100Hz - 400Hz).
     */
    virtual void step(float dt);

    // Vehicle control commands
    virtual bool arm(std::string& failure_reason);
    virtual void disarm();
    virtual bool is_armed() const { return _arming.is_armed(); }

    VehicleType get_type() const { return _type; }
    virtual const char* get_type_name() const = 0;

    // Subsystem accessors
    hal::HAL* get_hal() { return _hal; }
    drivers::SensorManager* get_sensors() { return _sensors.get(); }
    fusion::SensorFusion* get_fusion() { return _fusion.get(); }
    control::ArmingHandler* get_arming() { return &_arming; }
    control::VehicleMixer* get_mixer() { return _mixer.get(); }
    const control::VehicleMixer* get_mixer() const { return _mixer.get(); }
    const fusion::VehicleState& get_state() const { return _fusion->get_state(); }

    // User command inputs (Normalized [-1.0, 1.0])
    void set_user_inputs(float roll, float pitch, float yaw, float throttle, float forward = 0.0f, float lateral = 0.0f);

protected:
    hal::HAL* _hal{nullptr};
    VehicleType _type;

    std::unique_ptr<drivers::SensorManager> _sensors;
    std::unique_ptr<fusion::SensorFusion> _fusion;
    control::ArmingHandler _arming;
    std::unique_ptr<control::VehicleMixer> _mixer;

    control::ControlDemands _user_input;
    control::ControlDemands _demands;

    /**
     * @brief Vehicle-specific controller step.
     * Takes user input + vehicle state -> computes _demands.
     */
    virtual void update_controller(float dt) = 0;
};

} // namespace vehicles
} // namespace autopilot
