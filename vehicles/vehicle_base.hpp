#pragma once

#include "hal/hal.hpp"
#include "drivers/sensor_manager.hpp"
#include "fusion/ahrs_interface.hpp"
#include "control/arming_handler.hpp"
#include "control/vehicle_mixer.hpp"
#include "control/failsafe.hpp"
#include <string>

namespace autopilot {
namespace vehicles {

enum class VehicleType {
    UAV_MULTICOPTER,
    UGV_GROUND_ROVER,
    ROV_SUBMARINE,
    USV_SURFACE_BOAT   // runs the rover controller (heading/speed hold) on a hull
};

class VehicleBase {
public:
    VehicleBase(hal::HAL* hal, VehicleType type);
    virtual ~VehicleBase() = default;

    /**
     * Use an externally owned sensor + fusion stack (VehicleManager does this so
     * that one stack serves every vehicle profile). Must be called before init().
     * When attached, the owner is responsible for updating sensors and fusion;
     * the vehicle only runs its controller, failsafe and mixer.
     */
    void attach_shared_stack(drivers::SensorManager* sensors, fusion::SensorFusion* fusion);
    bool uses_shared_stack() const { return _owned_sensors == nullptr; }

    virtual bool init();

    /**
     * @brief Main periodic flight loop step (called at 100Hz - 400Hz).
     */
    virtual void step(float dt);

    // Vehicle control commands
    virtual bool arm(std::string& failure_reason);
    virtual void disarm();
    virtual bool is_armed() const { return _arming.is_armed(); }
    /** True for a multirotor that is flying: lets the fusion use its rotor-drag model. */
    virtual bool multirotor_in_air() const { return false; }

    VehicleType get_type() const { return _type; }
    virtual const char* get_type_name() const = 0;

    // Subsystem accessors
    hal::HAL* get_hal() { return _hal; }
    drivers::SensorManager* get_sensors() { return _sensors; }
    fusion::SensorFusion* get_fusion() { return _fusion; }
    control::ArmingHandler* get_arming() { return &_arming; }
    control::VehicleMixer* get_mixer() { return _mixer.get(); }
    const control::VehicleMixer* get_mixer() const { return _mixer.get(); }
    const fusion::VehicleState& get_state() const { return _fusion->get_state(); }

    // User command inputs (Normalized [-1.0, 1.0]). Each call also counts as an
    // RC heartbeat for the failsafe.
    void set_user_inputs(float roll, float pitch, float yaw, float throttle, float forward = 0.0f, float lateral = 0.0f);

    /** Demands computed by the last step() (after failsafe overrides). */
    const control::ControlDemands& get_demands() const { return _demands; }

    // Failsafe
    control::Failsafe& get_failsafe() { return _failsafe; }
    const control::Failsafe& get_failsafe() const { return _failsafe; }
    /** Battery voltage from a power module / ADC; < 0 = no monitor. */
    void set_battery_voltage(float v) { _battery_voltage = v; }
    /** Carry the RC link state over when VehicleManager swaps vehicle profiles. */
    void inherit_rc_link(const VehicleBase& other) {
        _last_rc_ms = other._last_rc_ms;
        _rc_received = other._rc_received;
        _user_input = other._user_input;
    }

protected:
    hal::HAL* _hal{nullptr};
    VehicleType _type;

    // Standalone vehicles own their stack; under VehicleManager these are null
    // and _sensors/_fusion point at the manager's shared instances.
    std::unique_ptr<drivers::SensorManager> _owned_sensors;
    std::unique_ptr<fusion::SensorFusion> _owned_fusion;
    drivers::SensorManager* _sensors{nullptr};
    fusion::SensorFusion* _fusion{nullptr};
    control::ArmingHandler _arming;
    std::unique_ptr<control::VehicleMixer> _mixer;

    control::ControlDemands _user_input;
    control::ControlDemands _demands;

    control::Failsafe _failsafe;
    uint32_t _last_rc_ms{0};
    bool _rc_received{false};
    float _battery_voltage{-1.0f};

    uint32_t now_ms() const;

    /**
     * Vehicle-specific "sticks are safe" arming check (e.g. throttle low on a
     * multicopter), so arming can never make the vehicle jump. Default: OK.
     */
    virtual bool check_arming_inputs(std::string& failure_reason) const {
        (void)failure_reason;
        return true;
    }

    /** Called when arming succeeds / on disarm: reset controller state. */
    virtual void on_arm() {}
    virtual void on_disarm() {}

    /**
     * Vehicle-specific reaction to a failsafe LAND. Default: neutral (stop).
     * Returns true when the vehicle has finished (landed) and should disarm.
     */
    virtual bool apply_failsafe_land(float dt);

    /**
     * @brief Vehicle-specific controller step.
     * Takes user input + vehicle state -> computes _demands.
     */
    virtual void update_controller(float dt) = 0;
};

} // namespace vehicles
} // namespace autopilot
