#pragma once

#include "hal/hal.hpp"
#include "drivers/sensor_manager.hpp"
#include "fusion/ahrs_interface.hpp"
#include "control/arming_handler.hpp"
#include "control/vehicle_mixer.hpp"
#include "system/pubsub.hpp"
#include "vehicles/vehicle_base.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/ugv/ugv_vehicle.hpp"
#include "vehicles/rov/rov_vehicle.hpp"
#include <memory>
#include <string>

namespace autopilot {
namespace vehicles {

/**
 * @brief Unified Dynamic Vehicle Manager powered by Publish/Subscribe Middleware.
 * Solves ArduPilot's flaw of requiring separate firmware binaries per vehicle.
 * Allows a single compiled binary to dynamically switch between UAV, UGV, and ROV
 * profiles at runtime without reflashing or rebooting the microcontroller.
 */
class VehicleManager {
public:
    VehicleManager(hal::HAL* hal, VehicleType initial_type = VehicleType::UAV_MULTICOPTER);
    ~VehicleManager() = default;

    bool init();

    /**
     * @brief Dynamically switch vehicle profile at runtime.
     * Safely disarms actuators before reconfiguring controllers and mixers.
     */
    bool switch_vehicle(VehicleType new_type, std::string& reason);

    /**
     * @brief Periodic execution step (called at 100Hz - 400Hz).
     * 1. Updates sensors & SensorFusion
     * 2. Publishes VehicleState to state_topic
     * 3. Controller consumes state, computes & publishes ControlDemands
     * 4. Mixer consumes demands and writes PWM outputs to hardware pins
     */
    void step(float dt);

    // Actuator arming & safety
    bool arm(std::string& failure_reason);
    void disarm();
    bool is_armed() const;

    VehicleType get_active_type() const { return _active_type; }
    const char* get_active_type_name() const;

    // User input command routing
    void set_user_inputs(float roll, float pitch, float yaw, float throttle, float forward = 0.0f, float lateral = 0.0f);

    /**
     * @brief Interface option to dynamically choose number of thrusters for ROV (2 to 16).
     * @param count Number of thrusters (3 = simple, 4 = basic, 6 = vectored BlueROV2, 8 = heavy AUV 6-DOF, up to 16).
     * @param layout Optional layout preset (defaults to AUTO).
     */
    bool configure_rov_thrusters(uint8_t count, ROVLayout layout = ROVLayout::AUTO);

    /**
     * @brief Interface option to dynamically choose number of rotors for UAV (3 to 16).
     * @param count Number of rotors (3 = Tricopter, 4 = Quad-X, 6 = Hexa-X, 8 = Octa-X, up to 16).
     */
    bool configure_uav_rotors(uint8_t count);

    /**
     * @brief Get active number of actuator channels in use.
     */
    uint8_t get_active_actuator_count() const;

    // Subsystem accessors
    hal::HAL* get_hal() { return _hal; }
    drivers::SensorManager* get_sensors() { return _sensors.get(); }
    fusion::SensorFusion* get_fusion() { return _fusion.get(); }
    control::ArmingHandler* get_arming() { return _arming.get(); }
    VehicleBase* get_active_vehicle() { return _active_vehicle.get(); }
    const fusion::VehicleState& get_latest_state() const { return _fusion->get_state(); }

    // Pub/Sub Topics for external subscribers (e.g. Telemetry, Loggers, GCS)
    system::Topic<fusion::VehicleState>& get_state_topic() { return _state_topic; }
    system::Topic<control::ControlDemands>& get_demands_topic() { return _demands_topic; }

private:
    hal::HAL* _hal{nullptr};
    VehicleType _active_type;

    // Shared flight stack core
    std::unique_ptr<drivers::SensorManager> _sensors;
    std::unique_ptr<fusion::SensorFusion> _fusion;
    std::unique_ptr<control::ArmingHandler> _arming;

    // Pub/Sub Message Topics
    system::Topic<fusion::VehicleState> _state_topic;
    system::Topic<control::ControlDemands> _demands_topic;

    // Subscriber handles
    typename system::Topic<fusion::VehicleState>::Subscriber _controller_state_sub;
    typename system::Topic<control::ControlDemands>::Subscriber _mixer_demands_sub;

    // Active vehicle controller & mixer
    std::unique_ptr<VehicleBase> _active_vehicle;
    std::unique_ptr<control::VehicleMixer> _active_mixer;

    void instantiate_vehicle(VehicleType type);
};

} // namespace vehicles
} // namespace autopilot

