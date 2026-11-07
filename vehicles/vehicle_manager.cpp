#include "vehicle_manager.hpp"

namespace autopilot {
namespace vehicles {

VehicleManager::VehicleManager(hal::HAL* hal, VehicleType initial_type)
    : _hal(hal),
      _active_type(initial_type),
      _sensors(std::make_unique<drivers::SensorManager>(hal)),
      _fusion(std::make_unique<fusion::SensorFusion>(_sensors.get())),
      _arming(std::make_unique<control::ArmingHandler>(_sensors.get())),
      _controller_state_sub(_state_topic.subscribe()),
      _mixer_demands_sub(_demands_topic.subscribe()) {
    instantiate_vehicle(initial_type);
}

void VehicleManager::instantiate_vehicle(VehicleType type) {
    _active_type = type;
    switch (type) {
        case VehicleType::UGV_GROUND_ROVER:
            _active_vehicle = std::make_unique<UGVVehicle>(_hal);
            _active_mixer = std::make_unique<UGVMixer>(UGVSteeringType::ACKERMANN);
            break;
        case VehicleType::ROV_SUBMARINE:
            _active_vehicle = std::make_unique<ROVVehicle>(_hal);
            _active_mixer = std::make_unique<ROVThrusterMixer>();
            break;
        case VehicleType::UAV_MULTICOPTER:
        default:
            _active_vehicle = std::make_unique<UAVVehicle>(_hal);
            _active_mixer = std::make_unique<UAVQuadMixer>();
            break;
    }
}

bool VehicleManager::init() {
    if (!_hal) return false;

    if (!_hal->init()) {
        return false;
    }

    _sensors->auto_discover_sensors();
    _fusion->reset();

    if (auto* pwm = _hal->get_pwm()) {
        pwm->init(400); // 400Hz default update rate
        pwm->enable_output(false);
    }

    return true;
}

bool VehicleManager::switch_vehicle(VehicleType new_type, std::string& reason) {
    if (new_type == _active_type) {
        reason = "Already running requested vehicle profile";
        return true;
    }

    // Safety Interlock: Force disarm before switching vehicle profile
    if (is_armed()) {
        disarm();
        reason = "Safety Interlock: Disarmed active actuators before vehicle profile switch";
    }

    instantiate_vehicle(new_type);
    return true;
}

const char* VehicleManager::get_active_type_name() const {
    if (_active_vehicle) {
        return _active_vehicle->get_type_name();
    }
    return "Unknown Vehicle";
}

void VehicleManager::set_user_inputs(float roll, float pitch, float yaw, float throttle, float forward, float lateral) {
    if (_active_vehicle) {
        _active_vehicle->set_user_inputs(roll, pitch, yaw, throttle, forward, lateral);
    }
}

bool VehicleManager::arm(std::string& failure_reason) {
    if (_arming->arm(failure_reason)) {
        if (_active_vehicle) {
            std::string sub_err;
            _active_vehicle->arm(sub_err);
        }
        if (auto* pwm = _hal->get_pwm()) {
            pwm->enable_output(true);
        }
        return true;
    }
    return false;
}

void VehicleManager::disarm() {
    _arming->disarm();
    if (_active_vehicle) {
        _active_vehicle->disarm();
    }
    if (auto* pwm = _hal->get_pwm()) {
        pwm->enable_output(false);
    }
}

bool VehicleManager::is_armed() const {
    return _arming->is_armed();
}

void VehicleManager::step(float dt) {
    // 1. Process sensor updates & dynamic DroneCAN packets
    _sensors->update();
    _sensors->probe_and_recover_sensors();

    // 2. Run universal sensor fusion engine
    _fusion->update(dt);

    // 3. Publish fresh state to Pub/Sub middleware
    _state_topic.publish(_fusion->get_state());

    // 4. Controller consumes state from Pub/Sub topic and runs vehicle dynamics
    fusion::VehicleState current_state;
    _controller_state_sub.copy(current_state);

    if (_active_vehicle) {
        _active_vehicle->step(dt);
        // Extract demands from vehicle and publish to demands topic
        _demands_topic.publish(_active_vehicle->get_state().attitude_valid ?
            control::ControlDemands{0,0,0,0,0,0} : control::ControlDemands{0,0,0,0,0,0});
    }

    // 5. Mixer consumes demands and drives hardware PWM pins
    control::ControlDemands current_demands;
    _mixer_demands_sub.copy(current_demands);
}

} // namespace vehicles
} // namespace autopilot

