#include "vehicle_manager.hpp"

namespace autopilot {
namespace vehicles {

VehicleManager::VehicleManager(hal::HAL* hal, VehicleType initial_type)
    : _hal(hal),
      _active_type(initial_type),
      _sensors(std::make_unique<drivers::SensorManager>(hal)),
      _fusion(std::make_unique<fusion::SensorFusion>(_sensors.get())),
      _controller_state_sub(_state_topic.subscribe()),
      _mixer_demands_sub(_demands_topic.subscribe()) {
    instantiate_vehicle(initial_type);
}

void VehicleManager::instantiate_vehicle(VehicleType type) {
    _active_type = type;
    std::unique_ptr<VehicleBase> previous = std::move(_active_vehicle);
    switch (type) {
        case VehicleType::UGV_GROUND_ROVER:
            _active_vehicle = std::make_unique<UGVVehicle>(_hal);
            break;
        case VehicleType::ROV_SUBMARINE:
            _active_vehicle = std::make_unique<ROVVehicle>(_hal);
            break;
        case VehicleType::USV_SURFACE_BOAT:
            _active_vehicle = std::make_unique<USVVehicle>(_hal);
            break;
        case VehicleType::UAV_MULTICOPTER:
        default:
            _active_vehicle = std::make_unique<UAVVehicle>(_hal);
            break;
    }
    if (_active_vehicle) {
        _active_vehicle->attach_shared_stack(_sensors.get(), _fusion.get());
        _active_vehicle->set_battery_voltage(_battery_voltage);
        if (previous) _active_vehicle->inherit_rc_link(*previous);
        if (_initialized) {
            _active_vehicle->init();
        }
    }
}

void VehicleManager::set_battery_voltage(float v) {
    _battery_voltage = v;
    if (_active_vehicle) _active_vehicle->set_battery_voltage(v);
}

bool VehicleManager::init() {
    if (!_hal) return false;

    if (!_hal->init()) {
        return false;
    }

    _sensors->auto_discover_sensors();
    _fusion->reset();

    _initialized = true;
    // Vehicle init only sets up its PWM outputs (the stack is shared).
    return _active_vehicle ? _active_vehicle->init() : true;
}

bool VehicleManager::switch_vehicle(VehicleType new_type, std::string& reason) {
    if (new_type == _active_type) {
        reason = "Already running requested vehicle profile";
        return true;
    }

    // Safety interlock: refuse while armed. Silently disarming here would cut the
    // motors of a flying multicopter (or a sub at depth) on a single bad command.
    if (is_armed()) {
        reason = "Refused: disarm before switching vehicle profile";
        return false;
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

bool VehicleManager::configure_rov_thrusters(uint8_t count, ROVLayout layout) {
    if (is_armed()) return false; // changing the allocation matrix under running thrusters is unsafe
    if (_active_type != VehicleType::ROV_SUBMARINE) {
        std::string reason;
        if (!switch_vehicle(VehicleType::ROV_SUBMARINE, reason)) {
            return false;
        }
    }
    auto* rov = dynamic_cast<ROVVehicle*>(_active_vehicle.get());
    if (rov) {
        return rov->set_thruster_count(count, layout);
    }
    return false;
}

bool VehicleManager::configure_uav_rotors(uint8_t count) {
    if (is_armed()) return false; // changing the motor layout in flight is unsafe
    if (_active_type != VehicleType::UAV_MULTICOPTER) {
        std::string reason;
        if (!switch_vehicle(VehicleType::UAV_MULTICOPTER, reason)) {
            return false;
        }
    }
    auto* uav = dynamic_cast<UAVVehicle*>(_active_vehicle.get());
    if (uav) {
        return uav->set_rotor_count(count);
    }
    return false;
}

uint8_t VehicleManager::get_active_actuator_count() const {
    if (_active_vehicle && _active_vehicle->get_mixer()) {
        return _active_vehicle->get_mixer()->get_num_channels();
    }
    return 0;
}

bool VehicleManager::arm(std::string& failure_reason) {
    // Single source of truth: the active vehicle's arming handler (one prearm
    // check, one armed flag, failsafe reset and PWM enable in one place).
    if (!_active_vehicle) {
        failure_reason = "No active vehicle";
        return false;
    }
    return _active_vehicle->arm(failure_reason);
}

void VehicleManager::disarm() {
    if (_active_vehicle) {
        _active_vehicle->disarm();
    }
}

bool VehicleManager::is_armed() const {
    return _active_vehicle && _active_vehicle->is_armed();
}

void VehicleManager::step(float dt) {
    // 1. Process sensor updates & dynamic DroneCAN packets
    _sensors->update();
    _sensors->probe_and_recover_sensors();

    // 2. Run universal sensor fusion engine
    _fusion->update(dt);

    // 3. Publish fresh state to Pub/Sub middleware
    _state_topic.publish(_fusion->get_state());

    // 4. Vehicle runs controller + failsafe + mixer on the shared state
    if (_active_vehicle) {
        _active_vehicle->step(dt);
        // 5. Publish the demands actually sent to the mixer (telemetry / logging)
        _demands_topic.publish(_active_vehicle->get_demands());
    }
}

} // namespace vehicles
} // namespace autopilot

