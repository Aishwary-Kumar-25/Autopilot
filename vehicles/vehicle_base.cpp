#include "vehicle_base.hpp"

namespace autopilot {
namespace vehicles {

VehicleBase::VehicleBase(hal::HAL* hal, VehicleType type)
    : _hal(hal),
      _type(type),
      _sensors(std::make_unique<drivers::SensorManager>(hal)),
      _fusion(std::make_unique<fusion::SensorFusion>(_sensors.get())),
      _arming(_sensors.get()) {}

bool VehicleBase::init() {
    if (!_hal) return false;

    // 1. Initialize HAL peripherals
    if (!_hal->init()) {
        return false;
    }

    // 2. Discover and initialize plug-and-play sensors on buses
    _sensors->auto_discover_sensors();

    // 3. Reset Sensor Fusion state estimator
    _fusion->reset();

    // 4. Initialize PWM hardware to standard disarmed output
    if (auto* pwm = _hal->get_pwm()) {
        pwm->init(400); // 400Hz standard update rate
        pwm->enable_output(false);
    }

    return true;
}

void VehicleBase::set_user_inputs(float roll, float pitch, float yaw, float throttle, float forward, float lateral) {
    _user_input.roll = roll;
    _user_input.pitch = pitch;
    _user_input.yaw = yaw;
    _user_input.throttle = throttle;
    _user_input.forward = forward;
    _user_input.lateral = lateral;
}

bool VehicleBase::arm(std::string& failure_reason) {
    if (_arming.arm(failure_reason)) {
        if (auto* pwm = _hal->get_pwm()) {
            pwm->enable_output(true);
        }
        return true;
    }
    return false;
}

void VehicleBase::disarm() {
    _arming.disarm();
    if (auto* pwm = _hal->get_pwm()) {
        pwm->enable_output(false);
    }
}

void VehicleBase::step(float dt) {
    // 1. Update sensors
    _sensors->update();

    // 2. Update universal sensor fusion engine
    _fusion->update(dt);

    // 3. Periodic low-rate plug-and-play sensor recovery check
    _sensors->probe_and_recover_sensors();

    // 4. Vehicle-specific control logic
    update_controller(dt);

    // 5. Output to actuators via vehicle mixer
    if (_mixer && _hal) {
        auto* pwm = _hal->get_pwm();
        bool armed = _arming.is_armed();
        _mixer->output_to_actuators(_demands, pwm, armed);
    }
}

} // namespace vehicles
} // namespace autopilot
