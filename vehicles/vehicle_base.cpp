#include "vehicle_base.hpp"

namespace autopilot {
namespace vehicles {

VehicleBase::VehicleBase(hal::HAL* hal, VehicleType type)
    : _hal(hal),
      _type(type),
      _owned_sensors(std::make_unique<drivers::SensorManager>(hal)),
      _owned_fusion(std::make_unique<fusion::SensorFusion>(_owned_sensors.get())),
      _sensors(_owned_sensors.get()),
      _fusion(_owned_fusion.get()),
      _arming(_owned_sensors.get()) {}

void VehicleBase::attach_shared_stack(drivers::SensorManager* sensors, fusion::SensorFusion* fusion) {
    if (!sensors || !fusion) return;
    _sensors = sensors;
    _fusion = fusion;
    _arming = control::ArmingHandler(sensors);
    _owned_fusion.reset();   // fusion references the owned sensors: drop it first
    _owned_sensors.reset();
}

uint32_t VehicleBase::now_ms() const {
    auto* t = _hal ? _hal->get_time() : nullptr;
    return t ? t->millis() : 0;
}

bool VehicleBase::init() {
    if (!_hal) return false;

    if (!uses_shared_stack()) {
        // 1. Initialize HAL peripherals
        if (!_hal->init()) {
            return false;
        }

        // 2. Discover and initialize plug-and-play sensors on buses
        _sensors->auto_discover_sensors();

        // 3. Reset Sensor Fusion state estimator
        _fusion->reset();
    }

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
    _last_rc_ms = now_ms();
    _rc_received = true;
}

bool VehicleBase::arm(std::string& failure_reason) {
    if (_arming.is_armed()) return true;
    if (!_rc_received || now_ms() - _last_rc_ms > _failsafe.config().rc_timeout_ms) {
        failure_reason = "PreArm: no RC input";
        return false;
    }
    if (_arming.arm(failure_reason)) {
        _failsafe.reset();
        _sensors->set_armed(true);   // latch sensor failures while flying
        on_arm();
        if (auto* pwm = _hal->get_pwm()) {
            pwm->enable_output(true);
        }
        return true;
    }
    return false;
}

void VehicleBase::disarm() {
    _arming.disarm();
    _sensors->set_armed(false);
    _failsafe.reset();
    on_disarm();
    _demands = control::ControlDemands{};
    if (auto* pwm = _hal->get_pwm()) {
        pwm->enable_output(false);
    }
}

bool VehicleBase::apply_failsafe_land(float) {
    _demands = control::ControlDemands{};
    return false; // ground/sub vehicles just hold neutral until the pilot disarms
}

void VehicleBase::step(float dt) {
    if (!uses_shared_stack()) {
        // 1. Update sensors
        _sensors->update();

        // 2. Update universal sensor fusion engine
        _fusion->update(dt);

        // 3. Periodic low-rate plug-and-play sensor recovery check
        _sensors->probe_and_recover_sensors();
    }

    // 4. Failsafe decides first; it overrides the pilot / controller
    control::FailsafeAction fs = control::FailsafeAction::NONE;
    if (_arming.is_armed()) {
        control::FailsafeInputs in;
        in.now_ms = now_ms();
        in.last_rc_ms = _last_rc_ms;
        in.rc_ever_received = _rc_received;
        in.battery_voltage = _battery_voltage;
        in.attitude_valid = get_state().attitude_valid;
        in.imu_usable = _sensors->has_healthy_imu();
        fs = _failsafe.update(in);
    }

    // 5. Vehicle-specific control logic (exactly one controller update per step)
    switch (fs) {
        case control::FailsafeAction::TERMINATE:
            disarm();
            break;
        case control::FailsafeAction::LAND:
            if (apply_failsafe_land(dt)) {
                disarm();   // landed: stop the motors
            }
            break;
        case control::FailsafeAction::NONE:
            update_controller(dt);
            break;
    }

    // 6. Output to actuators via vehicle mixer
    if (_mixer && _hal) {
        auto* pwm = _hal->get_pwm();
        bool armed = _arming.is_armed();
        _mixer->output_to_actuators(_demands, pwm, armed);
    }
}

} // namespace vehicles
} // namespace autopilot
