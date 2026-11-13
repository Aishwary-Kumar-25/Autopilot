#include "uav_vehicle.hpp"
#include <cmath>
#include <cstdio>

namespace autopilot {
namespace vehicles {

UAVVehicle::UAVVehicle(hal::HAL* hal, uint8_t initial_rotors)
    : VehicleBase(hal, VehicleType::UAV_MULTICOPTER),
      _rotor_count(initial_rotors) {
    auto mixer = std::make_unique<UAVQuadMixer>(initial_rotors);
    _matrix_mixer = mixer.get();
    _mixer = std::move(mixer);
    setup_pids();
}

const char* UAVVehicle::get_type_name() const {
    switch (_rotor_count) {
        case 3:  return "UAV Multicopter (Tricopter)";
        case 4:  return "UAV Multicopter (Quad-X)";
        case 6:  return "UAV Multicopter (Hexa-X)";
        case 8:  return "UAV Multicopter (Octa-X)";
        default: return "UAV Multicopter (Multi-Rotor)";
    }
}

bool UAVVehicle::set_rotor_count(uint8_t count) {
    if (count < 3 || count > hal::PWMOutput::MAX_CHANNELS || !_matrix_mixer) {
        return false;
    }
    _rotor_count = count;
    return _matrix_mixer->configure_uav(count);
}

void UAVVehicle::setup_pids() {
    // Angle Loop (P-only or PI)
    _roll_angle_pid.set_gains(4.5f, 0.0f, 0.0f);
    _roll_angle_pid.set_limits(-4.0f, 4.0f, 0.0f); // Max rate 4 rad/s (~230 deg/s)

    _pitch_angle_pid.set_gains(4.5f, 0.0f, 0.0f);
    _pitch_angle_pid.set_limits(-4.0f, 4.0f, 0.0f);

    // Rate Loop (PID)
    _roll_rate_pid.set_gains(0.15f, 0.10f, 0.004f);
    _roll_rate_pid.set_limits(-0.5f, 0.5f, 0.2f);

    _pitch_rate_pid.set_gains(0.15f, 0.10f, 0.004f);
    _pitch_rate_pid.set_limits(-0.5f, 0.5f, 0.2f);

    _yaw_rate_pid.set_gains(0.20f, 0.05f, 0.0f);
    _yaw_rate_pid.set_limits(-0.5f, 0.5f, 0.2f);

    // Altitude and Climb Rate Loop
    _altitude_pid.set_gains(1.0f, 0.0f, 0.0f);
    _altitude_pid.set_limits(-2.0f, 2.0f, 0.0f); // Max climb/descend speed: 2 m/s

    _velocity_z_pid.set_gains(0.3f, 0.1f, 0.02f);
    _velocity_z_pid.set_limits(-0.3f, 0.4f, 0.2f); // Throttle adjustment
}

void UAVVehicle::reset_pids() {
    _roll_angle_pid.reset();
    _pitch_angle_pid.reset();
    _roll_rate_pid.reset();
    _pitch_rate_pid.reset();
    _yaw_rate_pid.reset();
    _altitude_pid.reset();
    _velocity_z_pid.reset();
}

void UAVVehicle::set_mode(UAVFlightMode mode) {
    if (mode == _mode) return;
    const auto& st = get_state();
    if (mode == UAVFlightMode::ALT_HOLD) {
        _target_altitude_m = st.altitude_valid ? st.altitude_m : _target_altitude_m;
        _altitude_pid.reset();
        _velocity_z_pid.reset();
        // Flying steadily on manual collective: that collective is the best hover guess.
        if (is_armed() && !_landed && std::fabs(st.vertical_velocity_m_s) < 0.5f && _last_throttle > 0.0f) {
            const auto& e = st.euler_rad;
            set_hover_throttle(_last_throttle * std::max(std::cos(e.x) * std::cos(e.y), 0.7f));
        }
    }
    _mode = mode;
}

bool UAVVehicle::check_arming_inputs(std::string& failure_reason) const {
    const float thr = _user_input.throttle;
    if (_mode == UAVFlightMode::ALT_HOLD) {
        // Mid stick holds; only above the deadband would it climb.
        if (thr > 0.5f + STICK_DEADBAND) {
            failure_reason = "PreArm: throttle above mid stick (ALT_HOLD)";
            return false;
        }
    } else if (thr >= THROTTLE_GROUND) {
        failure_reason = "PreArm: throttle not low";
        return false;
    }
    return true;
}

void UAVVehicle::on_arm() {
    // Integrators from the last flight (or from sitting on the bench) must not
    // carry into a new takeoff.
    reset_pids();
    // A new flight starts with every motor trusted again (and re-checked in the air).
    if (_matrix_mixer) _matrix_mixer->clear_motor_failures();
    _motor_fdi.reset();
    _motor_emergency = false;
    _last_failed_motor = -1;
    _motor_failure_ms = 0;
    _landed = true;
    _landed_timer_s = 0.0f;
    _last_throttle = 0.0f;
    _althold_thr_lp = 0.0f;
    const auto& st = get_state();
    _target_altitude_m = st.altitude_valid ? st.altitude_m : 0.0f;
}

void UAVVehicle::on_disarm() {
    reset_pids();
    _last_throttle = 0.0f;
}

void UAVVehicle::set_gyro_filters(float lpf_hz, float notch_hz, float notch_bw_hz, float sample_hz) {
    for (int i = 0; i < 3; ++i) {
        if (lpf_hz > 0.0f) _gyro_lpf[i].set_lowpass(lpf_hz, sample_hz); else _gyro_lpf[i].set_passthrough();
        if (notch_hz > 0.0f) _gyro_notch[i].set_notch(notch_hz, notch_bw_hz, sample_hz); else _gyro_notch[i].set_passthrough();
    }
}

math::Vector3f UAVVehicle::filtered_rates() {
    const auto& w = get_state().angular_velocity;
    return math::Vector3f(_gyro_lpf[0].apply(_gyro_notch[0].apply(w.x)),
                          _gyro_lpf[1].apply(_gyro_notch[1].apply(w.y)),
                          _gyro_lpf[2].apply(_gyro_notch[2].apply(w.z)));
}

void UAVVehicle::run_attitude(float target_roll, float target_pitch, float target_yaw_rate, float dt, bool integrate) {
    const auto& state = get_state();
    const math::Vector3f rates = filtered_rates();
    // Angle error -> desired body rate (P only), rate error -> torque demand (PID)
    const float desired_roll_rate = _roll_angle_pid.update(target_roll, state.euler_rad.x, dt, integrate);
    const float desired_pitch_rate = _pitch_angle_pid.update(target_pitch, state.euler_rad.y, dt, integrate);
    _demands.roll = _roll_rate_pid.update(desired_roll_rate, rates.x, dt, integrate);
    _demands.pitch = _pitch_rate_pid.update(desired_pitch_rate, rates.y, dt, integrate);
    if (yaw_abandoned()) {
        // Nothing can produce yaw torque any more: no loop, no windup.
        _yaw_rate_pid.reset();
        _demands.yaw = 0.0f;
    } else {
        _demands.yaw = _yaw_rate_pid.update(target_yaw_rate, rates.z, dt, integrate);
    }
    _demands.forward = 0.0f;
    _demands.lateral = 0.0f;
}

float UAVVehicle::tilt_compensate(float throttle) const {
    // Keep vertical thrust constant when tilted ("angle boost"), capped at ~45 deg.
    const auto& e = get_state().euler_rad;
    const float c = std::max(std::cos(e.x) * std::cos(e.y), 0.7f);
    return throttle / c;
}

float UAVVehicle::run_climb_rate(float desired_climb_m_s, float dt, bool integrate) {
    const auto& state = get_state();
    const float delta = _velocity_z_pid.update(desired_climb_m_s, state.vertical_velocity_m_s, dt, integrate);
    const float thr = std::clamp(tilt_compensate(_hover_throttle + delta), 0.0f, 0.95f);

    // Hover throttle learning (slow, only in steady hover): lets the vz integrator
    // stay near zero so ALT_HOLD behaves the same on light and heavy airframes.
    if (integrate && !_landed && std::fabs(state.vertical_velocity_m_s) < 0.3f &&
        std::fabs(desired_climb_m_s) < 0.2f) {
        constexpr float TAU_S = 5.0f;
        const float level_thr = _hover_throttle + delta;
        set_hover_throttle(_hover_throttle + (level_thr - _hover_throttle) * (dt / TAU_S));
    }
    return thr;
}

void UAVVehicle::update_land_detector(float desired_climb, float dt, bool require_low_thrust) {
    const auto& state = get_state();
    // Landed = we're asking to go down (or idle), the motors are near idle relative
    // to hover, and nothing is moving. Must hold for 1 s.
    const bool want_down = desired_climb <= 0.0f;
    const bool low_thrust = !require_low_thrust || _last_throttle < 0.6f * _hover_throttle;
    // With yaw abandoned the vehicle spins all the way down: only roll/pitch count.
    const auto& w = state.angular_velocity;
    const float rot = yaw_abandoned() ? std::sqrt(w.x * w.x + w.y * w.y) : w.length();
    const bool still = std::fabs(state.vertical_velocity_m_s) < 0.25f && rot < 0.5f;
    if (want_down && low_thrust && still) {
        _landed_timer_s += dt;
        if (_landed_timer_s >= (require_low_thrust ? 1.0f : 3.0f)) _landed = true;
    } else {
        _landed_timer_s = 0.0f;
        if (_last_throttle > 0.8f * _hover_throttle) _landed = false;
    }
}

void UAVVehicle::update_controller(float dt) {
    const auto& state = get_state();

    if (_mode == UAVFlightMode::MANUAL) {
        _demands = _user_input;
        _last_throttle = _demands.throttle;
        return;
    }

    const float target_roll = std::clamp(_user_input.roll, -1.0f, 1.0f) * MAX_ANGLE_RAD;
    const float target_pitch = std::clamp(_user_input.pitch, -1.0f, 1.0f) * MAX_ANGLE_RAD;
    const float target_yaw_rate = std::clamp(_user_input.yaw, -1.0f, 1.0f) * MAX_YAW_RATE;

    if (_mode == UAVFlightMode::ALT_HOLD && state.altitude_valid) {
        // Mid stick = hold altitude, outside the deadband = climb/descend rate.
        const float stick = std::clamp(_user_input.throttle, 0.0f, 1.0f) - 0.5f;
        float desired_climb;
        if (std::fabs(stick) > STICK_DEADBAND) {
            const float s = (std::fabs(stick) - STICK_DEADBAND) / (0.5f - STICK_DEADBAND);
            desired_climb = std::copysign(s * MAX_CLIMB_RATE, stick);
            _target_altitude_m = state.altitude_m;   // re-capture when the stick is released
        } else {
            desired_climb = _altitude_pid.update(_target_altitude_m, state.altitude_m, dt);
        }

        // On the ground, only a deliberate climb stick takes off (altitude noise
        // around the target must not lift the vehicle). No windup while waiting.
        if (_landed && stick <= STICK_DEADBAND) {
            _demands = control::ControlDemands{};
            reset_pids();
            _target_altitude_m = state.altitude_m;
            _last_throttle = 0.0f;
            return;
        }
        const bool integrate = !_landed;
        run_attitude(target_roll, target_pitch, target_yaw_rate, dt, integrate);
        _demands.throttle = run_climb_rate(desired_climb, dt, integrate);
        _last_throttle = _demands.throttle;
        if (!_landed) {
            const auto& e = state.euler_rad;
            const float level = _demands.throttle * std::max(std::cos(e.x) * std::cos(e.y), 0.7f);
            _althold_thr_lp = _althold_thr_lp <= 0.0f ? level : _althold_thr_lp + (level - _althold_thr_lp) * (dt / (1.0f + dt));
        }
        update_land_detector(desired_climb, dt);
        return;
    }

    // STABILIZE (or ALT_HOLD without an altitude estimate): pilot sets collective.
    float thr_in = std::clamp(_user_input.throttle, 0.0f, 1.0f);
    if (_mode == UAVFlightMode::ALT_HOLD) {
        // Altitude estimate lost (e.g. the baro died): the pilot is still holding
        // the stick at mid for "hold", so mid stick must mean "the collective that was
        // just holding altitude", not 50 % collective (which climbs or sinks away).
        // Piecewise linear: 0 -> 0, 0.5 -> that collective, 1 -> 1.
        const float mid = _althold_thr_lp > 0.05f ? std::clamp(_althold_thr_lp, 0.15f, 0.85f) : _hover_throttle;
        thr_in = thr_in <= 0.5f ? thr_in * 2.0f * mid : mid + (thr_in - 0.5f) * 2.0f * (1.0f - mid);
    }
    if (thr_in < THROTTLE_GROUND) {
        // Throttle at zero = sitting on the ground: no torque demands (the mixer
        // would otherwise lift motors to make room for them) and no windup.
        _demands = control::ControlDemands{};
        reset_pids();
        _landed = true;
        _last_throttle = 0.0f;
        return;
    }
    // Integrate only once there's enough thrust to be flying.
    const bool integrate = thr_in > 0.5f * _hover_throttle;
    if (integrate) _landed = false;
    run_attitude(target_roll, target_pitch, target_yaw_rate, dt, integrate);
    _demands.throttle = std::clamp(tilt_compensate(thr_in), 0.0f, 1.0f);
    _last_throttle = _demands.throttle;
}

bool UAVVehicle::apply_failsafe_land(float dt) {
    const auto& state = get_state();
    if (_landed && _last_throttle <= 0.0f) {
        _demands = control::ControlDemands{};
        return true;   // already on the ground
    }
    // Level, no yaw, descend.
    run_attitude(0.0f, 0.0f, 0.0f, dt, true);
    if (state.altitude_valid) {
        _demands.throttle = run_climb_rate(-LAND_SPEED, dt, true);
    } else {
        // No altitude estimate: open-loop gentle descent just below hover.
        _demands.throttle = std::clamp(tilt_compensate(0.85f * _hover_throttle), 0.0f, 1.0f);
    }
    _last_throttle = _demands.throttle;
    // Without altitude the throttle never drops, so "still for 3 s" alone means landed.
    update_land_detector(-LAND_SPEED, dt, state.altitude_valid);
    _target_altitude_m = state.altitude_m;
    return _landed;
}

bool UAVVehicle::yaw_abandoned() const {
    if (!_matrix_mixer) return false;
    const auto st = _matrix_mixer->allocation_status();
    return st == control::AllocationStatus::NO_YAW || st == control::AllocationStatus::UNCONTROLLABLE;
}

void UAVVehicle::step(float dt) {
    VehicleBase::step(dt);
    if (!_motor_fdi_enabled || !_matrix_mixer || !is_armed()) return;
    // After a failure that already costs yaw (or more), the vehicle spins and the
    // detector's assumptions no longer hold: don't stack further verdicts on it.
    const auto status = _matrix_mixer->allocation_status();
    if (status != control::AllocationStatus::NOMINAL && status != control::AllocationStatus::DEGRADED) return;
    // MANUAL doesn't run the land detector: there, flying = real collective.
    const bool flying = _mode == UAVFlightMode::MANUAL ? _last_throttle > 0.2f : !_landed;
    const int m = _motor_fdi.update(*_matrix_mixer, get_state().angular_velocity, _demands.throttle, dt, flying,
                                    now_ms());
    if (m >= 0) declare_motor_failed(static_cast<uint8_t>(m), _motor_fdi.last_cause());
}

void UAVVehicle::declare_motor_failed(uint8_t channel, control::MotorFailureDetector::Cause cause) {
    if (!_matrix_mixer || _matrix_mixer->is_motor_failed(channel)) return;
    if (!_matrix_mixer->set_motor_failed(channel)) return;
    _last_failed_motor = static_cast<int8_t>(channel);
    _motor_failure_ms = now_ms();
    _motor_fdi.reset();

    // The yaw integrator was fighting the failure: start clean. Roll/pitch keep
    // theirs (the re-allocated mixer gives the same torque per unit demand).
    _yaw_rate_pid.reset();

    const auto status = _matrix_mixer->allocation_status();
    // Enough collective left to hover with some margin for control?
    const bool thrust_ok = _matrix_mixer->collective_limit() >= 1.15f * _hover_throttle;
    const bool emergency = status != control::AllocationStatus::DEGRADED || !thrust_ok;
    if (emergency) _motor_emergency = true;

    char t[51];
    std::snprintf(t, sizeof(t), "Motor %u failed (%s)", static_cast<unsigned>(channel + 1),
                  control::motor_failure_cause_name(cause));
    post_event(2 /* MAV_SEVERITY_CRITICAL */, t);
    if (status == control::AllocationStatus::DEGRADED && thrust_ok) {
        post_event(4 /* WARNING */, "Flying on remaining motors, yaw degraded: land");
    } else if (status == control::AllocationStatus::NO_YAW) {
        post_event(1 /* ALERT */, "No yaw control left: spinning, landing now");
    } else if (status == control::AllocationStatus::UNCONTROLLABLE) {
        post_event(0 /* EMERGENCY */, "Not enough motors left: emergency descent");
    } else {
        post_event(1 /* ALERT */, "Not enough thrust left: landing now");
    }
}

} // namespace vehicles
} // namespace autopilot
