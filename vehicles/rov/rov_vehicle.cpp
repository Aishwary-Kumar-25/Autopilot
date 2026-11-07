#include "rov_vehicle.hpp"

namespace autopilot {
namespace vehicles {

ROVVehicle::ROVVehicle(hal::HAL* hal, uint8_t initial_thrusters)
    : VehicleBase(hal, VehicleType::ROV_SUBMARINE) {
    auto mixer = std::make_unique<ROVThrusterMixer>(initial_thrusters);
    _matrix_mixer = mixer.get();
    _mixer = std::move(mixer);
    _thruster_count = initial_thrusters;
    _layout = (initial_thrusters == 3) ? ROVLayout::SIMPLE_3 :
              (initial_thrusters == 4) ? ROVLayout::BASIC_4 :
              (initial_thrusters == 8) ? ROVLayout::HEAVY_8 :
              (initial_thrusters == 6) ? ROVLayout::VECTORED_6 : ROVLayout::CUSTOM_N;
    setup_pids();
}

const char* ROVVehicle::get_type_name() const {
    switch (_layout) {
        case ROVLayout::SIMPLE_3:   return "ROV Submarine (3-Thruster Simple)";
        case ROVLayout::BASIC_4:    return "ROV Submarine (4-Thruster Basic)";
        case ROVLayout::VECTORED_6: return "ROV Submarine (ArduSub BlueROV2 6-DOF)";
        case ROVLayout::HEAVY_8:    return "AUV / Heavy ROV (8-Thruster Decoupled 6-DOF)";
        case ROVLayout::CUSTOM_N:
        default:                    return "ROV Submarine (Custom N-Thrusters)";
    }
}

const char* ROVVehicle::get_thruster_layout_name() const {
    if (_matrix_mixer) {
        return _matrix_mixer->get_preset_name();
    }
    return "Unknown Layout";
}

bool ROVVehicle::set_thruster_count(uint8_t count, ROVLayout layout) {
    if (count < 2 || count > hal::PWMOutput::MAX_CHANNELS || !_matrix_mixer) {
        return false;
    }
    _thruster_count = count;
    _layout = layout;

    if (layout == ROVLayout::AUTO) {
        if (count == 3) _layout = ROVLayout::SIMPLE_3;
        else if (count == 4) _layout = ROVLayout::BASIC_4;
        else if (count == 6) _layout = ROVLayout::VECTORED_6;
        else if (count == 8) _layout = ROVLayout::HEAVY_8;
        else _layout = ROVLayout::CUSTOM_N;
        return _matrix_mixer->configure_rov(count);
    }

    switch (layout) {
        case ROVLayout::SIMPLE_3:
            _thruster_count = 3;
            return _matrix_mixer->load_preset(control::MixerPreset::ROV_3_SIMPLE);
        case ROVLayout::BASIC_4:
            _thruster_count = 4;
            return _matrix_mixer->load_preset(control::MixerPreset::ROV_4_BASIC);
        case ROVLayout::VECTORED_6:
            _thruster_count = 6;
            return _matrix_mixer->load_preset(control::MixerPreset::ROV_6_VECTORED);
        case ROVLayout::HEAVY_8:
            _thruster_count = 8;
            return _matrix_mixer->load_preset(control::MixerPreset::ROV_8_HEAVY);
        case ROVLayout::CUSTOM_N:
        default:
            return _matrix_mixer->configure_rov(count);
    }
}

bool ROVVehicle::set_custom_thruster(uint8_t channel, float roll, float pitch, float yaw, float heave, float surge, float sway) {
    if (!_matrix_mixer) return false;
    control::ActuatorConfig cfg{roll, pitch, yaw, heave, surge, sway, true, false, 1000, 2000, 1500, true};
    _layout = ROVLayout::CUSTOM_N;
    return _matrix_mixer->set_actuator_config(channel, cfg);
}

void ROVVehicle::setup_pids() {
    // Attitude stabilization (Roll & Pitch self-leveling)
    _roll_pid.set_gains(2.0f, 0.1f, 0.05f);
    _roll_pid.set_limits(-0.6f, 0.6f, 0.2f);

    _pitch_pid.set_gains(2.0f, 0.1f, 0.05f);
    _pitch_pid.set_limits(-0.6f, 0.6f, 0.2f);

    _yaw_pid.set_gains(1.5f, 0.05f, 0.05f);
    _yaw_pid.set_limits(-0.6f, 0.6f, 0.2f);

    // Depth hold
    _depth_pid.set_gains(1.2f, 0.0f, 0.0f);
    _depth_pid.set_limits(-1.0f, 1.0f, 0.0f); // Max 1 m/s climb/sink rate

    _depth_rate_pid.set_gains(0.4f, 0.15f, 0.02f);
    _depth_rate_pid.set_limits(-0.7f, 0.7f, 0.3f);
}

void ROVVehicle::update_controller(float dt) {
    const auto& state = get_state();

    if (_mode == ROVMode::MANUAL) {
        _demands = _user_input;
        return;
    }

    _demands = _user_input;

    // Stabilize attitude (keep ROV level in water when user input is neutral)
    if (_mode == ROVMode::STABILIZE || _mode == ROVMode::DEPTH_HOLD) {
        // Roll & Pitch self leveling toward 0 or user target angle
        if (_thruster_count >= 4) {
            float target_roll = _user_input.roll * 0.5f;
            _demands.roll = _user_input.roll + _roll_pid.update(target_roll, state.euler_rad.x, dt);
        }
        if (_thruster_count >= 6) {
            float target_pitch = _user_input.pitch * 0.5f;
            _demands.pitch = _user_input.pitch + _pitch_pid.update(target_pitch, state.euler_rad.y, dt);
        }
    }

    // Depth Hold (ArduSub style)
    if (_mode == ROVMode::DEPTH_HOLD && state.altitude_valid) {
        // In water, depth = -altitude_m (or depth sensor)
        float current_depth = -state.altitude_m;
        float desired_sink_rate = _depth_pid.update(_target_depth_m, current_depth, dt);
        float current_sink_rate = -state.vertical_velocity_m_s;
        _demands.throttle = -_depth_rate_pid.update(desired_sink_rate, current_sink_rate, dt);
    }
}

} // namespace vehicles
} // namespace autopilot
