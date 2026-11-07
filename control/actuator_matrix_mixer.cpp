#include "actuator_matrix_mixer.hpp"
#include <cstring>
#include <cmath>
#include "drivers/can/dronecan_codec.hpp"
#include <algorithm>

namespace autopilot {
namespace control {

ActuatorMatrixMixer::ActuatorMatrixMixer() {
    load_preset(MixerPreset::ROV_6_VECTORED);
}

ActuatorMatrixMixer::ActuatorMatrixMixer(MixerPreset preset) {
    load_preset(preset);
}

const char* ActuatorMatrixMixer::get_preset_name() const {
    switch (_current_preset) {
        case MixerPreset::ROV_3_SIMPLE:   return "ROV 3-Thruster Simple (2 Horiz + 1 Vert)";
        case MixerPreset::ROV_4_BASIC:    return "ROV 4-Thruster Basic (2 Horiz + 2 Vert)";
        case MixerPreset::ROV_6_VECTORED: return "ROV 6-Thruster Vectored (BlueROV2 6-DOF)";
        case MixerPreset::ROV_8_HEAVY:    return "ROV 8-Thruster Heavy (Decoupled 6-DOF AUV)";
        case MixerPreset::UAV_TRI:        return "UAV 3-Rotor Tricopter";
        case MixerPreset::UAV_QUAD_X:     return "UAV 4-Rotor Quad-X";
        case MixerPreset::UAV_HEXA_X:     return "UAV 6-Rotor Hexa-X";
        case MixerPreset::UAV_OCTA_X:     return "UAV 8-Rotor Octa-X";
        case MixerPreset::UAV_DODECA_12:  return "UAV 12-Rotor Heavy-Lift";
        case MixerPreset::UAV_HEXADEC_16: return "UAV 16-Rotor Redundant Heavy-Lift";
        case MixerPreset::CUSTOM:
        default:                          return "Custom Actuator Allocation Matrix";
    }
}

bool ActuatorMatrixMixer::load_preset(MixerPreset preset) {
    _current_preset = preset;
    // Reset all actuators
    for (size_t i = 0; i < hal::PWMOutput::MAX_CHANNELS; ++i) {
        _actuators[i] = ActuatorConfig{};
        _actuators[i].enabled = false;
    }

    switch (preset) {
        case MixerPreset::ROV_3_SIMPLE:
            setup_rov_3_simple();
            break;
        case MixerPreset::ROV_4_BASIC:
            setup_rov_4_basic();
            break;
        case MixerPreset::ROV_6_VECTORED:
            setup_rov_6_vectored();
            break;
        case MixerPreset::ROV_8_HEAVY:
            setup_rov_8_heavy();
            break;
        case MixerPreset::UAV_TRI:
            setup_uav_tri();
            break;
        case MixerPreset::UAV_QUAD_X:
            setup_uav_quad_x();
            break;
        case MixerPreset::UAV_HEXA_X:
            setup_uav_hexa_x();
            break;
        case MixerPreset::UAV_OCTA_X:
            setup_uav_octa_x();
            break;
        case MixerPreset::UAV_DODECA_12:
            setup_uav_n_symmetric(12);
            break;
        case MixerPreset::UAV_HEXADEC_16:
            setup_uav_n_symmetric(16);
            break;
        case MixerPreset::CUSTOM:
        default:
            _num_channels = 0;
            break;
    }
    return true;
}

bool ActuatorMatrixMixer::configure_rov(uint8_t num_thrusters) {
    if (num_thrusters < 2 || num_thrusters > hal::PWMOutput::MAX_CHANNELS) {
        return false;
    }

    switch (num_thrusters) {
        case 3:
            return load_preset(MixerPreset::ROV_3_SIMPLE);
        case 4:
            return load_preset(MixerPreset::ROV_4_BASIC);
        case 6:
            return load_preset(MixerPreset::ROV_6_VECTORED);
        case 8:
            return load_preset(MixerPreset::ROV_8_HEAVY);
        default:
            _current_preset = MixerPreset::CUSTOM;
            setup_rov_n_symmetric(num_thrusters);
            return true;
    }
}

bool ActuatorMatrixMixer::configure_uav(uint8_t num_rotors) {
    if (num_rotors < 3 || num_rotors > hal::PWMOutput::MAX_CHANNELS) {
        return false;
    }

    switch (num_rotors) {
        case 3:
            return load_preset(MixerPreset::UAV_TRI);
        case 4:
            return load_preset(MixerPreset::UAV_QUAD_X);
        case 6:
            return load_preset(MixerPreset::UAV_HEXA_X);
        case 8:
            return load_preset(MixerPreset::UAV_OCTA_X);
        case 12:
            return load_preset(MixerPreset::UAV_DODECA_12);
        case 16:
            return load_preset(MixerPreset::UAV_HEXADEC_16);
        default:
            _current_preset = MixerPreset::CUSTOM;
            setup_uav_n_symmetric(num_rotors);
            return true;
    }
}

// ---------------- Preset Setup Implementations ----------------

void ActuatorMatrixMixer::setup_rov_3_simple() {
    _num_channels = 3;
    // Thruster 1: Horizontal Port
    _actuators[0] = {0.0f, 0.0f, 0.7f, 0.0f, 1.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // Thruster 2: Horizontal Starboard
    _actuators[1] = {0.0f, 0.0f, -0.7f, 0.0f, 1.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // Thruster 3: Vertical Center
    _actuators[2] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
}

void ActuatorMatrixMixer::setup_rov_4_basic() {
    _num_channels = 4;
    // Thruster 1: Horizontal Port
    _actuators[0] = {0.0f, 0.0f, 0.7f, 0.0f, 1.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // Thruster 2: Horizontal Starboard
    _actuators[1] = {0.0f, 0.0f, -0.7f, 0.0f, 1.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // Thruster 3: Vertical Port (Heave + Roll)
    _actuators[2] = {-1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // Thruster 4: Vertical Starboard (Heave + Roll)
    _actuators[3] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
}

void ActuatorMatrixMixer::setup_rov_6_vectored() {
    _num_channels = 6;
    // ArduSub Vectored 6-DOF BlueROV2 Standard Layout:
    // T1: Forward-Right (Vectored 45 deg)
    _actuators[0] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f, true, false, 1000, 2000, 1500, true};
    // T2: Forward-Left  (Vectored 45 deg)
    _actuators[1] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f, true, false, 1000, 2000, 1500, true};
    // T3: Rear-Right     (Vectored 45 deg)
    _actuators[2] = {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, -1.0f, true, false, 1000, 2000, 1500, true};
    // T4: Rear-Left      (Vectored 45 deg)
    _actuators[3] = {0.0f, 0.0f, -1.0f, 0.0f, -1.0f, 1.0f, true, false, 1000, 2000, 1500, true};
    // T5: Vertical-Right
    _actuators[4] = {-1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // T6: Vertical-Left
    _actuators[5] = {1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
}

void ActuatorMatrixMixer::setup_rov_8_heavy() {
    _num_channels = 8;
    // BlueROV2 Heavy & RoboSub AUV 8-Thruster Decoupled 6-DOF Configuration:
    // 4 Horizontal Vectored Thrusters (Surge, Sway, Yaw):
    _actuators[0] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f, true, false, 1000, 2000, 1500, true}; // Fwd-Right
    _actuators[1] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f, true, false, 1000, 2000, 1500, true}; // Fwd-Left
    _actuators[2] = {0.0f, 0.0f, 1.0f, 0.0f, -1.0f, -1.0f, true, false, 1000, 2000, 1500, true}; // Rear-Right
    _actuators[3] = {0.0f, 0.0f, -1.0f, 0.0f, -1.0f, 1.0f, true, false, 1000, 2000, 1500, true}; // Rear-Left

    // 4 Vertical Thrusters (Full Decoupled Pitch, Roll, and Heave Control!):
    // T5: Forward-Right Vertical (+Heave, -Roll, +Pitch)
    _actuators[4] = {-1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // T6: Forward-Left Vertical (+Heave, +Roll, +Pitch)
    _actuators[5] = {1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // T7: Rear-Right Vertical (+Heave, -Roll, -Pitch)
    _actuators[6] = {-1.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // T8: Rear-Left Vertical (+Heave, +Roll, -Pitch)
    _actuators[7] = {1.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
}

void ActuatorMatrixMixer::setup_rov_n_symmetric(uint8_t n) {
    _num_channels = std::min<uint8_t>(n, hal::PWMOutput::MAX_CHANNELS);

    if (_num_channels <= 2) {
        _num_channels = 2;
        _actuators[0] = {0.0f, 0.0f, 0.7f, 0.0f, 1.0f, 0.0f, true, false, 1000, 2000, 1500, true};
        _actuators[1] = {0.0f, 0.0f, -0.7f, 0.0f, 1.0f, 0.0f, true, false, 1000, 2000, 1500, true};
        return;
    }

    // Allocate vertical thrusters (heave/roll/pitch) and horizontal thrusters (surge/sway/yaw)
    uint8_t n_vert = (_num_channels >= 8) ? 4 : ((_num_channels >= 4) ? 2 : 1);
    uint8_t n_horiz = _num_channels - n_vert;

    // Configure Horizontal Vectored Thrusters
    for (uint8_t i = 0; i < n_horiz; ++i) {
        float angle = (2.0f * 3.14159265f * i) / n_horiz + (3.14159265f / n_horiz);
        float surge = std::cos(angle);
        float sway = -std::sin(angle);
        float yaw = (i % 2 == 0) ? 1.0f : -1.0f;
        _actuators[i] = {0.0f, 0.0f, yaw, 0.0f, surge, sway, true, false, 1000, 2000, 1500, true};
    }

    // Configure Vertical Thrusters
    if (n_vert == 1) {
        _actuators[n_horiz] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    } else if (n_vert == 2) {
        _actuators[n_horiz] = {-1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
        _actuators[n_horiz + 1] = {1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    } else { // 4 vertical thrusters
        _actuators[n_horiz]     = {-1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true}; // FR
        _actuators[n_horiz + 1] = {1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};  // FL
        _actuators[n_horiz + 2] = {-1.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true}; // RR
        _actuators[n_horiz + 3] = {1.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};  // RL
    }
}

void ActuatorMatrixMixer::setup_uav_tri() {
    _num_channels = 3;
    _actuators[0] = {-1.0f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true}; // Left
    _actuators[1] = {1.0f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true};  // Right
    _actuators[2] = {0.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true}; // Tail
}

void ActuatorMatrixMixer::setup_uav_quad_x() {
    _num_channels = 4;
    // Standard Quad-X mixing matrix
    _actuators[0] = {-1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true};  // M1 Front-Right (CCW)
    _actuators[1] = {1.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true};  // M2 Rear-Left   (CCW)
    _actuators[2] = {1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true};  // M3 Front-Left  (CW)
    _actuators[3] = {-1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true}; // M4 Rear-Right  (CW)
}

void ActuatorMatrixMixer::setup_uav_hexa_x() {
    setup_uav_n_symmetric(6);
}

void ActuatorMatrixMixer::setup_uav_octa_x() {
    setup_uav_n_symmetric(8);
}

void ActuatorMatrixMixer::setup_uav_n_symmetric(uint8_t n) {
    _num_channels = std::min<uint8_t>(n, hal::PWMOutput::MAX_CHANNELS);
    for (uint8_t i = 0; i < _num_channels; ++i) {
        float angle = (2.0f * 3.14159265f * i) / _num_channels + (3.14159265f / _num_channels);
        float roll = -std::sin(angle);
        float pitch = std::cos(angle);
        float yaw = (i % 2 == 0) ? 1.0f : -1.0f;
        _actuators[i] = {roll, pitch, yaw, 1.0f, 0.0f, 0.0f, false, false, 1000, 2000, 1000, true};
    }
}

// ---------------- Custom Actuator Configuration ----------------

bool ActuatorMatrixMixer::set_actuator_config(uint8_t channel, const ActuatorConfig& config) {
    if (channel >= hal::PWMOutput::MAX_CHANNELS) {
        return false;
    }
    _actuators[channel] = config;
    if (channel >= _num_channels && config.enabled) {
        _num_channels = channel + 1;
    }
    _current_preset = MixerPreset::CUSTOM;
    return true;
}

const ActuatorConfig& ActuatorMatrixMixer::get_actuator_config(uint8_t channel) const {
    static ActuatorConfig dummy{};
    if (channel < hal::PWMOutput::MAX_CHANNELS) {
        return _actuators[channel];
    }
    return dummy;
}

ActuatorConfig& ActuatorMatrixMixer::get_actuator_config_mut(uint8_t channel) {
    static ActuatorConfig dummy{};
    if (channel < hal::PWMOutput::MAX_CHANNELS) {
        return _actuators[channel];
    }
    return dummy;
}

void ActuatorMatrixMixer::set_num_channels(uint8_t count) {
    _num_channels = std::min<uint8_t>(count, hal::PWMOutput::MAX_CHANNELS);
    for (uint8_t i = 0; i < hal::PWMOutput::MAX_CHANNELS; ++i) {
        _actuators[i].enabled = (i < _num_channels);
    }
}

// ---------------- Output & Mixing Calculation ----------------

void ActuatorMatrixMixer::get_normalized_outputs(const ControlDemands& demands, float* out_values, size_t max_len) const {
    if (!out_values || max_len == 0) return;

    size_t count = std::min<size_t>(_num_channels, max_len);

    float max_abs_bidirectional = 1.0f;
    float max_unidirectional = 1.0f;
    float min_unidirectional = 0.0f;

    // Step 1: Compute raw unconstrained demands
    for (size_t i = 0; i < count; ++i) {
        if (!_actuators[i].enabled) {
            out_values[i] = 0.0f;
            continue;
        }

        const auto& act = _actuators[i];
        float raw = (demands.roll * act.roll) +
                    (demands.pitch * act.pitch) +
                    (demands.yaw * act.yaw) +
                    (demands.throttle * act.heave) +
                    (demands.forward * act.surge) +
                    (demands.lateral * act.sway);

        if (act.reversed) {
            raw = -raw;
        }
        out_values[i] = raw;

        if (act.bidirectional) {
            max_abs_bidirectional = std::max(max_abs_bidirectional, std::abs(raw));
        } else {
            max_unidirectional = std::max(max_unidirectional, raw);
            min_unidirectional = std::min(min_unidirectional, raw);
        }
    }

    // Step 2: Desaturation and Dynamic Scaling
    // Bidirectional (ROV): scale proportionally to prevent saturation clipping and preserve thrust vector angles
    if (max_abs_bidirectional > 1.0f) {
        float scale = 1.0f / max_abs_bidirectional;
        for (size_t i = 0; i < count; ++i) {
            if (_actuators[i].bidirectional && _actuators[i].enabled) {
                out_values[i] *= scale;
            }
        }
    }

    // Unidirectional (UAV): prioritize roll/pitch/yaw stabilization authority over pure throttle
    if (max_unidirectional > 1.0f) {
        float throttle_backoff = max_unidirectional - 1.0f;
        for (size_t i = 0; i < count; ++i) {
            if (!_actuators[i].bidirectional && _actuators[i].enabled) {
                out_values[i] -= throttle_backoff;
            }
        }
    }
    if (min_unidirectional < 0.0f) {
        for (size_t i = 0; i < count; ++i) {
            if (!_actuators[i].bidirectional && _actuators[i].enabled) {
                out_values[i] -= min_unidirectional;
            }
        }
    }

    // Final safety bounds clamping
    for (size_t i = 0; i < count; ++i) {
        if (!_actuators[i].enabled) {
            out_values[i] = 0.0f;
        } else if (_actuators[i].bidirectional) {
            out_values[i] = std::clamp(out_values[i], -1.0f, 1.0f);
        } else {
            out_values[i] = std::clamp(out_values[i], 0.0f, 1.0f);
        }
    }
}

void ActuatorMatrixMixer::output_to_actuators(const ControlDemands& demands, hal::PWMOutput* pwm, bool armed) {
    if (!pwm) return;

    if (!armed) {
        // Output safe neutral / disarmed PWM pulses
        for (uint8_t i = 0; i < hal::PWMOutput::MAX_CHANNELS; ++i) {
            if (i < _num_channels && _actuators[i].enabled) {
                pwm->set_channel(i, _actuators[i].neutral_pwm);
            } else {
                pwm->set_channel(i, 1000); // Disarmed safe default
            }
        }
        return;
    }

    float normalized[hal::PWMOutput::MAX_CHANNELS]{0.0f};
    get_normalized_outputs(demands, normalized, _num_channels);

    for (uint8_t i = 0; i < hal::PWMOutput::MAX_CHANNELS; ++i) {
        if (i < _num_channels && _actuators[i].enabled) {
            const auto& act = _actuators[i];
            uint16_t pulse = act.neutral_pwm;

            if (act.bidirectional) {
                // Map [-1.0, 1.0] to [min_pwm, max_pwm] with center at neutral_pwm
                float val = normalized[i];
                if (val >= 0.0f) {
                    pulse = static_cast<uint16_t>(act.neutral_pwm + (val * (act.max_pwm - act.neutral_pwm)));
                } else {
                    pulse = static_cast<uint16_t>(act.neutral_pwm + (val * (act.neutral_pwm - act.min_pwm)));
                }
            } else {
                // Map [0.0, 1.0] to [min_pwm, max_pwm]
                pulse = static_cast<uint16_t>(act.min_pwm + (normalized[i] * (act.max_pwm - act.min_pwm)));
            }

            pulse = std::clamp(pulse, act.min_pwm, act.max_pwm);
            pwm->set_channel(i, pulse);
        } else {
            pwm->set_channel(i, 1000);
        }
    }
}

// ---------------- DroneCAN ESC Output Integration ----------------

bool ActuatorMatrixMixer::generate_dronecan_esc_frame(const ControlDemands& demands, uint8_t& transfer_id, hal::CANFrame& out_frame, bool armed) const {
    // uavcan.equipment.esc.RawCommand (1030): int14[<=20] cmd, range [-8192, 8191].
    // Serialised with the DroneCAN bit order (see drivers/can/dronecan_codec.hpp).
    // Up to 4 ESCs fit in one frame (4 x 14 bit = 7 bytes + tail byte).
    constexpr uint8_t PRIORITY = 20;      // high priority for actuator commands
    constexpr uint8_t SOURCE_NODE_ID = 1; // autopilot node id

    float normalized[hal::PWMOutput::MAX_CHANNELS]{0.0f};
    if (armed) {
        get_normalized_outputs(demands, normalized, _num_channels);
    }

    int16_t esc_cmd[4]{0, 0, 0, 0};
    const uint8_t count = std::min<uint8_t>(_num_channels, 4);
    for (uint8_t i = 0; i < count; ++i) {
        if (_actuators[i].enabled && armed) {
            const float v = std::clamp(normalized[i], -1.0f, 1.0f);
            esc_cmd[i] = static_cast<int16_t>(std::lround(v * 8191.0f));
        }
    }

    uint8_t payload[8];
    const size_t len = drivers::dronecan::encode_esc_raw_command(esc_cmd, count, payload, sizeof(payload));
    if (len == 0 || len > 7) return false;
    if (drivers::dronecan::make_broadcast_frames(PRIORITY, drivers::dronecan::MSG_ID_ESC_RAW_COMMAND, SOURCE_NODE_ID,
                                                 transfer_id, payload, len, &out_frame, 1) != 1) {
        return false;
    }
    transfer_id = (transfer_id + 1) & 0x1F;
    return true;
}

bool ActuatorMatrixMixer::send_dronecan_esc_commands(const ControlDemands& demands, hal::CANDevice* can, uint8_t& transfer_id, bool armed) const {
    if (!can) return false;
    hal::CANFrame frame;
    if (generate_dronecan_esc_frame(demands, transfer_id, frame, armed)) {
        return can->send_frame(frame);
    }
    return false;
}

} // namespace control
} // namespace autopilot

