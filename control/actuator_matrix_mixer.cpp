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
    reset_failures();
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
    reset_failures();

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
    reset_failures();

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
    // Thruster 3: Vertical Port (Heave + Roll). Port pushing up rolls right-side-down,
    // which is +roll in FRD (the sign was inverted before, so STABILIZE fought itself).
    _actuators[2] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    // Thruster 4: Vertical Starboard (Heave + Roll)
    _actuators[3] = {-1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
}

void ActuatorMatrixMixer::setup_rov_6_vectored() {
    _num_channels = 6;
    // Vectored 6-DOF layout (BlueROV2-style), rows = {roll, pitch, yaw, heave, surge, sway}.
    // Horizontal thrusters sit at the four corners, angled 45 deg; positive output pushes
    // along these FRD directions:
    //   T1 front-right (+x,-y)   T2 front-left (+x,+y)   T3 rear-right (+x,+y)   T4 rear-left (+x,-y)
    // Each row is that thruster's real force/torque contribution, so the surge, sway and
    // yaw columns are orthogonal: a pure yaw demand produces pure yaw (no sideways drift).
    // (The previous rows had T3 = -T2 and T4 = -T1, rank 2: yaw also pushed the vehicle
    // sideways. sim/sub_sim.hpp models the geometry and test_sitl_vehicles checks it.)
    _actuators[0] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, -1.0f, true, false, 1000, 2000, 1500, true};
    _actuators[1] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, true, false, 1000, 2000, 1500, true};
    _actuators[2] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f, true, false, 1000, 2000, 1500, true};
    _actuators[3] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f, true, false, 1000, 2000, 1500, true};
    // T5/T6: vertical thrusters on the right/left side, level with the centre of mass:
    // heave and roll only. They have no pitch authority (a 6-thruster BlueROV2 can't pitch).
    _actuators[4] = {-1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
    _actuators[5] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
}

void ActuatorMatrixMixer::setup_rov_8_heavy() {
    _num_channels = 8;
    // BlueROV2 Heavy & RoboSub AUV 8-Thruster Decoupled 6-DOF Configuration:
    // 4 Horizontal Vectored Thrusters (Surge, Sway, Yaw):
    // (same geometry as the 6-thruster vectored frame, see setup_rov_6_vectored)
    _actuators[0] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, -1.0f, true, false, 1000, 2000, 1500, true}; // Fwd-Right
    _actuators[1] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, true, false, 1000, 2000, 1500, true};   // Fwd-Left
    _actuators[2] = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f, true, false, 1000, 2000, 1500, true};  // Rear-Right
    _actuators[3] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f, true, false, 1000, 2000, 1500, true};  // Rear-Left

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
        // right / left side, level with the centre of mass: heave + roll, no pitch authority
        _actuators[n_horiz] = {-1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
        _actuators[n_horiz + 1] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true, false, 1000, 2000, 1500, true};
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
    clear_motor_failures();   // edit the healthy geometry, not a re-allocated one
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
    clear_motor_failures();
    _num_channels = std::min<uint8_t>(count, hal::PWMOutput::MAX_CHANNELS);
    for (uint8_t i = 0; i < hal::PWMOutput::MAX_CHANNELS; ++i) {
        _actuators[i].enabled = (i < _num_channels);
    }
}

// ---------------- Output & Mixing Calculation ----------------

void ActuatorMatrixMixer::get_normalized_outputs(const ControlDemands& demands_in, float* out_values, size_t max_len) const {
    if (!out_values || max_len == 0) return;

    const size_t count = std::min<size_t>(_num_channels, max_len);

    // A NaN/Inf demand (e.g. from a diverged estimator) must never reach an ESC:
    // treat the whole demand vector as "neutral".
    ControlDemands demands = demands_in;
    if (!std::isfinite(demands.roll) || !std::isfinite(demands.pitch) || !std::isfinite(demands.yaw) ||
        !std::isfinite(demands.throttle) || !std::isfinite(demands.forward) || !std::isfinite(demands.lateral)) {
        demands = ControlDemands{};
    }

    // Split every unidirectional output into a roll/pitch(/surge/sway) part, a yaw part and
    // a collective part so desaturation can trade them off in priority order:
    //   1. roll/pitch  (keeps the vehicle upright)
    //   2. yaw
    //   3. collective  (shifted last, "air mode" style)
    float rp[hal::PWMOutput::MAX_CHANNELS]{};
    float yw[hal::PWMOutput::MAX_CHANNELS]{};
    float max_abs_bidirectional = 1.0f;
    bool any_uni = false;

    for (size_t i = 0; i < count; ++i) {
        const auto& act = _actuators[i];
        if (!act.enabled) {
            out_values[i] = 0.0f;
            continue;
        }
        const float sign = act.reversed ? -1.0f : 1.0f;
        if (act.bidirectional) {
            const float raw = sign * ((demands.roll * act.roll) + (demands.pitch * act.pitch) + (demands.yaw * act.yaw) +
                                      (demands.throttle * act.heave) + (demands.forward * act.surge) +
                                      (demands.lateral * act.sway));
            out_values[i] = raw;
            max_abs_bidirectional = std::max(max_abs_bidirectional, std::fabs(raw));
        } else {
            any_uni = true;
            rp[i] = sign * ((demands.roll * act.roll) + (demands.pitch * act.pitch) + (demands.forward * act.surge) +
                            (demands.lateral * act.sway));
            yw[i] = sign * demands.yaw * act.yaw;
            out_values[i] = sign * demands.throttle * act.heave;  // collective part for now
        }
    }

    // Bidirectional (ROV): scale proportionally, preserving the thrust vector direction.
    if (max_abs_bidirectional > 1.0f) {
        const float scale = 1.0f / max_abs_bidirectional;
        for (size_t i = 0; i < count; ++i) {
            if (_actuators[i].bidirectional && _actuators[i].enabled) out_values[i] *= scale;
        }
    }

    if (any_uni) {
        // Desaturation in priority order, for any collective direction h_i (all 1
        // on a healthy frame; uneven after a motor failure re-allocation, where a
        // uniform shift would itself produce torque):
        //   out_i = T * h_i + k_rp * rp_i + k_y * yw_i,   0 <= out_i <= 1
        // 1. the largest k_rp <= 1 for which some collective T fits every motor,
        // 2. then the largest k_y <= 1 on top of that,
        // 3. then T as close to the demanded collective as the motors allow.
        // On a healthy frame step 1/2 reduce to "the differential spread must fit
        // in [0, 1]" and step 3 to the usual collective shift ("air mode").
        // Motors with (almost) no collective share (the motor opposite a failed
        // one) can't be helped by any T: they are left out and simply clipped.
        float h[hal::PWMOutput::MAX_CHANNELS]{};
        float h_max = 0.0f;
        for (size_t i = 0; i < count; ++i) {
            const auto& act = _actuators[i];
            if (act.bidirectional || !act.enabled) continue;
            h[i] = (act.reversed ? -1.0f : 1.0f) * act.heave;
            h_max = std::max(h_max, h[i]);
        }
        const float h_min_driving = 0.05f * h_max;
        auto driving = [&](size_t i) {
            return !_actuators[i].bidirectional && _actuators[i].enabled && h[i] > h_min_driving;
        };

        // Largest k in [0, 1] such that a + k*w fits: for every pair (i, j)
        //   (-a_i - k w_i) / h_i <= (1 - a_j - k w_j) / h_j
        auto max_k = [&](const float* a, const float* w) {
            float k = 1.0f;
            for (size_t i = 0; i < count; ++i) {
                if (!driving(i)) continue;
                for (size_t j = 0; j < count; ++j) {
                    if (j == i || !driving(j)) continue;
                    const float b = w[j] / h[j] - w[i] / h[i];
                    if (b <= 1e-9f) continue;
                    const float c = 1.0f / h[j] + a[i] / h[i] - a[j] / h[j];
                    k = std::min(k, std::max(0.0f, c) / b);
                }
            }
            return k;
        };

        // Collective range in which every driving motor fits a + k*w (lo > hi: none).
        auto t_range = [&](const float* v, float& lo, float& hi) {
            lo = -1e9f;
            hi = 1e9f;
            for (size_t i = 0; i < count; ++i) {
                if (!driving(i)) continue;
                lo = std::max(lo, -v[i] / h[i]);
                hi = std::min(hi, (1.0f - v[i]) / h[i]);
            }
        };
        auto fit = [](float t, float lo, float hi) {
            if (lo <= hi) return std::clamp(t, lo, hi);
            return lo > -1e8f ? 0.5f * (lo + hi) : t;
        };

        float zero[hal::PWMOutput::MAX_CHANNELS]{};
        const float k_rp = max_k(zero, rp);
        float a_rp[hal::PWMOutput::MAX_CHANNELS]{};
        for (size_t i = 0; i < count; ++i) a_rp[i] = k_rp * rp[i];
        float lo, hi;
        t_range(a_rp, lo, hi);
        const float t_rp = fit(demands.throttle, lo, hi);   // collective after roll/pitch

        // Yaw may move that collective only a little: otherwise a yaw demand the
        // frame can't meet (yaw integrator wound up after a motor failure) would
        // drag the collective along and climb or sink the vehicle.
        constexpr float YAW_COLLECTIVE_BAND = 0.1f;
        float k_y = max_k(a_rp, yw);
        for (size_t i = 0; i < count; ++i) {
            if (!driving(i)) continue;
            if (yw[i] < 0.0f) {
                k_y = std::min(k_y, std::max(0.0f, a_rp[i] + (t_rp + YAW_COLLECTIVE_BAND) * h[i]) / -yw[i]);
            } else if (yw[i] > 0.0f) {
                k_y = std::min(k_y, std::max(0.0f, 1.0f - (t_rp - YAW_COLLECTIVE_BAND) * h[i] - a_rp[i]) / yw[i]);
            }
        }

        float v[hal::PWMOutput::MAX_CHANNELS]{};
        for (size_t i = 0; i < count; ++i) v[i] = a_rp[i] + k_y * yw[i];
        t_range(v, lo, hi);
        const float t = fit(t_rp, lo, hi);
        for (size_t i = 0; i < count; ++i) {
            if (_actuators[i].bidirectional || !_actuators[i].enabled) continue;
            out_values[i] = t * h[i] + v[i];
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
        for (float& o : _last_outputs) o = 0.0f;
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
    for (uint8_t i = 0; i < hal::PWMOutput::MAX_CHANNELS; ++i) _last_outputs[i] = normalized[i];

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

// ---------------- Motor failure: control re-allocation ----------------

const char* allocation_status_name(AllocationStatus s) {
    switch (s) {
        case AllocationStatus::NOMINAL:        return "nominal";
        case AllocationStatus::DEGRADED:       return "degraded";
        case AllocationStatus::NO_YAW:         return "no yaw";
        case AllocationStatus::UNCONTROLLABLE:
        default:                               return "uncontrollable";
    }
}

namespace {
/** In-place inverse of a small SPD-ish matrix (n <= 4), Gauss-Jordan with partial
 *  pivoting. False when a pivot is tiny relative to the matrix scale (singular). */
bool invert_small(float m[4][4], int n, float inv[4][4]) {
    float scale = 0.0f;
    for (int i = 0; i < n; ++i) scale = std::max(scale, std::fabs(m[i][i]));
    if (scale <= 0.0f) return false;
    float a[4][8]{};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) a[i][j] = m[i][j];
        a[i][n + i] = 1.0f;
    }
    for (int c = 0; c < n; ++c) {
        int p = c;
        for (int r = c + 1; r < n; ++r) if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-4f * scale) return false;
        if (p != c) for (int j = 0; j < 2 * n; ++j) std::swap(a[p][j], a[c][j]);
        const float d = a[c][c];
        for (int j = 0; j < 2 * n; ++j) a[c][j] /= d;
        for (int r = 0; r < n; ++r) {
            if (r == c) continue;
            const float f = a[r][c];
            if (f == 0.0f) continue;
            for (int j = 0; j < 2 * n; ++j) a[r][j] -= f * a[c][j];
        }
    }
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) inv[i][j] = a[i][n + j];
    return true;
}
} // namespace

void ActuatorMatrixMixer::reset_failures() {
    _failed_mask = 0;
    _alloc_status = AllocationStatus::NOMINAL;
    _collective_limit = 1.0f;
}

void ActuatorMatrixMixer::clear_motor_failures() {
    if (_failed_mask == 0) return;
    _actuators = _nominal;
    reset_failures();
}

const ActuatorConfig& ActuatorMatrixMixer::get_nominal_config(uint8_t channel) const {
    if (_failed_mask == 0) return get_actuator_config(channel);
    static ActuatorConfig dummy{};
    return channel < hal::PWMOutput::MAX_CHANNELS ? _nominal[channel] : dummy;
}

bool ActuatorMatrixMixer::set_motor_failed(uint8_t channel, bool failed) {
    if (channel >= _num_channels) return false;
    const ActuatorConfig& nom = get_nominal_config(channel);
    if (nom.bidirectional || !nom.enabled) return false;
    const uint16_t bit = static_cast<uint16_t>(1u << channel);
    const uint16_t mask = failed ? (_failed_mask | bit) : (_failed_mask & ~bit);
    if (mask == _failed_mask) return true;
    if (_failed_mask == 0) _nominal = _actuators;   // first failure: keep the healthy rows
    if (mask == 0) {
        clear_motor_failures();
        return true;
    }
    _failed_mask = mask;
    reallocate();
    return true;
}

void ActuatorMatrixMixer::reallocate() {
    // Effectiveness of motor i on (roll, pitch, yaw, collective) = its healthy row.
    auto row = [&](size_t i, float r[4]) {
        const auto& a = _nominal[i];
        r[0] = a.roll; r[1] = a.pitch; r[2] = a.yaw; r[3] = a.heave;
    };
    auto usable = [&](size_t i) { return _nominal[i].enabled && !_nominal[i].bidirectional; };

    // G = R^T R over the healthy frame; M = E E^T over the motors that are left.
    float G[4][4]{}, M[4][4]{};
    for (size_t i = 0; i < _num_channels; ++i) {
        if (!usable(i)) continue;
        float r[4];
        row(i, r);
        const bool alive = !(_failed_mask & (1u << i));
        for (int a = 0; a < 4; ++a) {
            for (int b = 0; b < 4; ++b) {
                G[a][b] += r[a] * r[b];
                if (alive) M[a][b] += r[a] * r[b];
            }
        }
    }

    // Roll/pitch/collective solve (axes 0, 1, 3): this sets the collective column.
    static const int ax3[3] = {0, 1, 3};
    float M3[4][4]{}, M3i[4][4]{}, X3[4][4]{};
    for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) M3[a][b] = M[ax3[a]][ax3[b]];
    const bool ok3 = invert_small(M3, 3, M3i);
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) sum += M3i[a][k] * G[ax3[k]][ax3[b]];
            X3[a][b] = sum;   // X3 = M3^-1 G3
        }

    // Full 4-axis solve: pure roll / pitch / yaw columns, if yaw is still controllable.
    float M4i[4][4]{}, X4[4][4]{};
    const bool ok4 = ok3 && invert_small(M, 4, M4i);
    if (ok4) {
        for (int a = 0; a < 4; ++a)
            for (int b = 0; b < 4; ++b) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) sum += M4i[a][k] * G[k][b];
                X4[a][b] = sum;
            }
    }

    // The collective always comes from the roll/pitch/collective solve. The 4-axis
    // solve would hover with zero yaw torque, but on a hexa it parks the motor
    // opposite the failed one at exactly 0, which leaves roll/pitch one-sided
    // (in SITL: a 9 deg tilt that can't be trimmed out). This way roll/pitch keep
    // two-sided authority and yaw takes what is left (a slow yaw drift).
    float h_max = 0.0f, h_min = 0.0f;
    for (size_t i = 0; i < _num_channels; ++i) {
        _actuators[i] = _nominal[i];
        if (!usable(i)) continue;
        if (_failed_mask & (1u << i)) {
            _actuators[i].enabled = false;   // a dead motor is commanded to idle
            continue;
        }
        if (!ok3) continue;
        float r[4];
        row(i, r);
        const float r3[3] = {r[0], r[1], r[3]};
        float c3[3];
        for (int b = 0; b < 3; ++b) c3[b] = r3[0] * X3[0][b] + r3[1] * X3[1][b] + r3[2] * X3[2][b];
        auto& a = _actuators[i];
        a.heave = c3[2];
        if (ok4) {
            float c4[4];
            for (int b = 0; b < 4; ++b) c4[b] = r[0] * X4[0][b] + r[1] * X4[1][b] + r[2] * X4[2][b] + r[3] * X4[3][b];
            a.roll = c4[0];
            a.pitch = c4[1];
            a.yaw = c4[2];
        } else {
            a.roll = c3[0];
            a.pitch = c3[1];
            a.yaw = 0.0f;
        }
        h_max = std::max(h_max, a.heave);
        h_min = std::min(h_min, a.heave);
    }

    if (!ok3 || h_max <= 0.0f || h_min < -0.05f * h_max) {
        _alloc_status = AllocationStatus::UNCONTROLLABLE;
    } else {
        _alloc_status = ok4 ? AllocationStatus::DEGRADED : AllocationStatus::NO_YAW;
    }
    _collective_limit = h_max > 0.0f ? std::min(1.0f, 1.0f / h_max) : 0.0f;
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
    // All channels (RawCommand carries up to 20). More than 4 ESCs need a multi-frame
    // transfer; the single-frame helper above only covers the first four.
    constexpr uint8_t PRIORITY = 20;
    constexpr uint8_t SOURCE_NODE_ID = 1;
    float normalized[hal::PWMOutput::MAX_CHANNELS]{0.0f};
    if (armed) get_normalized_outputs(demands, normalized, _num_channels);

    int16_t esc_cmd[hal::PWMOutput::MAX_CHANNELS]{};
    const uint8_t count = _num_channels;
    for (uint8_t i = 0; i < count; ++i) {
        if (_actuators[i].enabled && armed) {
            esc_cmd[i] = static_cast<int16_t>(std::lround(std::clamp(normalized[i], -1.0f, 1.0f) * 8191.0f));
        }
    }
    uint8_t payload[32];
    const size_t len = drivers::dronecan::encode_esc_raw_command(esc_cmd, count, payload, sizeof(payload));
    if (len == 0) return false;
    hal::CANFrame frames[6];
    const size_t n = drivers::dronecan::make_broadcast_frames(PRIORITY, drivers::dronecan::MSG_ID_ESC_RAW_COMMAND,
                                                              SOURCE_NODE_ID, transfer_id, payload, len, frames, 6);
    if (n == 0) return false;
    transfer_id = (transfer_id + 1) & 0x1F;
    bool ok = true;
    for (size_t i = 0; i < n; ++i) ok = can->send_frame(frames[i]) && ok;
    return ok;
}

} // namespace control
} // namespace autopilot

