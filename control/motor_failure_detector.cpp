#include "motor_failure_detector.hpp"
#include <algorithm>
#include <cmath>

namespace autopilot {
namespace control {

const char* motor_failure_cause_name(MotorFailureDetector::Cause c) {
    switch (c) {
        case MotorFailureDetector::Cause::LOST_THRUST:   return "lost thrust";
        case MotorFailureDetector::Cause::ESC_TELEMETRY: return "ESC telemetry";
        case MotorFailureDetector::Cause::NONE:
        default:                                         return "none";
    }
}

void MotorFailureDetector::reset() {
    _have_gyro = false;
    _alpha = math::Vector3f();
    for (uint8_t i = 0; i < N; ++i) {
        _u[i] = 0.0f;
        _u_lag[i] = 0.0f;
        _onset_s[i] = 0.0f;
        _sustained_s[i] = 0.0f;
        _esc_low_s[i] = 0.0f;
    }
}

void MotorFailureDetector::report_esc_rpm(uint8_t channel, float rpm, uint32_t now_ms) {
    if (channel >= N) return;
    _esc_rpm[channel] = rpm;
    _esc_ms[channel] = now_ms == 0 ? 1 : now_ms;
}

int MotorFailureDetector::update(const ActuatorMatrixMixer& mixer, const math::Vector3f& gyro, float collective,
                                 float dt, bool flying, uint32_t now_ms) {
    if (!(dt > 0.0f)) return -1;

    // Angular acceleration: differentiate the gyro, then low-pass.
    if (_have_gyro) {
        const float a = dt / (dt + 1.0f / (2.0f * 3.14159265f * _cfg.accel_lpf_hz));
        const math::Vector3f raw = (gyro - _last_gyro) * (1.0f / dt);
        if (std::isfinite(raw.x) && std::isfinite(raw.y) && std::isfinite(raw.z)) {
            _alpha = _alpha + (raw - _alpha) * a;
        }
    }
    _last_gyro = gyro;
    _have_gyro = true;

    const uint8_t n = std::min<uint8_t>(mixer.get_num_channels(), N);
    const float ku = dt / (dt + _cfg.output_tau_s);
    const float kl = dt / (dt + _cfg.motor_tau_s);
    float sum = 0.0f;
    int healthy = 0;
    for (uint8_t i = 0; i < n; ++i) {
        _u[i] += (mixer.last_output(i) - _u[i]) * ku;
        _u_lag[i] += (mixer.last_output(i) - _u_lag[i]) * kl;
        if (!mixer.is_motor_failed(i) && mixer.get_nominal_config(i).enabled) {
            sum += _u[i];
            ++healthy;
        }
    }

    if (!flying || collective < _cfg.min_collective || healthy < 2 ||
        mixer.allocation_status() == AllocationStatus::UNCONTROLLABLE) {
        for (uint8_t i = 0; i < N; ++i) {
            _onset_s[i] = 0.0f;
            _sustained_s[i] = 0.0f;
            _esc_low_s[i] = 0.0f;
        }
        return -1;
    }

    const bool steady = gyro.length() < _cfg.steady_rate;
    int best = -1;
    float best_score = 0.0f;
    Cause best_cause = Cause::NONE;
    for (uint8_t i = 0; i < n; ++i) {
        const ActuatorConfig& nom = mixer.get_nominal_config(i);
        if (mixer.is_motor_failed(i) || !nom.enabled || nom.bidirectional) continue;

        // 1. ESC telemetry: commanded, but not turning.
        const bool esc_fresh = _esc_ms[i] != 0 && now_ms - _esc_ms[i] <= _cfg.esc_timeout_ms;
        if (esc_fresh && _u[i] >= _cfg.esc_min_output && _esc_rpm[i] < _cfg.esc_min_rpm) {
            _esc_low_s[i] += dt;
        } else {
            _esc_low_s[i] = 0.0f;
        }
        if (_esc_low_s[i] >= _cfg.esc_time_s) {
            const float score = 10.0f + _esc_low_s[i];   // telemetry beats inference
            if (score > best_score) { best = i; best_score = score; best_cause = Cause::ESC_TELEMETRY; }
        }

        // 2a. Onset: the (lagged) commands push along ŝ_i, the body accelerates the other way.
        const float others = healthy > 1 ? (sum - _u[i]) / static_cast<float>(healthy - 1) : 0.0f;
        const float sx = nom.roll, sy = nom.pitch;
        const float sn = std::sqrt(sx * sx + sy * sy);
        if (sn > 1e-3f) {
            // Expected roll/pitch torque from every motor's lagged command (all-healthy
            // hypothesis), projected on ŝ_i and scaled to "motor i above balance" units.
            float push = 0.0f;
            for (uint8_t j = 0; j < n; ++j) {
                const ActuatorConfig& nj = mixer.get_nominal_config(j);
                if (!nj.enabled || nj.bidirectional || mixer.is_motor_failed(j)) continue;
                push += _u_lag[j] * (nj.roll * sx + nj.pitch * sy);
            }
            push /= sn * sn;
            const float against = -(_alpha.x * sx + _alpha.y * sy) / sn;   // rad/s^2
            if (push >= _cfg.onset_push && against >= _cfg.alpha_min) {
                _onset_s[i] += dt;
            } else {
                _onset_s[i] = std::max(0.0f, _onset_s[i] - 0.5f * dt);
            }
            if (_onset_s[i] >= _cfg.onset_time_s) {
                const float score = 1.0f + against / 100.0f;
                if (score > best_score) { best = i; best_score = score; best_cause = Cause::LOST_THRUST; }
            }
        }

        // 2b. Sustained: stays pinned near full while the others sit well below.
        // Only in steady flight: hard manoeuvring can pin a motor for a while too.
        if (steady && _u[i] >= _cfg.sustained_output && _u[i] - others >= _cfg.sustained_margin) {
            _sustained_s[i] += dt;
        } else {
            _sustained_s[i] = 0.0f;
        }
        if (_sustained_s[i] >= _cfg.sustained_time_s) {
            const float score = 1.0f + (_u[i] - others);
            if (score > best_score) { best = i; best_score = score; best_cause = Cause::LOST_THRUST; }
        }
    }

    if (best >= 0) {
        _last_cause = best_cause;
        for (uint8_t i = 0; i < N; ++i) {
            _onset_s[i] = 0.0f;
            _sustained_s[i] = 0.0f;
            _esc_low_s[i] = 0.0f;
        }
    }
    return best;
}

} // namespace control
} // namespace autopilot
