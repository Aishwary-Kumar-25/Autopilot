#pragma once

#include <algorithm>
#include <cmath>

namespace autopilot {
namespace math {

class PID {
public:
    struct Config {
        float kp{0.0f};
        float ki{0.0f};
        float kd{0.0f};
        float kff{0.0f};
        float imax{0.0f};       // Max absolute integrator contribution
        float out_min{-1.0f};   // Min overall output
        float out_max{1.0f};    // Max overall output
        float d_filter_hz{30.0f}; // Cutoff frequency for D-term derivative filter
    };

    PID() = default;
    explicit PID(const Config& config) : _config(config) {}

    void set_gains(float kp, float ki, float kd, float kff = 0.0f) {
        _config.kp = kp;
        _config.ki = ki;
        _config.kd = kd;
        _config.kff = kff;
    }

    void set_limits(float out_min, float out_max, float imax) {
        _config.out_min = out_min;
        _config.out_max = out_max;
        _config.imax = imax;
    }

    void reset() {
        _integrator = 0.0f;
        _last_derivative = 0.0f;
        _last_error = 0.0f;
        _last_target = 0.0f;
        _has_run = false;
    }

    float update(float target, float measurement, float dt) {
        if (dt <= 1e-6f) {
            return 0.0f;
        }

        float error = target - measurement;

        // Proportional term
        float p_term = _config.kp * error;

        // Integral term with anti-windup
        _integrator += _config.ki * error * dt;
        if (_config.imax > 0.0f) {
            _integrator = std::clamp(_integrator, -_config.imax, _config.imax);
        }

        // Derivative term calculated on measurement or error to prevent derivative kick
        float derivative = 0.0f;
        if (_has_run) {
            float raw_derivative = (error - _last_error) / dt;
            // First order low pass filter on D-term
            if (_config.d_filter_hz > 0.0f) {
                float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * _config.d_filter_hz);
                float alpha = dt / (dt + rc);
                derivative = _last_derivative + alpha * (raw_derivative - _last_derivative);
            } else {
                derivative = raw_derivative;
            }
        }
        _last_derivative = derivative;
        _last_error = error;
        _last_target = target;
        _has_run = true;

        float d_term = _config.kd * derivative;
        float ff_term = _config.kff * target;

        float output = p_term + _integrator + d_term + ff_term;
        return std::clamp(output, _config.out_min, _config.out_max);
    }

    float get_integrator() const { return _integrator; }
    void set_integrator(float val) { _integrator = val; }

private:
    Config _config;
    float _integrator{0.0f};
    float _last_derivative{0.0f};
    float _last_error{0.0f};
    float _last_target{0.0f};
    bool _has_run{false};
};

} // namespace math
} // namespace autopilot
