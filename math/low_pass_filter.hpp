#pragma once

#include <cmath>

namespace autopilot {
namespace math {

class LowPassFilter {
public:
    LowPassFilter() : _cutoff_hz(0.0f), _alpha(1.0f), _output(0.0f), _initialized(false) {}
    LowPassFilter(float cutoff_hz, float sample_rate_hz) {
        set_cutoff_frequency(cutoff_hz, sample_rate_hz);
    }

    void set_cutoff_frequency(float cutoff_hz, float sample_rate_hz) {
        _cutoff_hz = cutoff_hz;
        if (_cutoff_hz <= 0.0f || sample_rate_hz <= 0.0f) {
            _alpha = 1.0f;
            return;
        }
        float dt = 1.0f / sample_rate_hz;
        float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * _cutoff_hz);
        _alpha = dt / (dt + rc);
    }

    float apply(float sample) {
        if (!_initialized) {
            _output = sample;
            _initialized = true;
            return _output;
        }
        _output += _alpha * (sample - _output);
        return _output;
    }

    float get_output() const {
        return _output;
    }

    void reset(float value = 0.0f) {
        _output = value;
        _initialized = false;
    }

private:
    float _cutoff_hz{0.0f};
    float _alpha{1.0f};
    float _output{0.0f};
    bool _initialized{false};
};

} // namespace math
} // namespace autopilot
