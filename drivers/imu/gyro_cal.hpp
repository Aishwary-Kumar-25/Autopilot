#pragma once

#include "math/vector3.hpp"
#include <cmath>
#include <cstdint>

namespace autopilot {
namespace drivers {

/**
 * Running mean/variance (Welford) of gyro samples for bias calibration, with a
 * "was the vehicle still?" verdict. Calibrating while someone carries or bumps
 * the vehicle bakes that motion into the bias, and the attitude then drifts at
 * that rate for the whole flight; such a calibration must be rejected.
 */
class GyroCalAccumulator {
public:
    // Noise of a still MEMS gyro at the 42-44 Hz DLPF is ~0.005 rad/s rms; walking
    // with the vehicle is 10x that.
    static constexpr float MAX_STDDEV_RAD_S = 0.03f;
    // A still gyro's raw bias is well under 5 deg/s on these parts.
    static constexpr float MAX_BIAS_RAD_S = 0.2f;
    static constexpr uint16_t MIN_SAMPLES = 20;

    void add(const math::Vector3f& g) {
        ++_n;
        const math::Vector3f d = g - _mean;
        _mean += d / static_cast<float>(_n);
        const math::Vector3f d2 = g - _mean;
        _m2.x += d.x * d2.x;
        _m2.y += d.y * d2.y;
        _m2.z += d.z * d2.z;
    }

    uint16_t count() const { return _n; }
    const math::Vector3f& mean() const { return _mean; }

    float max_stddev() const {
        if (_n < 2) return 0.0f;
        const float inv = 1.0f / static_cast<float>(_n - 1);
        const float m = std::fmax(_m2.x, std::fmax(_m2.y, _m2.z));
        return std::sqrt(m * inv);
    }

    /** True if enough samples were taken, the vehicle was still and the bias is plausible. */
    bool still() const {
        return _n >= MIN_SAMPLES && max_stddev() <= MAX_STDDEV_RAD_S && _mean.length() <= MAX_BIAS_RAD_S;
    }

private:
    uint16_t _n{0};
    math::Vector3f _mean{0, 0, 0};
    math::Vector3f _m2{0, 0, 0};
};

} // namespace drivers
} // namespace autopilot
