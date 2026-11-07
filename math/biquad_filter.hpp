#pragma once

#include <cmath>

namespace autopilot {
namespace math {

/**
 * Second-order IIR (biquad) filter, transposed direct form II (2 state floats,
 * 5 multiplies per sample). Coefficients from the RBJ Audio EQ Cookbook.
 *
 *  - 2-pole low-pass: -40 dB/decade, much sharper than the 1-pole LowPassFilter
 *    for the same phase lag at low frequency. Typical use: gyro / D-term filter.
 *  - Notch: removes a narrow band (e.g. motor / propeller vibration at the
 *    blade-pass frequency) with almost no phase lag elsewhere, so the rate loop
 *    can keep high gains.
 *
 * A filter that was never configured (or got invalid parameters) passes the
 * signal through unchanged.
 */
class BiquadFilter {
public:
    BiquadFilter() = default;

    /** 2-pole Butterworth low-pass (Q = 1/sqrt(2)). */
    void set_lowpass(float cutoff_hz, float sample_hz) {
        if (!valid(cutoff_hz, sample_hz)) { set_passthrough(); return; }
        const float w0 = 2.0f * kPi * cutoff_hz / sample_hz;
        const float cw = std::cos(w0), sw = std::sin(w0);
        const float alpha = sw / (2.0f * 0.70710678f);
        const float a0 = 1.0f + alpha;
        _b0 = (1.0f - cw) * 0.5f / a0;
        _b1 = (1.0f - cw) / a0;
        _b2 = _b0;
        _a1 = -2.0f * cw / a0;
        _a2 = (1.0f - alpha) / a0;
        _enabled = true;
        reset();
    }

    /** Notch at center_hz with -3 dB bandwidth bandwidth_hz. */
    void set_notch(float center_hz, float bandwidth_hz, float sample_hz) {
        if (!valid(center_hz, sample_hz) || bandwidth_hz <= 0.0f) { set_passthrough(); return; }
        const float w0 = 2.0f * kPi * center_hz / sample_hz;
        const float cw = std::cos(w0), sw = std::sin(w0);
        const float q = center_hz / bandwidth_hz;
        const float alpha = sw / (2.0f * q);
        const float a0 = 1.0f + alpha;
        _b0 = 1.0f / a0;
        _b1 = -2.0f * cw / a0;
        _b2 = _b0;
        _a1 = _b1;
        _a2 = (1.0f - alpha) / a0;
        _enabled = true;
        reset();
    }

    void set_passthrough() {
        _enabled = false;
        reset();
    }

    bool enabled() const { return _enabled; }

    /** Reset state; the next sample primes the filter to its steady state (no start-up transient). */
    void reset() { _z1 = _z2 = 0.0f; _primed = false; }

    float apply(float x) {
        if (!_enabled) return x;
        if (!_primed) {
            // Steady state for a constant input x: output = DC gain * x.
            const float dc = (_b0 + _b1 + _b2) / (1.0f + _a1 + _a2);
            const float y = dc * x;
            _z1 = y - _b0 * x;              // from y = b0 x + z1
            _z2 = _b2 * x - _a2 * y;        // from the z2 update
            _primed = true;
        }
        const float y = _b0 * x + _z1;
        _z1 = _b1 * x - _a1 * y + _z2;
        _z2 = _b2 * x - _a2 * y;
        if (!std::isfinite(y)) { reset(); return x; } // never let a NaN latch inside the filter
        return y;
    }

private:
    static constexpr float kPi = 3.14159265358979f;
    float _b0{1.0f}, _b1{0.0f}, _b2{0.0f}, _a1{0.0f}, _a2{0.0f};
    float _z1{0.0f}, _z2{0.0f};
    bool _enabled{false};
    bool _primed{false};

    static bool valid(float f, float fs) { return f > 0.0f && fs > 0.0f && f < 0.45f * fs; }
};

} // namespace math
} // namespace autopilot
