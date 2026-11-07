/**
 * Biquad low-pass / notch filters and the compass hard/soft-iron calibrator.
 */
#include "math/biquad_filter.hpp"
#include "drivers/mag/compass_calibrator.hpp"
#include "tests/test_common.hpp"
#include <cmath>
#include <cstdio>

using namespace autopilot;

// Steady-state amplitude gain of a filter for a sine at f_hz.
static float gain_at(math::BiquadFilter f, float f_hz, float fs) {
    f.reset();
    const int n = static_cast<int>(fs * 2.0f);
    float peak = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float y = f.apply(std::sin(2.0f * 3.14159265f * f_hz * i / fs));
        if (i > n / 2) peak = std::fmax(peak, std::fabs(y));
    }
    return peak;
}

static void test_biquad() {
    std::printf("[biquad filters]\n");
    const float fs = 1000.0f;
    math::BiquadFilter lp;
    lp.set_lowpass(50.0f, fs);
    CHECK_NEAR(gain_at(lp, 5.0f, fs), 1.0f, 0.01);       // passband
    CHECK_NEAR(gain_at(lp, 50.0f, fs), 0.7071f, 0.03);   // -3 dB at cutoff
    CHECK(gain_at(lp, 400.0f, fs) < 0.03f);              // 2-pole: ~-37 dB at 8x cutoff

    math::BiquadFilter notch;
    notch.set_notch(120.0f, 30.0f, fs);
    CHECK(gain_at(notch, 120.0f, fs) < 0.02f);           // > 34 dB rejection at the centre
    CHECK_NEAR(gain_at(notch, 10.0f, fs), 1.0f, 0.02);   // flight-control band untouched
    CHECK_NEAR(gain_at(notch, 400.0f, fs), 1.0f, 0.05);

    // First sample primes the state: a constant input passes with no start-up transient.
    math::BiquadFilter p;
    p.set_lowpass(20.0f, fs);
    CHECK_NEAR(p.apply(3.0f), 3.0f, 1e-4);
    CHECK_NEAR(p.apply(3.0f), 3.0f, 1e-4);

    // Invalid settings / NaN input: pass-through, and NaN never latches.
    math::BiquadFilter bad;
    bad.set_lowpass(600.0f, fs);  // above Nyquist
    CHECK(!bad.enabled());
    CHECK_NEAR(bad.apply(1.5f), 1.5f, 0.0);
    p.apply(std::nanf(""));
    CHECK(std::isfinite(p.apply(1.0f)));
}

static void test_compass_cal() {
    std::printf("[compass calibration]\n");
    const math::Vector3f off(0.12f, -0.30f, 0.05f);   // hard iron (gauss)
    const math::Vector3f stretch(1.15f, 0.90f, 1.0f); // soft iron (diagonal)
    const float earth = 0.48f;
    uint32_t s = 99;
    auto rnd = [&]() { s = s * 1664525u + 1013904223u; return static_cast<float>(s >> 8) / 16777216.0f; };

    drivers::CompassCalibrator cal;
    for (int i = 0; i < 400; ++i) {
        // uniform direction on the sphere
        const float z = 2.0f * rnd() - 1.0f, a = 6.2831853f * rnd(), r = std::sqrt(1.0f - z * z);
        const math::Vector3f u(r * std::cos(a), r * std::sin(a), z);
        const float noise = 0.004f * (rnd() - 0.5f);
        const math::Vector3f raw(u.x * earth * stretch.x + off.x + noise, u.y * earth * stretch.y + off.y - noise,
                                 u.z * earth * stretch.z + off.z + noise);
        cal.add_sample(raw);
    }
    const auto res = cal.fit();
    std::printf("  ok=%d offset=(%.3f %.3f %.3f) scale=(%.3f %.3f %.3f) r=%.3f rms=%.4f %s\n", res.ok, res.offset.x,
                res.offset.y, res.offset.z, res.scale.x, res.scale.y, res.scale.z, res.field_radius, res.rms_residual,
                res.failure);
    CHECK(res.ok);
    CHECK_NEAR(res.offset.x, off.x, 0.005); CHECK_NEAR(res.offset.y, off.y, 0.005); CHECK_NEAR(res.offset.z, off.z, 0.005);
    // Corrected field has the same length in every direction
    CHECK_NEAR(res.scale.x / res.scale.z, 1.0f / 1.15f, 0.01);
    CHECK_NEAR(res.scale.y / res.scale.z, 1.0f / 0.90f, 0.01);
    const math::Vector3f px = drivers::CompassCalibrator::apply(res, math::Vector3f(earth * stretch.x + off.x, off.y, off.z));
    const math::Vector3f py = drivers::CompassCalibrator::apply(res, math::Vector3f(off.x, earth * stretch.y + off.y, off.z));
    CHECK_NEAR(px.length(), py.length(), 0.005);
    CHECK(res.rms_residual < 0.02f);

    // Only rotated about yaw (z never swings): rejected for poor coverage.
    drivers::CompassCalibrator flat;
    for (int i = 0; i < 300; ++i) {
        const float a = 6.2831853f * i / 300.0f;
        flat.add_sample(math::Vector3f(0.3f * std::cos(a) + off.x, 0.3f * std::sin(a) + off.y, 0.38f + off.z + 0.01f * (rnd() - 0.5f)));
    }
    CHECK(!flat.fit().ok);

    // Too few samples
    drivers::CompassCalibrator few;
    few.add_sample(math::Vector3f(0.1f, 0.2f, 0.3f));
    CHECK(!few.fit().ok);
}

int main() {
    test_biquad();
    test_compass_cal();
    return test::finish("test_filters_and_cal");
}
