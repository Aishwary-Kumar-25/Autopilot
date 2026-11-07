#include "complementary.hpp"
#include <cmath>

namespace autopilot {
namespace fusion {

AltitudeEstimator::AltitudeEstimator(float w_z) {
    set_natural_frequency(w_z);
}

void AltitudeEstimator::set_natural_frequency(float w_z) {
    _k1 = 3.0f * w_z;
    _k2 = 3.0f * w_z * w_z;
    _k3 = w_z * w_z * w_z;
}

void AltitudeEstimator::reset(float initial_alt_m) {
    _alt_m = initial_alt_m;
    _vel_z_m_s = 0.0f;
    _accel_bias = 0.0f;
}

void AltitudeEstimator::predict(float accel_z_up_m_s2, float dt) {
    if (dt <= 1e-6f) return;

    // Apply estimated bias
    float corrected_accel = accel_z_up_m_s2 - _accel_bias;

    // Numerical integration (Euler / Trapezoidal)
    _alt_m += _vel_z_m_s * dt + 0.5f * corrected_accel * dt * dt;
    _vel_z_m_s += corrected_accel * dt;
}

void AltitudeEstimator::correct(float measured_alt_m, float dt) {
    if (dt <= 1e-6f) return;

    float error = measured_alt_m - _alt_m;

    _alt_m += _k1 * error * dt;
    _vel_z_m_s += _k2 * error * dt;
    _accel_bias -= _k3 * error * dt;
}

} // namespace fusion
} // namespace autopilot
