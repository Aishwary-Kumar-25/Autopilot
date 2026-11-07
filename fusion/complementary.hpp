#pragma once

#include "math/vector3.hpp"

namespace autopilot {
namespace fusion {

/**
 * @brief 3rd-order Baro-Inertial complementary filter for altitude and vertical speed estimation.
 * Fuses vertical Earth-frame acceleration with Barometer altitude.
 */
class AltitudeEstimator {
public:
    AltitudeEstimator(float w_z = 2.0f);

    void reset(float initial_alt_m = 0.0f);
    void set_natural_frequency(float w_z);

    /**
     * @brief Predict step with vertical acceleration (Earth frame, upward positive).
     */
    void predict(float accel_z_up_m_s2, float dt);

    /**
     * @brief Correct step with Barometer/Rangefinder altitude measurement.
     */
    void correct(float measured_alt_m, float dt);

    float get_altitude() const { return _alt_m; }
    float get_vertical_velocity() const { return _vel_z_m_s; }
    float get_accel_bias() const { return _accel_bias; }

private:
    float _alt_m{0.0f};
    float _vel_z_m_s{0.0f};
    float _accel_bias{0.0f};

    // Filter gains derived from natural frequency w_z
    float _k1{2.0f};  // 3 * w_z
    float _k2{1.0f};  // 3 * w_z^2
    float _k3{0.1f};  // w_z^3
};

} // namespace fusion
} // namespace autopilot
