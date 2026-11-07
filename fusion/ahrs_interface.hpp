#pragma once

#include "state_types.hpp"
#include "madgwick.hpp"
#include "complementary.hpp"
#include "drivers/sensor_manager.hpp"

namespace autopilot {
namespace fusion {

/**
 * Simple tilt/heading complementary filter. Used as the backup attitude
 * estimator: a deliberately different algorithm from Madgwick with no shared
 * state, so a bug or divergence in one is unlikely to hit the other.
 */
struct ComplementaryAttitude {
    float roll{0.0f}, pitch{0.0f}, yaw{0.0f};
    // Accel/mag correction time constant. A multirotor's accelerometer barely sees
    // tilt during sustained manoeuvres (thrust is always along body Z), so a short
    // time constant drags the estimate toward level. 0 = snap to the accel/mag.
    float tau_s{4.0f};
    void update(const math::Vector3f& gyro, const math::Vector3f& accel, const math::Vector3f* mag, float dt);
    static void tilt_from_accel(const math::Vector3f& accel, float& roll, float& pitch);
};

/**
 * Universal sensor fusion (shared by every vehicle type).
 *
 * Frame convention: body FRD, earth NED, accelerometer = specific force
 * (level at rest reads (0,0,-g)). See drivers/sensor_interface.hpp.
 *
 *  - Primary attitude: Madgwick. Before use as a gravity reference, the GPS-observed
 *    linear acceleration is removed from the accelerometer reading; otherwise a
 *    sustained turn or acceleration slowly drags the attitude toward "level".
 *  - Backup attitude: ComplementaryAttitude.
 *  - Selector: if the two disagree on tilt by > SWITCH_DEG for > SWITCH_TIME_S, the
 *    one closer to the (filtered) accelerometer tilt wins. NaN always loses.
 *  - Altitude: 3rd-order baro/accel complementary filter.
 *  - Horizontal position/velocity: GPS (relative to the first fix) blended with accel.
 */
class SensorFusion {
public:
    static constexpr float SWITCH_DEG = 20.0f;
    static constexpr float SWITCH_TIME_S = 0.5f;
    // Madgwick gain: high right after alignment for fast convergence, then low so
    // sustained manoeuvre accelerations don't pull the attitude toward level.
    static constexpr float BETA_ALIGN = 0.1f;
    static constexpr float BETA_FLIGHT = 0.03f;
    static constexpr float ALIGN_SETTLE_S = 3.0f;

    SensorFusion(drivers::SensorManager* sensor_manager);

    void reset();
    void update(float dt);

    const VehicleState& get_state() const { return _state; }
    uint32_t estimator_switches() const { return _switches; }

    math::Vector3f body_to_ned(const math::Vector3f& v_body) const;
    math::Vector3f ned_to_body(const math::Vector3f& v_ned) const;

private:
    drivers::SensorManager* _sensors{nullptr};
    MadgwickFilter _attitude_filter{0.1f};
    ComplementaryAttitude _backup;
    AltitudeEstimator _alt_estimator{2.5f};
    VehicleState _state;

    bool _aligned{false};
    float _disagree_time{0.0f};
    uint8_t _source{0};
    uint32_t _switches{0};
    float _acc_roll_lp{0.0f}, _acc_pitch_lp{0.0f};

    // GPS
    bool _have_home{false};
    double _home_lat{0.0}, _home_lon{0.0};
    uint32_t _last_gps_ms{0};
    math::Vector3f _prev_gps_vel{0, 0, 0};
    math::Vector3f _gps_lin_accel{0, 0, 0};
    float _time_since_align{0.0f};

    // Baro reference
    bool _baro_ref_set{false};
    float _baro_ref_alt{0.0f};

    void update_attitude(float dt);
    void update_altitude(float dt);
    void update_position(float dt);
};

} // namespace fusion
} // namespace autopilot
