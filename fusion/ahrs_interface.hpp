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
 *  - Primary attitude, no GPS: Madgwick (gyro + accel + mag).
 *  - Primary attitude, GPS-aided: a multirotor's accelerometer reads thrust + drag,
 *    not gravity, so during a sustained bank or turn it "looks level" and Madgwick
 *    drifts toward level. With GPS velocity available the primary instead
 *      1. integrates the gyro (Madgwick with no accel feedback),
 *      2. corrects tilt by comparing the change in GPS velocity over a ~0.4 s window
 *         with the integral of the earth-frame specific force over the SAME window
 *         (shifted by the GPS latency), the approach ArduPilot's DCM uses, and
 *      3. corrects heading only (never tilt) from the tilt-compensated magnetometer.
 *    Comparing integrals over matched windows avoids the lag of differentiating GPS
 *    velocity, which otherwise points the correction the wrong way in a turn.
 *  - Backup attitude: ComplementaryAttitude, with the GPS-derived acceleration simply
 *    subtracted from the accelerometer (a deliberately different method).
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
    // GPS-aided tilt correction
    static constexpr float GPS_TILT_GAIN = 1.0f;          // 1/s: tilt error time constant ~1 s
    static constexpr uint32_t GPS_WINDOW_MS = 400;       // velocity-change comparison window
    // After the last GPS correction, coast on the gyro (+ mag heading) this long before
    // falling back to plain Madgwick. Covers 1 Hz receivers and short dropouts: a few
    // seconds of gyro drift is far smaller than the accel's pull toward level mid-manoeuvre.
    static constexpr uint32_t GPS_AIDING_TIMEOUT_MS = 3000;
    static constexpr uint32_t GPS_MAX_WINDOW_MS = 1100;  // longest usable window (1 Hz receivers)
    static constexpr uint32_t GPS_DEFAULT_DELAY_MS = 100; // receiver output latency (u-blox M8/M9: ~50-150 ms)
    static constexpr float MAG_YAW_GAIN = 0.3f;          // 1/s, heading-only mag correction while GPS-aided
    static constexpr uint32_t DEAD_RECKON_MS = 2000;     // horizontal dead reckoning after the last GPS sample

    SensorFusion(drivers::SensorManager* sensor_manager);

    void reset();
    void update(float dt);

    const VehicleState& get_state() const { return _state; }
    uint32_t estimator_switches() const { return _switches; }
    /** True while the primary attitude is being corrected from GPS velocity. */
    bool gps_aiding() const { return _gps_aiding; }
    /** GPS output latency (receiver dependent). The tilt comparison is shifted by this much. */
    void set_gps_delay_ms(uint32_t ms) { _gps_delay_ms = ms; }

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

    // GPS-aided tilt: earth-frame specific force integrated into 10 ms buckets
    // (1.6 s of history) so it can be summed over the same window as a GPS
    // velocity change. Per-bucket sums, not a running total, so float precision
    // doesn't degrade over a long flight.
    static constexpr int SF_BUCKETS = 160;
    static constexpr uint32_t SF_BUCKET_MS = 10;
    math::Vector3f _sf_sum[SF_BUCKETS]{};
    uint32_t _sf_slot[SF_BUCKETS]{};          // which 10 ms slot each bucket holds (+1; 0 = empty)
    struct GpsVel { uint32_t t_ms; math::Vector3f v; bool has_vz; };
    static constexpr int GPS_HIST = 16;
    GpsVel _gps_hist[GPS_HIST]{};
    int _gps_count{0}, _gps_head{0};
    uint32_t _gps_delay_ms{GPS_DEFAULT_DELAY_MS};
    uint32_t _now_ms{0};                      // IMU sample time
    uint32_t _last_gps_corr_ms{0};
    bool _have_gps_corr{false};
    bool _gps_aiding{false};
    bool _mag_in_use{false};

    // Baro reference
    bool _baro_ref_set{false};
    float _baro_ref_alt{0.0f};

    void update_attitude(float dt);
    void update_altitude(float dt);
    void update_position(float dt);
    void gps_tilt_correction(const GpsVel& sample);
    void mag_heading_correction(const math::Vector3f& mag, float dt);
};

} // namespace fusion
} // namespace autopilot
