#pragma once

#include "state_types.hpp"
#include <cmath>
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
 *  - Gyro bias: estimated online (on top of the boot calibration) from every
 *    correction the primary applies, mapped into the body frame, like the integral
 *    term of a Mahony filter. Temperature drift in flight is learned instead of
 *    showing up as a tilt or heading error. The backup uses the raw gyro, so a bad
 *    bias estimate can't take both estimators with it.
 *  - Primary attitude, no GPS, on the ground (or not a multirotor): Madgwick
 *    (gyro + accel + mag).
 *  - Primary attitude, no GPS, multirotor in the air: rotor-drag aiding. A
 *    multirotor's accelerometer reads thrust + drag; thrust is along body Z, so the
 *    body X/Y specific force is drag only, and drag is a known function of the
 *    air-relative velocity (f_xy = -mcoef*v - rho*|v|*v/(2*bcoef), PX4/ArduPilot
 *    EKF drag fusion). A velocity observer integrates the earth-frame specific force
 *    plus gravity, compares the result in the body frame with the velocity the drag
 *    model reads off the accelerometer, and turns the difference into a tilt
 *    correction (a tilt error makes the integrated velocity drift sideways at g per
 *    radian). Sustained banks, turns and braking no longer look level. A constant
 *    wind cancels out: the model works on air-relative velocity.
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
    // Rotor-drag aided tilt (multirotor in the air, no GPS)
    static constexpr float DRAG_MCOEF_DEFAULT = 0.2f;    // 1/s, momentum (rotor) drag: accel per m/s of airspeed
    static constexpr float DRAG_BCOEF_DEFAULT = 100.0f;  // kg/m^2, ballistic coeff. of the body (0 = no bluff drag)
    static constexpr float DRAG_VEL_GAIN = 1.6f;         // 1/s, velocity observer gain
    static constexpr float DRAG_TILT_GAIN = 1.0f;        // (1/s^2): with the above, ~1 rad/s, damping 0.8
    static constexpr float DRAG_INNOV_TAU_S = 0.1f;      // low-pass on the drag-velocity innovation (vibration)
    // Vibration: the drag airspeed is the accelerometer divided by mcoef (~5x noise
    // gain), so on a vibrating frame the observer is slowed down. Noise is measured
    // from sample-to-sample differences of body X/Y specific force; above the reference
    // level the gains scale with sqrt(ref / noise), down to DRAG_MIN_GAIN_SCALE.
    static constexpr float DRAG_NOISE_REF = 0.15f;       // m/s^2 rms per sample
    static constexpr float DRAG_MIN_GAIN_SCALE = 0.3f;
    static constexpr float DRAG_MAX_AIRSPEED = 25.0f;    // beyond this the model is not trusted
    // Right after take-off the airspeed can jump (wind the ground was holding back):
    // correct velocity only, fast, before trusting the observer for tilt.
    static constexpr float DRAG_SETTLE_S = 1.5f;
    static constexpr float DRAG_SETTLE_VEL_GAIN = 5.0f;  // 1/s
    // Online drag-coefficient calibration. Over a 2 s window the gyro alone keeps the
    // attitude accurate, so integrating the specific force with a gyro-only copy of
    // the attitude gives the true change in airspeed (constant wind cancels). The
    // change in drag-measured airspeed over the same window is (true mcoef / assumed
    // mcoef) times that, which corrects the coefficient. Several staggered windows.
    static constexpr int DRAG_CAL_TRACKS = 4;
    static constexpr float DRAG_CAL_WINDOW_S = 3.0f;
    static constexpr float DRAG_CAL_MIN_DV = 3.0f;       // m/s: only windows with a real speed change
    static constexpr float DRAG_CAL_GAIN = 0.5f;        // fraction of log(ratio) applied per window
    static constexpr float DRAG_MCOEF_MIN = 0.03f, DRAG_MCOEF_MAX = 1.0f;
    // With drag aiding available, stop trusting old GPS corrections sooner.
    static constexpr uint32_t GPS_AIDING_TIMEOUT_DRAG_MS = 1500;
    // Online gyro bias: bias moves by -BIAS_GAIN x each body-frame correction (rad),
    // i.e. integral/proportional ratio of a Mahony filter (time constant ~1/(gain*Kp)).
    static constexpr float BIAS_GAIN = 0.1f;
    static constexpr float BIAS_MAX_RAD_S = 0.1f;        // residual after boot cal; clamp
    static constexpr float BIAS_LEARN_MAX_RATE = 1.5f;   // rad/s: don't learn during fast rotation (scale-factor errors)
    static constexpr float BIAS_LEARN_MAX_STEP = 0.01f;  // rad per tick: larger "corrections" are resets, not drift
    // While drag-aided, learn only when the drag observer agrees with the gyro (a large
    // innovation is a transient or a model error, not gyro drift).
    static constexpr float BIAS_LEARN_MAX_DRAG_INNOV = 0.6f; // m/s

    enum class TiltAiding : uint8_t { ACCEL = 0, DRAG = 1, GPS = 2 };

    SensorFusion(drivers::SensorManager* sensor_manager);

    void reset();
    void update(float dt);

    const VehicleState& get_state() const { return _state; }
    uint32_t estimator_switches() const { return _switches; }
    /** True while the primary attitude is being corrected from GPS velocity. */
    bool gps_aiding() const { return _gps_aiding; }
    /** GPS output latency (receiver dependent). The tilt comparison is shifted by this much. */
    void set_gps_delay_ms(uint32_t ms) { _gps_delay_ms = ms; }

    /**
     * Tell the fusion whether the vehicle is a multirotor that is flying (armed and
     * not landed). Only then is the body X/Y specific force pure drag, so only then
     * is rotor-drag aiding used. VehicleManager sets this every loop.
     */
    void set_multirotor_in_air(bool in_air) { _mc_in_air = in_air; }
    /** Airframe drag model. mcoef 1/s (> 0), bcoef kg/m^2 (0 disables the quadratic term). */
    void set_drag_model(float mcoef, float bcoef);
    float drag_mcoef() const { return _drag_mcoef; }
    /** Turn rotor-drag aiding / online gyro bias learning on or off (both on by default). */
    void set_drag_aiding_enabled(bool on) { _drag_enabled = on; }
    void set_gyro_bias_learning(bool on) { _bias_learning = on; }
    /** Learn mcoef in flight (on by default). Learned value: drag_mcoef(); store it to keep it. */
    void set_drag_calibration(bool on) { _drag_cal_enabled = on; }
    uint32_t drag_calibration_windows() const { return _drag_cal_windows; }
    float drag_bcoef() const { return _drag_bcoef; }
    TiltAiding tilt_aiding() const { return _tilt_aiding; }
    /** Online gyro bias estimate (rad/s, body), already removed from state.angular_velocity. */
    const math::Vector3f& gyro_bias() const { return _gyro_bias; }
    /** Measured accelerometer noise (m/s^2 rms per sample, body X/Y). */
    float accel_noise() const { return std::sqrt(_accel_noise_var); }
    /** Air-relative velocity from the drag observer (NED, m/s); valid while drag_aiding. */
    const math::Vector3f& air_velocity_ned() const { return _drag_vel; }

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
    bool _gps_lin_valid{false};
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
    TiltAiding _tilt_aiding{TiltAiding::ACCEL};

    // Online gyro bias
    math::Vector3f _gyro_bias{0, 0, 0};
    math::Quaternion _q_pred{1, 0, 0, 0};    // last primary attitude, propagated by the gyro only
    bool _q_pred_valid{false};
    const void* _bias_imu{nullptr};           // IMU the bias belongs to

    // Rotor-drag aiding
    bool _mc_in_air{false};
    bool _drag_enabled{true};
    bool _drag_cal_enabled{true};
    struct DragCalTrack {
        bool active{false};
        float t{0.0f};
        math::Quaternion q;               // gyro-only attitude since the window started
        math::Vector3f dv_int{0, 0, 0};   // integral of (R f + g) dt, NED
        math::Vector3f vm0{0, 0, 0};      // drag-measured airspeed at the start, NED
        math::Vector3f a0{0, 0, 0};       // kinematic acceleration at the start, NED (lag correction)
    };
    DragCalTrack _cal[DRAG_CAL_TRACKS]{};
    float _cal_clock{0.0f};
    uint32_t _drag_cal_windows{0};
    bool _bias_learning{true};
    bool _drag_active{false};                 // observer initialised and running
    float _drag_time_s{0.0f};                 // since the observer started
    float _accel_noise_var{0.0f};             // body X/Y specific force noise variance (m/s^2)^2
    math::Vector3f _prev_accel{0, 0, 0};
    bool _have_prev_accel{false};
    float _drag_mcoef{DRAG_MCOEF_DEFAULT};
    float _drag_bcoef{DRAG_BCOEF_DEFAULT};
    math::Vector3f _drag_vel{0, 0, 0};        // air-relative velocity estimate, NED
    math::Vector3f _drag_innov{0, 0, 0};      // low-passed body X/Y innovation
    math::Vector3f _drag_vm_lp{0, 0, 0};      // drag-measured body velocity (X/Y), low-passed
    math::Vector3f _drag_lin_body{0, 0, 0};   // attitude-independent body accel from the drag model

    // Baro reference
    bool _baro_ref_set{false};
    float _baro_ref_alt{0.0f};
    const void* _baro_ref_src{nullptr}; // barometer the reference belongs to

    void update_attitude(float dt);
    void update_altitude(float dt);
    void update_position(float dt);
    void gps_tilt_correction(const GpsVel& sample);
    void mag_heading_correction(const math::Vector3f& mag, float dt);
    float drag_velocity_from_accel(float f) const;
    void drag_observer(const math::Vector3f& gyro, const math::Vector3f& accel, bool apply_tilt, float dt);
    void learn_gyro_bias();
    void drag_calibration(const math::Vector3f& gyro, const math::Vector3f& accel, float dt);
};

} // namespace fusion
} // namespace autopilot
