#pragma once

#include "vehicles/vehicle_base.hpp"
#include "uav_mixer.hpp"
#include "math/pid.hpp"
#include "math/biquad_filter.hpp"
#include "control/motor_failure_detector.hpp"
#include <algorithm>

namespace autopilot {
namespace vehicles {

enum class UAVFlightMode {
    MANUAL,
    STABILIZE,
    ALT_HOLD
};

/** The UAV's PID loops, for tuning from a parameter system / ground station. */
enum class UAVPid : uint8_t {
    ROLL_ANGLE, PITCH_ANGLE, ROLL_RATE, PITCH_RATE, YAW_RATE, ALTITUDE, CLIMB_RATE
};

class UAVVehicle : public VehicleBase {
public:
    explicit UAVVehicle(hal::HAL* hal, uint8_t initial_rotors = 4);

    const char* get_type_name() const override;

    /**
     * Change flight mode. Entering ALT_HOLD captures the current altitude as the
     * target (and, when hovering in STABILIZE, seeds the hover throttle), so a
     * mid-air mode switch holds where the vehicle is instead of flying to a stale target.
     */
    void set_mode(UAVFlightMode mode);
    UAVFlightMode get_mode() const { return _mode; }

    void set_target_altitude(float target_alt_m) { _target_altitude_m = target_alt_m; }

    /**
     * @brief Configure number of rotors (3 to 16, e.g. Quad, Hexa, Octa).
     */
    bool set_rotor_count(uint8_t count);
    uint8_t get_rotor_count() const { return _rotor_count; }

    control::ActuatorMatrixMixer* get_matrix_mixer() { return _matrix_mixer; }

    /** Controller step + motor failure monitoring (detection acts on the next step). */
    void step(float dt) override;

    // ---- Motor failure handling ----
    /**
     * After every step the motor failure detector watches the motor commands and
     * the body's angular acceleration (and ESC RPM, if reported). On a detected
     * failure the mixer re-allocates control to the remaining motors:
     *   - DEGRADED (hexa, octa, ...): keeps flying under pilot control, yaw
     *     authority reduced; a GCS warning recommends landing.
     *   - NO_YAW (quad on 3 motors), not enough thrust left, or UNCONTROLLABLE:
     *     failsafe LAND (committed), yaw abandoned, the vehicle spins down.
     * Failures stay latched until the next arm.
     */
    control::MotorFailureDetector& motor_failure_detector() { return _motor_fdi; }
    void set_motor_failure_detection(bool enabled) { _motor_fdi_enabled = enabled; }
    /** Declare a motor failed from outside (ESC fault flag, operator, test). */
    void declare_motor_failed(uint8_t channel, control::MotorFailureDetector::Cause cause);
    /** ESC telemetry in: forwarded to the detector. */
    void report_esc_rpm(uint8_t channel, float rpm) { _motor_fdi.report_esc_rpm(channel, rpm, now_ms()); }
    int8_t last_failed_motor() const { return _last_failed_motor; }
    uint32_t motor_failure_time_ms() const { return _motor_failure_ms; }
    bool motor_emergency() const { return _motor_emergency; }

    /** Learned hover throttle (starts at 0.5, adapts slowly while hovering in ALT_HOLD). */
    float get_hover_throttle() const { return _hover_throttle; }
    void set_hover_throttle(float t) { _hover_throttle = std::clamp(t, 0.15f, 0.85f); }
    float get_target_altitude() const { return _target_altitude_m; }
    bool is_landed() const { return _landed; }
    bool multirotor_in_air() const override { return is_armed() && !_landed; }

    /**
     * Optional rate-loop gyro filtering (both off by default):
     *  - lpf_hz   : 2-pole low-pass on the gyro used by the rate PIDs (0 = off)
     *  - notch_hz : notch at the motor/prop vibration frequency (0 = off)
     * sample_hz is the rate step() is called at.
     */
    void set_gyro_filters(float lpf_hz, float notch_hz, float notch_bw_hz, float sample_hz);

    /** Access one PID loop (gains are read with config(), changed with set_gains()). */
    math::PID& pid(UAVPid which) {
        switch (which) {
            case UAVPid::ROLL_ANGLE:  return _roll_angle_pid;
            case UAVPid::PITCH_ANGLE: return _pitch_angle_pid;
            case UAVPid::ROLL_RATE:   return _roll_rate_pid;
            case UAVPid::PITCH_RATE:  return _pitch_rate_pid;
            case UAVPid::YAW_RATE:    return _yaw_rate_pid;
            case UAVPid::ALTITUDE:    return _altitude_pid;
            case UAVPid::CLIMB_RATE:
            default:                  return _velocity_z_pid;
        }
    }

    // Tunables (exposed for the simulator / parameter system)
    static constexpr float MAX_ANGLE_RAD = 30.0f * 0.0174532925f;
    static constexpr float MAX_YAW_RATE = 3.0f;      // rad/s
    static constexpr float MAX_CLIMB_RATE = 2.0f;    // m/s
    static constexpr float LAND_SPEED = 0.6f;        // m/s failsafe descent
    static constexpr float STICK_DEADBAND = 0.1f;    // around mid stick in ALT_HOLD
    static constexpr float THROTTLE_GROUND = 0.05f;  // below this the pilot wants motors idle

protected:
    void update_controller(float dt) override;
    bool check_arming_inputs(std::string& failure_reason) const override;
    void on_arm() override;
    void on_disarm() override;
    bool apply_failsafe_land(float dt) override;
    bool actuator_failure_needs_land() const override { return _motor_emergency; }

private:
    UAVFlightMode _mode{UAVFlightMode::STABILIZE};
    float _target_altitude_m{1.0f};
    uint8_t _rotor_count{4};
    control::ActuatorMatrixMixer* _matrix_mixer{nullptr};

    // Cascaded Attitude Controllers
    math::PID _roll_angle_pid;
    math::PID _pitch_angle_pid;
    math::PID _roll_rate_pid;
    math::PID _pitch_rate_pid;
    math::PID _yaw_rate_pid;

    // Altitude Controller
    math::PID _altitude_pid;
    math::PID _velocity_z_pid;

    math::BiquadFilter _gyro_lpf[3];
    math::BiquadFilter _gyro_notch[3];
    math::Vector3f filtered_rates();

    control::MotorFailureDetector _motor_fdi;
    bool _motor_fdi_enabled{true};
    bool _motor_emergency{false};
    int8_t _last_failed_motor{-1};
    uint32_t _motor_failure_ms{0};
    /** Yaw can't be controlled (quad on 3 motors): no yaw loop, spin allowed. */
    bool yaw_abandoned() const;

    float _hover_throttle{0.5f};
    // Level-equivalent collective that was holding altitude over the last ~second
    // (ALT_HOLD with a valid altitude). If the altitude estimate is lost, mid stick
    // keeps this collective instead of a hover guess. 0 = not known yet.
    float _althold_thr_lp{0.0f};
    float _last_throttle{0.0f};
    bool _landed{true};
    float _landed_timer_s{0.0f};

    void setup_pids();
    void reset_pids();
    /** Attitude cascade -> roll/pitch/yaw demands. */
    void run_attitude(float target_roll, float target_pitch, float target_yaw_rate, float dt, bool integrate);
    /** Climb-rate loop -> collective throttle (hover-relative, tilt compensated). */
    float run_climb_rate(float desired_climb_m_s, float dt, bool integrate);
    float tilt_compensate(float throttle) const;
    void update_land_detector(float desired_climb, float dt, bool require_low_thrust = true);
};

} // namespace vehicles
} // namespace autopilot
