#pragma once
/**
 * Motor failure detection and isolation (FDI) for multirotors.
 *
 * Works on any frame the ActuatorMatrixMixer describes and needs no airframe
 * model. Two independent ways to find a dead motor (dead ESC, motor or lost
 * propeller):
 *
 *  1. Lost-thrust signature (always available). Motor i's healthy row says
 *     which way it pushes the vehicle: its (roll, pitch) coefficients ŝ_i. From
 *     the motor commands, delayed by the motor's spin-up lag, the detector knows
 *     which way the motors SHOULD be turning the body right now (assuming all are
 *     healthy). When motor i stops producing thrust, the attitude controller
 *     drives it up to correct the resulting tilt, so the commands push along
 *     +ŝ_i, yet the body keeps angular-accelerating along -ŝ_i. Motor i is suspect
 *     while
 *        - the lag-filtered commands push along ŝ_i by at least onset_push, AND
 *        - the body angular-accelerates against it (α · ŝ_i < -alpha_min),
 *     for onset_time_s. Comparing against the LAGGED command matters: on a hard
 *     stick reversal the command flips at once but the motors take tens of ms
 *     to follow, and for that moment the body still accelerates "against" the
 *     new command (a false alarm with the raw command).
 *     Once the other motors have taken over the torque the acceleration fades, so
 *     a second, slower rule catches a motor that stays pinned near full while the
 *     others sit well below it in steady (not manoeuvring) flight (sustained_time_s).
 *
 *  2. ESC telemetry (when the ESCs report RPM, e.g. DroneCAN esc.Status or
 *     BLHeli telemetry): commanded above esc_min_output but turning slower than
 *     esc_min_rpm for esc_time_s.
 *
 * The detector only reports. The vehicle decides what to do (re-allocate the
 * mixer, land). Only motors not already marked failed are monitored.
 */
#include "control/actuator_matrix_mixer.hpp"
#include "math/vector3.hpp"
#include <cstdint>

namespace autopilot {
namespace control {

class MotorFailureDetector {
public:
    struct Config {
        // Lost-thrust signature
        float accel_lpf_hz{25.0f};       // angular acceleration filter
        float output_tau_s{0.015f};      // motor command filter (sustained rule, ESC check)
        float motor_tau_s{0.04f};        // motor spin-up lag the expected torque is delayed by
        float onset_push{0.10f};         // expected push along ŝ_i, in "motor i above balance" units
        float alpha_min{5.0f};           // rad/s^2 against the motor's push
        float onset_time_s{0.03f};
        float sustained_output{0.95f};
        float sustained_margin{0.35f};   // above the mean of the other motors
        float sustained_time_s{0.4f};
        float steady_rate{0.5f};         // rad/s body rate: below = not manoeuvring (yaw included)
        float min_collective{0.15f};     // below this nothing is flying
        // ESC telemetry
        float esc_min_rpm{1000.0f};
        float esc_min_output{0.25f};
        float esc_time_s{0.05f};
        uint32_t esc_timeout_ms{200};
    };

    enum class Cause : uint8_t { NONE = 0, LOST_THRUST = 1, ESC_TELEMETRY = 2 };

    MotorFailureDetector() = default;
    explicit MotorFailureDetector(const Config& c) : _cfg(c) {}
    void set_config(const Config& c) { _cfg = c; }
    const Config& config() const { return _cfg; }

    /** Forget all evidence (on arm, after a re-allocation). */
    void reset();

    /** Telemetry from ESC `channel`: mechanical RPM at now_ms. */
    void report_esc_rpm(uint8_t channel, float rpm, uint32_t now_ms);

    /**
     * Call once per control step, after the mixer produced this step's outputs.
     *   gyro      body rates (rad/s, FRD)
     *   flying    armed and airborne (no detection on the ground)
     * Returns the channel of a newly detected failed motor, or -1.
     */
    int update(const ActuatorMatrixMixer& mixer, const math::Vector3f& gyro, float collective, float dt, bool flying,
               uint32_t now_ms);

    Cause last_cause() const { return _last_cause; }
    /** Filtered angular acceleration (rad/s^2), for logging. */
    const math::Vector3f& angular_accel() const { return _alpha; }

private:
    static constexpr uint8_t N = hal::PWMOutput::MAX_CHANNELS;
    Config _cfg;
    bool _have_gyro{false};
    math::Vector3f _last_gyro;
    math::Vector3f _alpha;
    float _u[N]{};
    float _u_lag[N]{};
    float _onset_s[N]{};
    float _sustained_s[N]{};
    float _esc_rpm[N]{};
    uint32_t _esc_ms[N]{};
    float _esc_low_s[N]{};
    Cause _last_cause{Cause::NONE};
};

const char* motor_failure_cause_name(MotorFailureDetector::Cause c);

} // namespace control
} // namespace autopilot
