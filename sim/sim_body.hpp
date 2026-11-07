#pragma once
/**
 * Common base for every simulated vehicle body (quad, rover, boat, sub).
 *
 * Test/SITL only, not flight code. A body owns the true state (NED position and
 * velocity, body->NED attitude quaternion, FRD body rates and the kinematic
 * acceleration) that the simulated sensors in sim/quad_sim.hpp read. Because the
 * sensors only see a SimBody, the SAME sensor set (and so the same firmware
 * sensor stack and estimator) runs on every vehicle type.
 *
 * Each body turns the raw PWM pulses the firmware wrote (microseconds, exactly
 * what would go to ESCs / servos) into forces with its own physics.
 */
#include <cmath>
#include <cstdint>

namespace autopilot {
namespace sim {

struct V3 {
    double x{0}, y{0}, z{0};
    V3() = default;
    V3(double a, double b, double c) : x(a), y(b), z(c) {}
    V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
    V3 cross(const V3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

class SimBody {
public:
    virtual ~SimBody() = default;

    double g = 9.80665;

    // True state
    V3 pos, vel;                  // NED, m and m/s
    double q[4]{1, 0, 0, 0};      // body -> NED (w, x, y, z)
    V3 omega;                     // body rates, rad/s (FRD)
    V3 accel_ned;                 // last kinematic acceleration (NED)

    virtual const char* name() const = 0;

    /**
     * Advance by dt. pwm_us[i] is output channel i in microseconds; n == 0 (or a
     * null pointer) means the outputs are disabled (disarmed): no thrust / neutral.
     */
    virtual void step_pwm(const uint16_t* pwm_us, int n, double dt) = 0;

    /** Fraction of full power currently drawn, 0..1 (for the simulated battery). */
    virtual double load() const = 0;

    /** False when the GPS antenna can't see the sky (e.g. a submerged ROV). */
    virtual bool gps_available() const { return true; }

    /** Put the body at rest at (north, east) facing yaw, on its natural resting place. */
    virtual void place(double north, double east, double yaw) {
        pos = V3(north, east, 0.0);
        vel = omega = accel_ned = V3();
        set_yaw(yaw);
    }

    // ---- frames ----
    V3 body_to_ned(const V3& v) const {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        return {
            (1 - 2 * (y * y + z * z)) * v.x + 2 * (x * y - w * z) * v.y + 2 * (x * z + w * y) * v.z,
            2 * (x * y + w * z) * v.x + (1 - 2 * (x * x + z * z)) * v.y + 2 * (y * z - w * x) * v.z,
            2 * (x * z - w * y) * v.x + 2 * (y * z + w * x) * v.y + (1 - 2 * (x * x + y * y)) * v.z};
    }
    V3 ned_to_body(const V3& v) const {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        return {
            (1 - 2 * (y * y + z * z)) * v.x + 2 * (x * y + w * z) * v.y + 2 * (x * z - w * y) * v.z,
            2 * (x * y - w * z) * v.x + (1 - 2 * (x * x + z * z)) * v.y + 2 * (y * z + w * x) * v.z,
            2 * (x * z + w * y) * v.x + 2 * (y * z - w * x) * v.y + (1 - 2 * (x * x + y * y)) * v.z};
    }
    double roll() const { return std::atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])); }
    double pitch() const { return std::asin(std::fmax(-1.0, std::fmin(1.0, 2 * (q[0] * q[2] - q[3] * q[1])))); }
    double yaw() const { return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])); }
    double altitude() const { return -pos.z; }
    double ground_speed() const { return std::sqrt(vel.x * vel.x + vel.y * vel.y); }

    /** Specific force the accelerometer measures (body, m/s^2): a - g. */
    V3 specific_force() const { return ned_to_body(accel_ned - V3(0, 0, g)); }

    void set_yaw(double yaw) {
        q[0] = std::cos(yaw / 2); q[1] = 0; q[2] = 0; q[3] = std::sin(yaw / 2);
    }

protected:
    /** Quaternion kinematics with the current body rates. */
    void integrate_attitude(double dt) {
        const double w = q[0], x = q[1], y = q[2], z = q[3];
        const double p = omega.x, qq = omega.y, r = omega.z;
        q[0] += 0.5 * dt * (-x * p - y * qq - z * r);
        q[1] += 0.5 * dt * (w * p + y * r - z * qq);
        q[2] += 0.5 * dt * (w * qq - x * r + z * p);
        q[3] += 0.5 * dt * (w * r + x * qq - y * p);
        const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (double& c : q) c /= n;
    }

    /** First-order lag toward target (motor / servo response). */
    static double lag(double current, double target, double tau, double dt) {
        return current + (target - current) * (dt / (tau + dt));
    }
    /** Bidirectional ESC / servo pulse -> [-1, 1] (1500 us = 0), missing = 0. */
    static double bidir(const uint16_t* pwm, int n, int ch) {
        if (!pwm || ch >= n) return 0.0;
        return std::fmax(-1.0, std::fmin(1.0, (static_cast<double>(pwm[ch]) - 1500.0) / 500.0));
    }
    /** Unidirectional ESC pulse -> [0, 1] (1000 us = 0), missing = 0. */
    static double unidir(const uint16_t* pwm, int n, int ch) {
        if (!pwm || ch >= n) return 0.0;
        return std::fmax(0.0, std::fmin(1.0, (static_cast<double>(pwm[ch]) - 1000.0) / 1000.0));
    }
};

} // namespace sim
} // namespace autopilot
