#pragma once
/**
 * Surface vehicles for SITL: an Ackermann rover and a twin-thruster boat.
 *
 * Test/SITL only. Both read the same PWM pulses the firmware's UGV mixer writes
 * (ch0/ch1, 1500 us = neutral) and drive the same simulated sensors as the quad,
 * so the shared SensorManager + SensorFusion run unchanged on them.
 */
#include "sim/sim_body.hpp"
#include <algorithm>

namespace autopilot {
namespace sim {

/**
 * 1/10-scale Ackermann rover (kinematic bicycle with a grip limit).
 *   ch0 steering servo (2000 us = full right), ch1 drive ESC (2000 us = full forward).
 * The vehicle stays level on flat ground; turning produces real centripetal
 * acceleration on the accelerometer, which the estimator must not read as tilt.
 */
class RoverModel : public SimBody {
public:
    double mass = 3.0;              // kg
    double wheelbase = 0.33;        // m
    double max_steer = 0.52;        // rad (30 deg)
    double steer_tau = 0.08;        // s, servo
    double max_drive = 15.0;        // N at full throttle
    double esc_tau = 0.10;          // s
    double c_roll = 0.03;           // rolling resistance coefficient
    double c_aero = 0.3;            // N per (m/s)^2 (air + drivetrain losses)
    double max_lat_accel = 0.8 * 9.80665; // tyre grip limit

    double speed = 0.0;             // signed forward speed, m/s
    double steer = 0.0;             // rad
    double drive = 0.0;             // N

    const char* name() const override { return "rover"; }
    double load() const override { return std::fabs(drive) / max_drive; }
    void place(double north, double east, double yaw) override {
        SimBody::place(north, east, yaw);
        speed = steer = drive = 0.0;
    }

    void step_pwm(const uint16_t* pwm_us, int n, double dt) override {
        steer = lag(steer, bidir(pwm_us, n, 0) * max_steer, steer_tau, dt);
        drive = lag(drive, bidir(pwm_us, n, 1) * max_drive, esc_tau, dt);

        // Longitudinal: drive force against rolling resistance (smoothed sign) + air drag.
        const double resist = c_roll * mass * g * std::tanh(speed / 0.05) + c_aero * speed * std::fabs(speed);
        double a_long = (drive - resist) / mass;
        if (std::fabs(speed) < 0.02 && std::fabs(drive) < c_roll * mass * g) {  // static friction
            a_long = 0.0;
            speed = 0.0;
        }
        speed += a_long * dt;

        // Yaw rate from steering geometry, limited by tyre grip.
        double r = speed * std::tan(steer) / wheelbase;
        if (std::fabs(speed * r) > max_lat_accel) r = std::copysign(max_lat_accel / std::fabs(speed), r);
        omega = V3(0.0, 0.0, r);
        integrate_attitude(dt);

        vel = body_to_ned(V3(speed, 0.0, 0.0));
        accel_ned = body_to_ned(V3(a_long, speed * r, 0.0));
        pos = pos + vel * dt;
        pos.z = 0.0;
    }
};

/**
 * Twin-thruster boat (catamaran), 3-DOF surge / sway / yaw with linear +
 * quadratic hull damping and an optional constant water current.
 *   ch0 port thruster, ch1 starboard thruster (1500 us = stopped, reversible;
 *   props are weaker in reverse, like real ones).
 * Unlike the rover there is no grip: the hull skids sideways in a turn and
 * drifts with the current, so heading and course differ.
 */
class BoatModel : public SimBody {
public:
    double mass = 6.0;              // kg
    double iz = 0.8;                // kg m^2
    double half_beam = 0.25;        // m, thruster offset from centreline
    double max_fwd = 12.0;          // N per thruster
    double max_rev = 7.0;           // N per thruster
    double thr_tau = 0.15;          // s
    double xu = 2.0, xuu = 3.0;     // surge damping
    double yv = 10.0, yvv = 15.0;   // sway damping (hull resists sideways motion)
    double nr = 1.5, nrr = 1.0;     // yaw damping
    V3 current;                     // NED water current, m/s

    double u = 0.0, v = 0.0;        // body velocity relative to the water
    double t_port = 0.0, t_stbd = 0.0;

    const char* name() const override { return "boat"; }
    double load() const override { return (std::fabs(t_port) + std::fabs(t_stbd)) / (2.0 * max_fwd); }
    void place(double north, double east, double yaw) override {
        SimBody::place(north, east, yaw);
        u = v = t_port = t_stbd = 0.0;
    }

    void step_pwm(const uint16_t* pwm_us, int n, double dt) override {
        auto thrust = [this](double c) { return c >= 0 ? c * max_fwd : c * max_rev; };
        t_port = lag(t_port, thrust(bidir(pwm_us, n, 0)), thr_tau, dt);
        t_stbd = lag(t_stbd, thrust(bidir(pwm_us, n, 1)), thr_tau, dt);
        double r = omega.z;

        const double X = t_port + t_stbd - (xu * u + xuu * u * std::fabs(u));
        const double Y = -(yv * v + yvv * v * std::fabs(v));
        // Port (left, y = -b) thruster pushing forward turns the bow to starboard (+yaw).
        const double N = (t_port - t_stbd) * half_beam - (nr * r + nrr * r * std::fabs(r));

        // Rigid body in a rotating frame: m(u' - v r) = X, m(v' + u r) = Y.
        const double ax = X / mass, ay = Y / mass;
        u += (ax + v * r) * dt;
        v += (ay - u * r) * dt;
        r += N / iz * dt;

        omega = V3(0.0, 0.0, r);
        integrate_attitude(dt);
        accel_ned = body_to_ned(V3(ax, ay, 0.0));
        vel = body_to_ned(V3(u, v, 0.0)) + current;
        pos = pos + vel * dt;
        pos.z = 0.0;
    }
};

} // namespace sim
} // namespace autopilot
