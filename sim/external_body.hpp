#pragma once
/**
 * ExternalBody: a vehicle whose physics runs in another program.
 *
 * Test/SITL only. With `autopilot_sitl --physics json` the flight stack does
 * not simulate the vehicle itself: an external simulator (Gazebo through the
 * ardupilot_gazebo plugin, or any script that speaks the same JSON protocol,
 * see sim/json_physics.hpp) integrates the dynamics and sends back the true
 * state every physics step. This body just holds that state, so the SAME
 * simulated sensors (with the same noise and fault injection) read it as they
 * read the built-in quad, rover, boat or sub.
 */
#include "sim/sim_body.hpp"
#include <cmath>
#include <cstdint>

namespace autopilot {
namespace sim {

class ExternalBody : public SimBody {
public:
    int n_outputs = 0;              // channels the firmware drove on the last step (0 = disarmed)
    uint16_t last_pwm[16]{};

    const char* name() const override { return "external"; }

    /** Physics happens elsewhere; only remember what was commanded (for load()). */
    void step_pwm(const uint16_t* pwm_us, int n, double) override {
        n_outputs = pwm_us ? (n > 16 ? 16 : n) : 0;
        for (int i = 0; i < 16; ++i) last_pwm[i] = (pwm_us && i < n_outputs) ? pwm_us[i] : 0;
    }

    double load() const override {
        if (n_outputs == 0) return 0.0;
        double sum = 0.0;
        for (int i = 0; i < n_outputs; ++i) sum += unidir(last_pwm, n_outputs, i);
        return sum / n_outputs;
    }

    /**
     * Set the true state from one physics frame.
     * gyro: body rates FRD (rad/s); accel_body: specific force FRD (m/s^2, -9.8 z at rest);
     * pos/vel NED; quat body->NED (w, x, y, z).
     */
    void set_state(const double gyro[3], const double accel_body[3], const double pos_ned[3],
                   const double vel_ned[3], const double quat[4]) {
        double n = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
        if (n < 1e-9) n = 1.0;
        for (int i = 0; i < 4; ++i) q[i] = quat[i] / n;
        omega = V3(gyro[0], gyro[1], gyro[2]);
        pos = V3(pos_ned[0], pos_ned[1], pos_ned[2]);
        vel = V3(vel_ned[0], vel_ned[1], vel_ned[2]);
        // specific_force() = R^T (a - g)  =>  a = R f + g
        accel_ned = body_to_ned(V3(accel_body[0], accel_body[1], accel_body[2])) + V3(0, 0, g);
    }

    /** The external simulator owns the pose; placing is a no-op. */
    void place(double, double, double) override {}
};

} // namespace sim
} // namespace autopilot
