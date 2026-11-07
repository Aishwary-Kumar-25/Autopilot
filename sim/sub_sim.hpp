#pragma once
/**
 * ROV / submarine for SITL: 6-DOF rigid body with buoyancy, a righting moment,
 * hull damping and reversible thrusters at real positions.
 *
 * Test/SITL only. The thruster geometry here is physical (position + thrust
 * direction for each thruster, FRD body frame); it is NOT read from the mixer.
 * So if a mixer row has a wrong sign, the simulated vehicle really moves the
 * wrong way and the closed-loop tests catch it.
 *
 * Numbers are BlueROV2-like: 11.5 kg, ~1% positively buoyant, T200-class
 * thrusters (40 N forward / 32 N reverse), damping from published identification.
 * Added mass is ignored. Depth is reported through the barometer as -altitude,
 * which is what the ROV depth-hold controller reads.
 */
#include "sim/sim_body.hpp"
#include <algorithm>
#include <vector>

namespace autopilot {
namespace sim {

class SubModel : public SimBody {
public:
    struct Thruster { V3 pos, dir; };   // dir: unit thrust direction for positive output

    double mass = 11.5;
    V3 inertia{0.16, 0.16, 0.16};
    double buoyancy_ratio = 1.01;   // buoyancy / weight when fully submerged
    double hull_height = 0.25;      // m, for the partly-submerged buoyancy at the surface
    double cb_above_cg = 0.02;      // m: buoyancy above gravity -> self-righting
    double max_fwd = 40.0, max_rev = 32.0, thr_tau = 0.05;
    V3 lin_damp{4.0, 6.0, 6.0}, quad_damp{18.0, 21.0, 36.0};
    double rot_lin = 0.3, rot_quad = 1.5;

    std::vector<Thruster> thrusters;
    std::vector<double> thrust;
    int modelled_layout = 6;

    explicit SubModel(int n_thrusters = 6) { set_layout(n_thrusters); }

    const char* name() const override { return "sub"; }
    bool gps_available() const override { return pos.z < 0.2; } // antenna at the surface
    double load() const override {
        double s = 0;
        for (double t : thrust) s += std::fabs(t);
        return thrust.empty() ? 0.0 : s / (thrust.size() * max_fwd);
    }

    /** Resting depth when floating with thrusters off. */
    double float_depth() const { return hull_height / buoyancy_ratio - hull_height / 2.0; }

    void place(double north, double east, double yaw) override {
        SimBody::place(north, east, yaw);
        pos.z = float_depth();
        std::fill(thrust.begin(), thrust.end(), 0.0);
    }

    /**
     * Physical layouts matching the firmware presets (3 / 4 / 6 / 8). Returns
     * false for counts without a modelled geometry (the 6-thruster frame is used).
     */
    bool set_layout(int n) {
        const double s = 0.70710678;
        const V3 up(0, 0, -1);
        thrusters.clear();
        bool ok = true;
        if (n == 3 || n == 4) {
            thrusters.push_back({V3(-0.10, -0.15, 0), V3(1, 0, 0)});   // T1 port horizontal
            thrusters.push_back({V3(-0.10, 0.15, 0), V3(1, 0, 0)});    // T2 starboard horizontal
            if (n == 3) {
                thrusters.push_back({V3(0, 0, 0), up});                 // T3 centre vertical
            } else {
                thrusters.push_back({V3(0, -0.15, 0), up});             // T3 port vertical
                thrusters.push_back({V3(0, 0.15, 0), up});              // T4 starboard vertical
            }
        } else {
            ok = (n == 6 || n == 8);
            thrusters.push_back({V3(0.156, 0.111, 0), V3(s, -s, 0)});   // T1 front-right
            thrusters.push_back({V3(0.156, -0.111, 0), V3(s, s, 0)});   // T2 front-left
            thrusters.push_back({V3(-0.156, 0.111, 0), V3(s, s, 0)});   // T3 rear-right
            thrusters.push_back({V3(-0.156, -0.111, 0), V3(s, -s, 0)}); // T4 rear-left
            if (n == 8) {
                thrusters.push_back({V3(0.12, 0.218, 0), up});          // T5 front-right vertical
                thrusters.push_back({V3(0.12, -0.218, 0), up});         // T6 front-left
                thrusters.push_back({V3(-0.12, 0.218, 0), up});         // T7 rear-right
                thrusters.push_back({V3(-0.12, -0.218, 0), up});        // T8 rear-left
            } else {
                thrusters.push_back({V3(0, 0.218, 0), up});             // T5 right vertical
                thrusters.push_back({V3(0, -0.218, 0), up});            // T6 left vertical
            }
        }
        modelled_layout = static_cast<int>(thrusters.size());
        thrust.assign(thrusters.size(), 0.0);
        return ok;
    }

    void step_pwm(const uint16_t* pwm_us, int n, double dt) override {
        // Thrust: force + torque in the body frame.
        V3 f_body, tau;
        for (size_t i = 0; i < thrusters.size(); ++i) {
            const double c = bidir(pwm_us, n, static_cast<int>(i));
            thrust[i] = lag(thrust[i], c >= 0 ? c * max_fwd : c * max_rev, thr_tau, dt);
            const V3 f = thrusters[i].dir * thrust[i];
            f_body = f_body + f;
            tau = tau + thrusters[i].pos.cross(f);
        }

        // Hull damping on the body-frame velocity (still water).
        const V3 vb = ned_to_body(vel);
        f_body = f_body - V3(lin_damp.x * vb.x + quad_damp.x * vb.x * std::fabs(vb.x),
                             lin_damp.y * vb.y + quad_damp.y * vb.y * std::fabs(vb.y),
                             lin_damp.z * vb.z + quad_damp.z * vb.z * std::fabs(vb.z));
        tau = tau - V3(rot_lin * omega.x + rot_quad * omega.x * std::fabs(omega.x),
                       rot_lin * omega.y + rot_quad * omega.y * std::fabs(omega.y),
                       rot_lin * omega.z + rot_quad * omega.z * std::fabs(omega.z));

        // Gravity and buoyancy (partly out of the water near the surface).
        const double weight = mass * g;
        const double submerged = std::clamp((pos.z + hull_height / 2.0) / hull_height, 0.0, 1.0);
        const double buoy = buoyancy_ratio * weight * submerged;
        const V3 buoy_body = ned_to_body(V3(0, 0, -buoy));
        tau = tau + V3(0, 0, -cb_above_cg).cross(buoy_body);   // centre of buoyancy is above the CG

        accel_ned = (body_to_ned(f_body) + V3(0, 0, weight - buoy)) * (1.0 / mass);
        vel = vel + accel_ned * dt;
        pos = pos + vel * dt;

        const V3 Iw(inertia.x * omega.x, inertia.y * omega.y, inertia.z * omega.z);
        const V3 rhs = tau - omega.cross(Iw);
        omega = omega + V3(rhs.x / inertia.x, rhs.y / inertia.y, rhs.z / inertia.z) * dt;
        integrate_attitude(dt);
    }
};

} // namespace sim
} // namespace autopilot
