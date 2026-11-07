/**
 * SITL: attitude without GPS, using the rotor-drag model and the online gyro bias.
 *
 * Part 1, drag. Without GPS a multirotor's accelerometer cannot tell a held bank
 * from level flight: thrust is along body Z, so the body X/Y specific force is only
 * drag. The fusion uses that: drag is a known function of airspeed, so the
 * accelerometer gives the body X/Y airspeed, and a velocity observer turns the
 * mismatch with the integrated acceleration into a tilt correction.
 *
 *   A. 15 deg bank held for 4 s (long sideways acceleration)
 *   B. 12 deg forward pitch + yaw rate for 6 s (sustained turn)
 *   E. 18 deg dash for 4 s, then hard braking (the case named in the roadmap)
 *
 * each flown with the drag model off (accelerometer only, the old behaviour) and on,
 * then again with a 5 m/s wind, quadratic body drag the fusion only approximates,
 * and drag coefficients set 2x too low / too high.
 *
 * Part 2, gyro bias. A residual bias (on top of the boot calibration, e.g. from
 * temperature drift) is injected after arming. The fusion must learn it from its
 * own corrections, in flight with and without GPS, and on the ground.
 */
#include "comms/params.hpp"
#include "comms/vehicle_params.hpp"
#include "tests/sitl_rig.hpp"
#include "tests/test_common.hpp"
#include <cstdio>

using namespace autopilot;
using namespace autopilot::sitl;

namespace {

struct Setup {
    bool gps{false};
    bool drag{true};
    bool bias_learning{true};
    double wind_n{0.0}, wind_e{0.0};
    double cda{0.0};          // sim bluff-body drag area, m^2
    float mcoef_scale{1.0f};  // fusion's momentum drag coefficient vs. the true one
    float mcoef_abs{0.0f};    // > 0: use exactly this coefficient (e.g. learned in an earlier flight)
    double accel_noise{0.05}; // m/s^2 rms (vibration)
    double gust{0.0};         // m/s amplitude of a 0.4 Hz / 0.25 Hz gusting wind on top
    math::Vector3f gyro_bias{0, 0, 0};
};

struct Result {
    double max_err_deg{0};
    double end_err_deg{0};
    double hover_err_deg{0};
    double max_truth_deg{0};
    math::Vector3f bias_est{0, 0, 0};
    double max_yaw_err_deg{0};
    float mcoef{0};
    uint32_t switches{0};     // primary/backup estimator switches
    bool flying{false};
};

double yaw_err_deg(const Rig& r) {
    const double yaw_true = std::atan2(2 * (r.quad.q[0] * r.quad.q[3] + r.quad.q[1] * r.quad.q[2]),
                                       1 - 2 * (r.quad.q[2] * r.quad.q[2] + r.quad.q[3] * r.quad.q[3]));
    double d = r.vm->get_latest_state().euler_rad.z - yaw_true;
    while (d > M_PI) d -= 2 * M_PI;
    while (d < -M_PI) d += 2 * M_PI;
    return std::fabs(d) * RAD2DEG;
}

struct Phase { float roll, pitch, yaw; double seconds; bool measure{true}; };

Result fly(const Setup& s, const Phase* phases, int n_phases, double hover_before_s = 3.0) {
    Rig r(s.gps);
    Result out;
    if (!r.uav) return out;
    auto* fusion = r.vm->get_fusion();
    fusion->set_drag_aiding_enabled(s.drag);
    fusion->set_gyro_bias_learning(s.bias_learning);
    // The true momentum drag is quad.drag / quad.mass per second.
    fusion->set_drag_model(s.mcoef_abs > 0.0f ? s.mcoef_abs : static_cast<float>(r.quad.drag / r.quad.mass) * s.mcoef_scale,
                           s.cda > 0.0 ? static_cast<float>(r.quad.mass / s.cda) : 0.0f);
    r.quad.drag_cda = s.cda;
    r.quad.wind = sim::V3(s.wind_n, s.wind_e, 0.0);

    r.uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
    r.run(1.0);
    std::string err;
    if (!r.vm->arm(err)) return out;
    r.imu0->gyro_bias = r.imu1->gyro_bias = s.gyro_bias; // drift appears after the boot cal
    r.imu0->accel_noise = r.imu1->accel_noise = s.accel_noise;
    r.run(1.0);
    r.rc[3] = 0.8f;
    r.run(2.5);
    r.rc[3] = 0.5f;
    r.run(hover_before_s);
    out.hover_err_deg = r.est_err_deg();

    for (int i = 0; i < n_phases; ++i) {
        r.rc[0] = phases[i].roll;
        r.rc[1] = phases[i].pitch;
        r.rc[2] = phases[i].yaw;
        const double end = r.t + phases[i].seconds;
        while (r.t < end) {
            if (s.gust > 0.0) {
                r.quad.wind = sim::V3(s.wind_n + s.gust * std::sin(2 * M_PI * 0.4 * r.t),
                                      s.wind_e + s.gust * std::cos(2 * M_PI * 0.25 * r.t), 0.0);
            }
            r.step();
            if (!phases[i].measure) continue;
            out.max_err_deg = std::fmax(out.max_err_deg, r.est_err_deg());
            out.max_yaw_err_deg = std::fmax(out.max_yaw_err_deg, yaw_err_deg(r));
            out.max_truth_deg = std::fmax(out.max_truth_deg,
                                          std::fmax(std::fabs(r.quad.roll()), std::fabs(r.quad.pitch())) * RAD2DEG);
        }
    }
    out.end_err_deg = r.est_err_deg();
    out.bias_est = fusion->gyro_bias();
    out.mcoef = fusion->drag_mcoef();
    out.switches = fusion->estimator_switches();
    out.flying = r.vm->is_armed() && !r.quad.on_ground && r.quad.altitude() > 1.0;
    return out;
}

// Manoeuvres (each ends with 3 s back at level stick)
const Phase BANK[] = {{0.5f, 0.0f, 0.0f, 4.0}, {0, 0, 0, 3.0}};
const Phase TURN[] = {{0.0f, -0.4f, 0.5f, 6.0}, {0, 0, 0, 3.0}};
const Phase DASH_BRAKE[] = {{0.0f, -0.6f, 0.0f, 4.0}, {0.0f, 0.6f, 0.0f, 2.0}, {0, 0, 0, 3.0}};

struct Manoeuvre { const char* name; const Phase* p; int n; };
const Manoeuvre MANOEUVRES[] = {
    {"A. 15 deg bank, 4 s", BANK, 2},
    {"B. pitch + yaw turn", TURN, 2},
    {"E. dash, then brake", DASH_BRAKE, 3},
};

} // namespace

int main() {
    std::printf("== SITL attitude without GPS: rotor-drag aiding ==\n");
    for (int m = 0; m < 3; ++m) {
        const Manoeuvre& M = MANOEUVRES[m];
        Setup off;
        off.drag = false;
        const Result a = fly(off, M.p, M.n);
        const Result d = fly(Setup{}, M.p, M.n);
        std::printf("  %-22s accel only: max err %5.2f deg | drag model: max err %5.2f deg, end %4.2f (truth up to %4.1f)\n",
                    M.name, a.max_err_deg, d.max_err_deg, d.end_err_deg, d.max_truth_deg);
        CHECK(a.flying && d.flying);
        CHECK(a.max_err_deg > 2.0);                   // the manoeuvre really fools the accelerometer
        CHECK(d.max_err_deg < 0.35 * a.max_err_deg); // drag model removes most of the error
        CHECK(d.max_err_deg < 2.0);
        CHECK(d.end_err_deg < 0.75);
        CHECK(d.hover_err_deg < 0.5);
        CHECK(d.switches == 0); // primary and backup agree: no estimator switching
    }

    // Robustness, drag model on. Each case flies A, B and E; for wrong coefficients
    // that is the first flight, then a short dash/brake calibration flight learns the
    // coefficient, and A, B, E are flown again with the learned value (as if stored).
    std::printf("  robustness (drag model on):\n");
    const float c_true = 0.25f / 1.2f; // QuadModel: drag / mass
    struct Case { const char* name; Setup s; };
    Setup wind;      wind.wind_n = 5.0; wind.wind_e = -2.0;
    Setup bluff;     bluff.cda = 0.012;
    Setup low;       low.mcoef_scale = 0.5f;
    Setup high;      high.mcoef_scale = 2.0f;
    Setup gusts;     gusts.wind_n = 3.0; gusts.gust = 1.0;
    Setup vib;       vib.accel_noise = 1.5;
    Setup all;       all.wind_n = 5.0; all.wind_e = -2.0; all.cda = 0.012; all.mcoef_scale = 1.5f;
    const Case cases[] = {
        {"wind 5.4 m/s", wind},
        {"bluff-body drag", bluff},
        {"gusts 3+-1 m/s", gusts},
        {"vibration 1.5 m/s2", vib},
        {"mcoef 2x too low", low},
        {"mcoef 2x too high", high},
        {"wind+bluff+1.5x", all},
    };
    const double accel_only_worst = 12.0; // A/E above are 12-15 deg with the accelerometer alone
    auto worst_of = [](const Setup& s, double& worst, double& hover, bool& flying) {
        worst = hover = 0;
        flying = true;
        for (const Manoeuvre& M : MANOEUVRES) {
            const Result x = fly(s, M.p, M.n);
            worst = std::fmax(worst, x.max_err_deg);
            hover = std::fmax(hover, x.hover_err_deg);
            flying = flying && x.flying;
        }
    };
    for (const Case& c : cases) {
        double worst, hover;
        bool flying;
        worst_of(c.s, worst, hover, flying);
        std::printf("    %-18s worst err %5.2f deg over A/B/E, hover err %4.2f deg\n", c.name, worst, hover);
        CHECK(flying);
        if (c.s.gust > 0.0 || c.s.accel_noise > 0.1) {
            // Gusts (unmodelled airspeed changes) and vibration (white noise at the
            // sample rate, a pessimistic model) cost accuracy, but the drag model stays
            // well ahead of the accelerometer alone in the same conditions.
            Setup acc = c.s;
            acc.drag = false;
            double acc_worst, acc_hover;
            bool acc_flying;
            worst_of(acc, acc_worst, acc_hover, acc_flying);
            std::printf("    %-18s   (accel only, same conditions: worst err %5.2f deg)\n", "", acc_worst);
            CHECK(worst < 0.5 * acc_worst);
            CHECK(hover < 1.0);
            continue;
        }
        if (c.s.mcoef_scale == 1.0f) {
            CHECK(worst < 2.0);
            CHECK(hover < 0.5); // steady wind must not be read as tilt
            continue;
        }
        // Wrong coefficient, first flight: still better than the accelerometer alone.
        CHECK(worst < 0.8 * accel_only_worst);
        CHECK(hover < 1.5);
        const Phase cal[] = {{0.0f, -0.6f, 0.0f, 4.0}, {0.0f, 0.6f, 0.0f, 2.0}, {0, 0, 0, 2.0},
                             {0.0f, -0.6f, 0.0f, 4.0}, {0.0f, 0.6f, 0.0f, 2.0}, {0, 0, 0, 2.0}};
        const Result learned = fly(c.s, cal, 6);
        Setup next = c.s;
        next.mcoef_abs = learned.mcoef;
        worst_of(next, worst, hover, flying);
        std::printf("    %-18s   learned mcoef %.3f (true %.3f, set %.3f); next flight worst err %5.2f deg, hover %4.2f\n",
                    "", learned.mcoef, c_true, c_true * c.s.mcoef_scale, worst, hover);
        CHECK(flying);
        CHECK_NEAR(learned.mcoef, c_true, 0.15 * c_true);
        CHECK(worst < 2.0);
        CHECK(hover < 0.5);
    }

    std::printf("== SITL online gyro bias ==\n");
    const math::Vector3f bias(0.03f, -0.025f, 0.02f); // 1.7, -1.4, 1.1 deg/s
    {
        // 25 s hover (not scored) to learn, then the turn: bias learned in flight
        // without GPS (drag + mag). Errors are measured over the turn and after it.
        const Phase hover_turn[] = {{0, 0, 0, 25.0, false}, {0.0f, -0.4f, 0.5f, 6.0}, {0, 0, 0, 3.0}};
        Setup on;
        on.gyro_bias = bias;
        Setup off = on;
        off.bias_learning = false;
        const Result l = fly(on, hover_turn, 3);
        const Result n = fly(off, hover_turn, 3);
        std::printf("  no GPS, learning off: turn: max tilt err %5.2f deg, max yaw err %5.2f deg\n", n.max_err_deg, n.max_yaw_err_deg);
        std::printf("  no GPS, learning on : turn: max tilt err %5.2f deg, max yaw err %5.2f deg, bias est (%.4f %.4f %.4f) true (%.4f %.4f %.4f)\n",
                    l.max_err_deg, l.max_yaw_err_deg, l.bias_est.x, l.bias_est.y, l.bias_est.z, bias.x, bias.y, bias.z);
        CHECK(l.flying && n.flying);
        CHECK_NEAR(l.bias_est.x, bias.x, 0.006);
        CHECK_NEAR(l.bias_est.y, bias.y, 0.006);
        CHECK_NEAR(l.bias_est.z, bias.z, 0.006);
        CHECK(l.end_err_deg < 0.5);
        CHECK(l.max_err_deg < 0.75);
        CHECK(l.max_err_deg < 0.5 * n.max_err_deg);
        CHECK(l.max_yaw_err_deg < 0.5 * n.max_yaw_err_deg);
    }
    {
        // Same with GPS: the GPS tilt correction has no integral term of its own.
        const Phase hover_turn[] = {{0, 0, 0, 25.0, false}, {0.0f, -0.4f, 0.5f, 6.0}, {0, 0, 0, 3.0}};
        Setup on;
        on.gps = true;
        on.gyro_bias = bias;
        Setup off = on;
        off.bias_learning = false;
        const Result l = fly(on, hover_turn, 3);
        const Result n = fly(off, hover_turn, 3);
        std::printf("  GPS, learning off   : end tilt err %5.2f deg\n", n.end_err_deg);
        std::printf("  GPS, learning on    : end tilt err %5.2f deg, bias est (%.4f %.4f %.4f)\n", l.end_err_deg,
                    l.bias_est.x, l.bias_est.y, l.bias_est.z);
        CHECK(l.flying);
        CHECK_NEAR(l.bias_est.x, bias.x, 0.006);
        CHECK_NEAR(l.bias_est.y, bias.y, 0.006);
        CHECK(l.end_err_deg < 0.5);
        CHECK(l.end_err_deg < n.end_err_deg);
    }
    {
        // On the ground, disarmed: Madgwick (accel + mag) path, bias learned before take-off.
        Rig r(false);
        r.imu0->gyro_bias = r.imu1->gyro_bias = bias;
        r.run(40.0);
        const math::Vector3f b = r.vm->get_fusion()->gyro_bias();
        std::printf("  on the ground, 40 s : bias est (%.4f %.4f %.4f), tilt err %.2f deg\n", b.x, b.y, b.z, r.est_err_deg());
        CHECK_NEAR(b.x, bias.x, 0.005);
        CHECK_NEAR(b.y, bias.y, 0.005);
        CHECK_NEAR(b.z, bias.z, 0.005);
        CHECK(r.est_err_deg() < 0.5);
        // The rate controller sees the corrected rate.
        CHECK(r.vm->get_latest_state().angular_velocity.length() < 0.015f);
    }

    std::printf("== learned drag coefficient -> saved parameter ==\n");
    {
        Rig r(false);
        comms::ParamStore ps;
        comms::register_vehicle_params(ps, *r.vm);
        auto* fusion = r.vm->get_fusion();
        CHECK(ps.set("EKF_DRAG_MCOEF", 0.1f) == comms::ParamStore::SetResult::OK);
        CHECK_NEAR(fusion->drag_mcoef(), 0.1f, 1e-6);
        CHECK(ps.set("EKF_DRAG_BCOEF", 0.0f) == comms::ParamStore::SetResult::OK);
        r.uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
        r.run(1.0);
        std::string err;
        CHECK(r.vm->arm(err));
        r.rc[3] = 0.8f;
        r.run(2.5);
        r.rc[3] = 0.5f;
        r.run(2.0);
        for (int k = 0; k < 2; ++k) {
            r.rc[1] = -0.6f; r.run(4.0);
            r.rc[1] = 0.6f;  r.run(2.0);
            r.rc[1] = 0.0f;  r.run(2.0);
        }
        const float learned = fusion->drag_mcoef();
        comms::sync_learned_params(ps, *r.vm);   // armed: nothing is written
        CHECK_NEAR(ps.value(ps.find("EKF_DRAG_MCOEF")), 0.1f, 1e-6);
        r.vm->disarm();
        comms::sync_learned_params(ps, *r.vm);
        const float saved = ps.value(ps.find("EKF_DRAG_MCOEF"));
        std::printf("  set 0.100, learned %.3f in flight, parameter after disarm %.3f\n", learned, saved);
        CHECK(learned > 0.17f);
        CHECK_NEAR(saved, learned, 1e-6);
    }

    return test::finish("test_sitl_drag_bias");
}
