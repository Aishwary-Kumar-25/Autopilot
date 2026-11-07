/**
 * SITL: GPS-aided attitude during sustained manoeuvres.
 *
 * A multirotor's accelerometer sees thrust + drag, not gravity: while it keeps
 * accelerating sideways at a held bank angle, the specific force stays along body
 * Z and "looks level". An attitude filter that trusts it as a gravity reference
 * drifts toward level, so the true bank angle ends up larger than both the
 * estimate and the pilot's command.
 *
 * The fusion removes the GPS-observed acceleration from the accelerometer before
 * using it as the gravity reference. This test flies the same manoeuvres with and
 * without GPS and checks that GPS aiding keeps the estimate on the truth.
 *
 * Manoeuvres (ALT_HOLD, after a climb to ~2.4 m):
 *   A. 15 deg bank held for 4 s (long sideways acceleration)
 *   B. 12 deg forward pitch + yaw rate for 6 s (sustained turn)
 *   C. B again with a mis-modelled GPS latency, and with 5 Hz / 1 Hz receivers
 *   D. A again with the GPS lost half way through
 */
#include "tests/sitl_rig.hpp"
#include "tests/test_common.hpp"
#include <cstdio>

using namespace autopilot;
using namespace autopilot::sitl;

namespace {

struct Result {
    double max_err_deg{0};   // worst |estimate - truth| over the manoeuvre
    double end_err_deg{0};   // error at the end of the held phase
    double end_truth_deg{0}; // true tilt at the end of the held phase
    double hover_err_deg{0}; // error in steady hover just before the manoeuvre
    bool flying{false};
};

double tilt_err_deg(const Rig& r) { return r.est_err_deg(); }

struct GpsSetup {
    uint32_t period_ms{100};
    uint32_t latency_ms{100};
    double dropout_at_s{-1}; // seconds into the manoeuvre; <0 = never
};

Result fly(bool with_gps, float roll_stick, float pitch_stick, float yaw_stick, double hold_s,
           GpsSetup g = GpsSetup{}) {
    Rig r(with_gps);
    Result out;
    if (r.gps) {
        r.gps->period_ms = g.period_ms;
        r.gps->latency_ms = g.latency_ms;
    }
    if (!r.uav) return out;
    r.uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
    r.run(1.0);
    std::string err;
    if (!r.vm->arm(err)) return out;
    r.run(1.0);
    r.rc[3] = 0.8f;
    r.run(2.5);
    r.rc[3] = 0.5f;
    r.run(3.0);

    r.rc[0] = roll_stick;
    r.rc[1] = pitch_stick;
    r.rc[2] = yaw_stick;
    const double start = r.t, end = r.t + hold_s;
    out.hover_err_deg = tilt_err_deg(r);
    while (r.t < end) {
        if (r.gps && g.dropout_at_s >= 0 && r.t - start >= g.dropout_at_s) r.gps->set_fault(sim::Fault::DROPOUT);
        r.step();
        out.max_err_deg = std::fmax(out.max_err_deg, tilt_err_deg(r));
    }
    out.end_err_deg = tilt_err_deg(r);
    out.end_truth_deg = std::fmax(std::fabs(r.quad.roll()), std::fabs(r.quad.pitch())) * 57.29577951;

    // Back to level: the estimate must not have been corrupted.
    r.rc[0] = r.rc[1] = r.rc[2] = 0.0f;
    r.run(3.0);
    out.max_err_deg = std::fmax(out.max_err_deg, tilt_err_deg(r));
    out.flying = r.vm->is_armed() && !r.quad.on_ground && r.quad.altitude() > 1.0;
    return out;
}

void report(const char* name, const Result& no_gps, const Result& gps) {
    std::printf("  %-22s no GPS: max err %5.2f deg, end err %5.2f deg (truth %5.2f)\n", name, no_gps.max_err_deg,
                no_gps.end_err_deg, no_gps.end_truth_deg);
    std::printf("  %-22s   GPS : max err %5.2f deg, end err %5.2f deg (truth %5.2f)\n", "", gps.max_err_deg,
                gps.end_err_deg, gps.end_truth_deg);
}

} // namespace

int main() {
    std::printf("== SITL GPS-aided attitude ==\n");

    // A. Held 15 deg bank: 0.5 stick of a 30 deg max angle.
    const Result a0 = fly(false, 0.5f, 0.0f, 0.0f, 4.0);
    const Result a1 = fly(true, 0.5f, 0.0f, 0.0f, 4.0);
    report("A. 15 deg bank, 4 s", a0, a1);

    // B. Forward pitch with constant yaw rate: a sustained turn.
    const Result b0 = fly(false, 0.0f, -0.4f, 0.5f, 6.0);
    const Result b1 = fly(true, 0.0f, -0.4f, 0.5f, 6.0);
    report("B. 12 deg pitch + yaw", b0, b1);

    CHECK(a0.flying && a1.flying && b0.flying && b1.flying);
    // Without GPS the estimate really does drift (proves the test exercises the problem).
    CHECK(a0.end_err_deg > 2.0);
    // With GPS the estimate stays on the truth.
    CHECK(a1.max_err_deg < 1.5);
    CHECK(a1.end_err_deg < 1.0);
    CHECK(b1.max_err_deg < 1.5);
    CHECK(b1.end_err_deg < 1.0);
    CHECK(a1.max_err_deg < 0.5 * a0.max_err_deg);
    CHECK(b1.max_err_deg < 0.5 * b0.max_err_deg);
    // The pilot gets the bank angle they asked for.
    CHECK_NEAR(a1.end_truth_deg, 15.0, 1.5);

    std::printf("  hover err before manoeuvre: no GPS %.2f deg, GPS %.2f deg\n", a0.hover_err_deg, a1.hover_err_deg);
    CHECK(a1.hover_err_deg < 1.0);

    // C. Robustness of the turn (B) to the receiver: latency the fusion doesn't know
    //    about (it assumes 100 ms), 5 Hz and 1 Hz update rates.
    struct Case { const char* name; GpsSetup g; double max_tol; };
    const Case cases[] = {
        {"latency 0 ms", {100, 0, -1}, 2.0},
        {"latency 200 ms", {100, 200, -1}, 2.0},
        {"5 Hz GPS", {200, 100, -1}, 1.5},
        {"1 Hz GPS", {1000, 100, -1}, 2.5},
    };
    for (const Case& c : cases) {
        const Result x = fly(true, 0.0f, -0.4f, 0.5f, 6.0, c.g);
        std::printf("  C. turn, %-15s max err %5.2f deg, end err %5.2f deg\n", c.name, x.max_err_deg, x.end_err_deg);
        CHECK(x.flying);
        CHECK(x.max_err_deg < c.max_tol);
    }

    // D. GPS lost 2 s into the held bank: coast on the gyro, keep flying.
    {
        GpsSetup g;
        g.dropout_at_s = 2.0;
        const Result x = fly(true, 0.5f, 0.0f, 0.0f, 4.0, g);
        std::printf("  D. bank, GPS lost after 2 s: end-of-bank err %.2f deg, worst %.2f deg (incl. 3 s after)\n",
                    x.end_err_deg, x.max_err_deg);
        CHECK(x.flying);
        CHECK(x.end_err_deg < 1.5);
    }

    return test::finish("test_sitl_gps_attitude");
}
