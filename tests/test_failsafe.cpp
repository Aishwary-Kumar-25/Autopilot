#include "control/failsafe.hpp"
#include "math/pid.hpp"
#include "tests/test_common.hpp"
#include <cstdio>

using namespace autopilot;
using control::FailsafeAction;

static control::FailsafeInputs ok_inputs(uint32_t now) {
    control::FailsafeInputs in;
    in.now_ms = now;
    in.last_rc_ms = now;
    in.rc_ever_received = true;
    in.battery_voltage = 16.0f;
    in.attitude_valid = true;
    in.imu_usable = true;
    return in;
}

int main() {
    std::printf("== Failsafe + PID ==\n");
    control::Failsafe::Config cfg;
    cfg.rc_timeout_ms = 500;
    cfg.battery_low_v = 14.0f;
    cfg.battery_debounce_ms = 2000;
    control::Failsafe fs(cfg);

    // Nominal
    CHECK(fs.update(ok_inputs(1000)) == FailsafeAction::NONE);

    // RC loss -> LAND, RC back -> NONE (pure RC loss is recoverable)
    auto in = ok_inputs(2000);
    in.last_rc_ms = 1400;
    CHECK(fs.update(in) == FailsafeAction::LAND);
    CHECK(fs.reasons() & control::FS_RC_LOSS);
    CHECK(fs.update(ok_inputs(2100)) == FailsafeAction::NONE);

    // Battery sag shorter than the debounce does nothing; sustained sag latches LAND
    in = ok_inputs(3000); in.battery_voltage = 13.0f;
    CHECK(fs.update(in) == FailsafeAction::NONE);
    in = ok_inputs(4000); in.battery_voltage = 14.5f;   // recovered: debounce restarts
    CHECK(fs.update(in) == FailsafeAction::NONE);
    in = ok_inputs(5000); in.battery_voltage = 13.0f;
    CHECK(fs.update(in) == FailsafeAction::NONE);
    in = ok_inputs(7100); in.battery_voltage = 13.0f;
    CHECK(fs.update(in) == FailsafeAction::LAND);
    in = ok_inputs(7200); in.battery_voltage = 15.0f;   // voltage recovers when load drops
    CHECK(fs.update(in) == FailsafeAction::LAND);       // ...but the landing is committed

    // No IMU -> TERMINATE, never relaxes until reset
    fs.reset();
    in = ok_inputs(8000); in.imu_usable = false;
    CHECK(fs.update(in) == FailsafeAction::TERMINATE);
    CHECK(fs.update(ok_inputs(8100)) == FailsafeAction::TERMINATE);
    fs.reset();
    CHECK(fs.update(ok_inputs(8200)) == FailsafeAction::NONE);

    // Never heard RC at all
    in = ok_inputs(9000); in.rc_ever_received = false;
    CHECK(fs.update(in) == FailsafeAction::LAND);

    // ---- PID ----
    math::PID pid;
    pid.set_gains(1.0f, 0.0f, 0.1f);
    pid.set_limits(-10.0f, 10.0f, 0.0f);
    pid.update(0.0f, 0.0f, 0.01f);
    // Setpoint step with steady measurement: derivative-on-measurement => no kick
    const float out = pid.update(5.0f, 0.0f, 0.01f);
    CHECK_NEAR(out, 5.0f, 1e-4);

    // imax = 0 disables the integrator entirely
    math::PID pi0;
    pi0.set_gains(0.0f, 10.0f, 0.0f);
    pi0.set_limits(-1.0f, 1.0f, 0.0f);
    for (int i = 0; i < 100; ++i) pi0.update(1.0f, 0.0f, 0.01f);
    CHECK_NEAR(pi0.get_integrator(), 0.0f, 1e-6);

    // Integrator bounded by imax, frozen when integrate=false, cleared by reset()
    math::PID pi;
    pi.set_gains(0.0f, 1.0f, 0.0f);
    pi.set_limits(-1.0f, 1.0f, 0.3f);
    for (int i = 0; i < 1000; ++i) pi.update(1.0f, 0.0f, 0.01f);
    CHECK_NEAR(pi.get_integrator(), 0.3f, 1e-5);
    pi.reset();
    for (int i = 0; i < 100; ++i) pi.update(1.0f, 0.0f, 0.01f, false);
    CHECK_NEAR(pi.get_integrator(), 0.0f, 1e-6);

    // Anti-windup: no integration into a saturated output
    math::PID sat;
    sat.set_gains(2.0f, 1.0f, 0.0f);
    sat.set_limits(-1.0f, 1.0f, 5.0f);
    for (int i = 0; i < 500; ++i) sat.update(1.0f, 0.0f, 0.01f); // P alone saturates
    CHECK_NEAR(sat.get_integrator(), 0.0f, 1e-6);

    return test::finish("test_failsafe");
}
