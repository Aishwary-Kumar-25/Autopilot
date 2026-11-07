/**
 * Closed-loop SITL flight test: the real flight stack (VehicleManager, shared
 * SensorManager + SensorFusion, UAV controller, failsafe, Quad-X mixer) flies
 * the simulated quad in sim/quad_sim.hpp.
 *
 * Scenario
 *   1. arm on the ground in ALT_HOLD, climb on the stick, hold altitude
 *   2. roll step: attitude tracks the stick
 *   3. IMU0 freezes mid-flight -> sensor manager fails it over to IMU1, keeps flying
 *   4. RC link lost -> failsafe lands -> land detector disarms on the ground
 */
#include "hal/posix/posix_hal.hpp"
#include "sim/quad_sim.hpp"
#include "tests/test_common.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/vehicle_manager.hpp"
#include <cstdio>
#include <memory>

using namespace autopilot;

namespace {
constexpr double RAD2DEG = 57.29577951;
constexpr uint32_t LOOP_MS = 4;     // 250 Hz flight loop
constexpr int PHYSICS_SUBSTEPS = 4; // 1 kHz physics

struct Rig {
    hal::PosixHAL hal;
    sim::QuadModel quad;
    sim::SimContext ctx;
    std::shared_ptr<sim::SimIMU> imu0, imu1;
    std::unique_ptr<vehicles::VehicleManager> vm;
    vehicles::UAVVehicle* uav{nullptr};
    double t{0.0};
    bool rc_on{true};
    float rc[4]{0, 0, 0, 0.5f};   // roll, pitch, yaw, throttle sticks

    Rig() {
        hal.time().set_manual_time_mode(true);
        hal.time().advance_time_ms(1000);
        ctx.model = &quad;
        ctx.time = &hal.time();
        vm = std::make_unique<vehicles::VehicleManager>(&hal, vehicles::VehicleType::UAV_MULTICOPTER);
        vm->init();
        imu0 = std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU0");
        imu1 = std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU1");
        vm->get_sensors()->register_sensor(imu0);
        vm->get_sensors()->register_sensor(imu1);
        vm->get_sensors()->register_sensor(std::make_shared<sim::SimBaro>(&ctx));
        vm->get_sensors()->register_sensor(std::make_shared<sim::SimCompass>(&ctx));
        uav = dynamic_cast<vehicles::UAVVehicle*>(vm->get_active_vehicle());
    }

    void step() {
        if (rc_on) vm->set_user_inputs(rc[0], rc[1], rc[2], rc[3]);
        vm->step(LOOP_MS * 1e-3f);
        double cmd[4];
        for (int i = 0; i < 4; ++i) {
            const uint16_t us = hal.pwm().is_enabled() ? hal.pwm().get_channel(i) : 1000;
            cmd[i] = (static_cast<double>(us) - 1000.0) / 1000.0;
        }
        for (int s = 0; s < PHYSICS_SUBSTEPS; ++s) quad.step(cmd, LOOP_MS * 1e-3 / PHYSICS_SUBSTEPS);
        hal.time().advance_time_ms(LOOP_MS);
        t += LOOP_MS * 1e-3;
    }
    void run(double seconds) {
        const double end = t + seconds;
        while (t < end) step();
    }
    double est_err_deg() const {
        const auto& s = vm->get_latest_state();
        return std::fmax(std::fabs(s.euler_rad.x - quad.roll()), std::fabs(s.euler_rad.y - quad.pitch())) * RAD2DEG;
    }
};
} // namespace

int main() {
    std::printf("== SITL closed-loop flight ==\n");
    Rig r;
    CHECK(r.uav != nullptr);
    if (!r.uav) return test::finish("test_sitl_flight");
    r.uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);

    // Let the estimator settle on the ground, then arm with the stick centred.
    r.run(1.0);
    std::string err;
    CHECK(r.vm->arm(err));
    if (!err.empty()) std::printf("  arm: %s\n", err.c_str());
    r.run(1.0);
    CHECK(r.quad.on_ground);           // mid stick on the ground = stay put
    CHECK_NEAR(r.quad.altitude(), 0.0, 0.01);

    // 1. Climb on the stick for 2.5 s, then centre the stick and hold.
    r.rc[3] = 0.8f;
    r.run(2.5);
    r.rc[3] = 0.5f;
    r.run(1.5);
    const double hold_alt = r.quad.altitude();
    r.run(5.0);
    std::printf("  climb+hold: alt %.2f m (held from %.2f), vz %.2f, hover thr %.3f, est err %.2f deg\n",
                r.quad.altitude(), hold_alt, -r.quad.vel.z, r.uav->get_hover_throttle(), r.est_err_deg());
    CHECK(!r.quad.on_ground);
    CHECK(r.quad.altitude() > 1.5 && r.quad.altitude() < 4.0);
    CHECK_NEAR(r.quad.altitude(), hold_alt, 0.5);
    CHECK(std::fabs(r.quad.vel.z) < 0.25);
    CHECK(std::fabs(r.quad.roll()) * RAD2DEG < 3.0);
    CHECK(std::fabs(r.quad.pitch()) * RAD2DEG < 3.0);
    CHECK(r.est_err_deg() < 2.0);
    CHECK_NEAR(r.uav->get_hover_throttle(), 0.49, 0.06); // learned ~ m g / (4 Tmax)

    // 2. Roll step: 0.3 stick = 9 deg
    r.rc[0] = 0.3f;
    r.run(1.0);
    const double roll_deg = r.quad.roll() * RAD2DEG;
    std::printf("  roll step: target 9.0 deg, got %.2f deg (estimate %.2f), alt %.2f m\n", roll_deg,
                r.vm->get_latest_state().euler_rad.x * RAD2DEG, r.quad.altitude());
    // The controller tracks the ESTIMATE tightly. Truth runs a few degrees higher:
    // without GPS, a multirotor accelerometer can't see sustained tilt (thrust is
    // along body Z), so the low-gain accel correction drifts slowly toward level.
    CHECK_NEAR(r.vm->get_latest_state().euler_rad.x * RAD2DEG, 9.0, 1.5);
    CHECK_NEAR(roll_deg, 9.0, 4.0);
    r.rc[0] = 0.0f;
    r.run(2.0);
    CHECK(std::fabs(r.quad.roll()) * RAD2DEG < 3.0);
    CHECK(r.quad.altitude() > 1.0);

    // 3. Primary IMU freezes (chip hung, still ACKs on the bus).
    auto* sm = r.vm->get_sensors();
    CHECK(sm->get_active_imu() == r.imu0.get());
    const uint32_t failovers_before = sm->failover_count();
    r.imu0->set_fault(sim::Fault::STUCK);
    r.run(3.0);
    std::printf("  IMU0 stuck: active=%s failovers=%u alt %.2f m roll %.2f deg est err %.2f deg\n",
                sm->get_active_imu() ? sm->get_active_imu()->name() : "none",
                static_cast<unsigned>(sm->failover_count()), r.quad.altitude(), r.quad.roll() * RAD2DEG,
                r.est_err_deg());
    CHECK(sm->get_active_imu() == r.imu1.get());
    CHECK(sm->failover_count() > failovers_before);
    CHECK(r.vm->is_armed());
    CHECK(!r.quad.on_ground);
    CHECK(std::fabs(r.quad.roll()) * RAD2DEG < 5.0);
    CHECK(r.est_err_deg() < 3.0);

    // 4. RC link lost -> failsafe LAND -> disarm on touchdown.
    r.rc_on = false;
    r.run(0.4);
    CHECK(r.vm->is_armed());       // still within the 500 ms RC timeout
    r.run(0.4);
    CHECK(r.vm->get_active_vehicle()->get_failsafe().action() == control::FailsafeAction::LAND);
    double t_land = 0.0;
    while (r.vm->is_armed() && t_land < 20.0) {
        r.step();
        t_land += LOOP_MS * 1e-3;
    }
    std::printf("  RC loss: disarmed after %.1f s, alt %.2f m, touchdown %.2f m/s\n", t_land, r.quad.altitude(),
                r.quad.max_touchdown_speed);
    CHECK(!r.vm->is_armed());
    CHECK(r.quad.on_ground);
    CHECK(r.quad.max_touchdown_speed < 1.0);
    CHECK(!r.hal.pwm().is_enabled());

    return test::finish("test_sitl_flight");
}
