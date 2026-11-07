/**
 * Regression tests for vehicle-level safety fixes:
 *  - arming refused with the throttle / drive sticks up (UAV, UGV, ROV)
 *  - controllers don't run (or wind up) while disarmed
 *  - switching to ALT_HOLD in flight holds the current altitude, not a stale target
 *  - barometer failover doesn't step the altitude estimate
 *  - UGV heading hold has no derivative kick on a new target heading
 */
#include "hal/posix/posix_hal.hpp"
#include "vehicles/vehicle_manager.hpp"
#include "tests/fake_sensors.hpp"
#include "tests/test_common.hpp"
#include <cstdio>
#include <memory>

using namespace autopilot;

struct Bench {
    hal::PosixHAL hal;
    std::unique_ptr<vehicles::VehicleManager> vm;
    std::shared_ptr<test::FakeIMU> imu;
    std::shared_ptr<test::FakeBaro> baro;

    explicit Bench(vehicles::VehicleType type) {
        hal.time().set_manual_time_mode(true);
        hal.time().advance_time_ms(1000);
        vm = std::make_unique<vehicles::VehicleManager>(&hal, type);
        vm->init();
        imu = std::make_shared<test::FakeIMU>(&hal.time());
        baro = std::make_shared<test::FakeBaro>(&hal.time());
        imu->init();
        baro->init();
        vm->get_sensors()->register_sensor(imu);
        vm->get_sensors()->register_sensor(baro);
    }
    // RC link: the last stick values are re-sent every step, like a live receiver.
    float rc[6]{0, 0, 0, 0, 0, 0};
    void sticks(float r, float p, float y, float t, float f = 0.0f, float l = 0.0f) {
        rc[0] = r; rc[1] = p; rc[2] = y; rc[3] = t; rc[4] = f; rc[5] = l;
        vm->set_user_inputs(r, p, y, t, f, l);
    }
    void run(float seconds) {
        const int n = static_cast<int>(seconds * 100.0f + 0.5f);
        for (int i = 0; i < n; ++i) {
            hal.time().advance_time_ms(10);
            vm->set_user_inputs(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5]);
            vm->step(0.01f);
        }
    }
};

static void test_arming_sticks() {
    std::printf("[arming requires safe sticks]\n");
    std::string why;
    {
        Bench b(vehicles::VehicleType::UAV_MULTICOPTER);
        b.run(0.5f);
        auto* uav = static_cast<vehicles::UAVVehicle*>(b.vm->get_active_vehicle());
        uav->set_mode(vehicles::UAVFlightMode::STABILIZE);
        b.sticks(0, 0, 0, 0.6f);
        CHECK(!b.vm->arm(why));
        CHECK(why.find("throttle") != std::string::npos);
        b.sticks(0, 0, 0, 0.0f);
        CHECK(b.vm->arm(why));
        b.vm->disarm();

        uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
        b.sticks(0, 0, 0, 0.5f);   // mid stick holds: OK
        CHECK(b.vm->arm(why));
        b.vm->disarm();
        b.sticks(0, 0, 0, 0.9f);   // would climb straight away
        CHECK(!b.vm->arm(why));
    }
    {
        Bench b(vehicles::VehicleType::UGV_GROUND_ROVER);
        b.run(0.5f);
        b.sticks(0, 0, 0, 0.0f, 0.5f);
        CHECK(!b.vm->arm(why));
        b.sticks(0, 0, 0, 0.0f, 0.0f);
        CHECK(b.vm->arm(why));
    }
    {
        Bench b(vehicles::VehicleType::ROV_SUBMARINE);
        b.run(0.5f);
        b.sticks(0, 0, 0, 0.0f, 0.0f, 0.5f);
        CHECK(!b.vm->arm(why));
        b.sticks(0, 0, 0, 0.0f, 0.0f, 0.0f);
        CHECK(b.vm->arm(why));
    }
}

static void test_no_control_while_disarmed() {
    std::printf("[no controller output / windup while disarmed]\n");
    Bench b(vehicles::VehicleType::UGV_GROUND_ROVER);
    b.run(0.5f);
    auto* ugv = static_cast<vehicles::UGVVehicle*>(b.vm->get_active_vehicle());
    ugv->set_mode(vehicles::UGVMode::HEADING_HOLD);
    ugv->set_target_heading(b.vm->get_latest_state().heading_rad + 1.5f);
    b.sticks(0, 0, 0, 0, 0);
    b.run(5.0f);  // 5 s disarmed with a 1.5 rad heading error
    const auto& d = b.vm->get_active_vehicle()->get_demands();
    CHECK(d.yaw == 0.0f && d.forward == 0.0f && d.throttle == 0.0f);

    // After arming, the first demand is the plain P response: no integrator that
    // wound up on the bench (kp 1.2 * 1.5 rad saturates at 1.0 either way, so use a small error).
    ugv->set_target_heading(b.vm->get_latest_state().heading_rad + 0.2f);
    b.run(5.0f);
    std::string why;
    CHECK(b.vm->arm(why));
    b.run(0.01f);
    CHECK_NEAR(b.vm->get_active_vehicle()->get_demands().yaw, 1.2f * 0.2f, 0.03f);
}

static void test_ugv_no_heading_kick() {
    std::printf("[UGV heading hold: no derivative kick on a new target]\n");
    Bench b(vehicles::VehicleType::UGV_GROUND_ROVER);
    b.run(0.5f);
    auto* ugv = static_cast<vehicles::UGVVehicle*>(b.vm->get_active_vehicle());
    b.sticks(0, 0, 0, 0, 0);
    std::string why;
    CHECK(b.vm->arm(why));
    ugv->set_mode(vehicles::UGVMode::HEADING_HOLD);
    const float h = b.vm->get_latest_state().heading_rad;
    ugv->set_target_heading(h + 0.1f);
    b.run(0.2f);
    const float before = b.vm->get_active_vehicle()->get_demands().yaw;
    ugv->set_target_heading(h + 0.2f);
    b.run(0.01f);
    const float after = b.vm->get_active_vehicle()->get_demands().yaw;
    // P step is 1.2 * 0.1 = 0.12; the old D-on-error code added ~0.5 here.
    CHECK(after - before > 0.08f && after - before < 0.16f);
}

static void test_alt_hold_captures_current_altitude() {
    std::printf("[ALT_HOLD entered in flight holds the current altitude]\n");
    Bench b(vehicles::VehicleType::UAV_MULTICOPTER);
    auto* uav = static_cast<vehicles::UAVVehicle*>(b.vm->get_active_vehicle());
    uav->set_mode(vehicles::UAVFlightMode::STABILIZE);
    b.run(1.0f);
    b.sticks(0, 0, 0, 0.0f);
    std::string why;
    CHECK(b.vm->arm(why));
    CHECK_NEAR(uav->get_target_altitude(), 0.0f, 0.2f);

    // "Fly" up to 12 m on manual collective (the fake baro reports the altitude).
    b.sticks(0, 0, 0, 0.5f);
    for (int i = 1; i <= 120; ++i) {
        b.baro->altitude = 0.1f * i;
        b.run(0.05f);
    }
    b.run(4.0f);
    CHECK_NEAR(b.vm->get_latest_state().altitude_m, 12.0f, 0.3f);

    uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
    CHECK_NEAR(uav->get_target_altitude(), 12.0f, 0.3f);
    // Seeded from the steady manual collective (level, so 0.5).
    CHECK_NEAR(uav->get_hover_throttle(), 0.5f, 0.01f);
}

static void test_baro_failover_is_continuous() {
    std::printf("[barometer failover keeps the altitude continuous]\n");
    Bench b(vehicles::VehicleType::UAV_MULTICOPTER);
    auto baro2 = std::make_shared<test::FakeBaro>(&b.hal.time(), "FakeBaro2");
    baro2->init();
    baro2->altitude = 7.0f;   // its own ground reference: reads 7 m higher
    b.vm->get_sensors()->register_sensor(baro2);
    b.run(5.0f);
    CHECK(b.vm->get_sensors()->get_active_baro() == b.baro.get());
    CHECK_NEAR(b.vm->get_latest_state().altitude_m, 0.0f, 0.1f);

    b.baro->fail = true;
    float worst = 0.0f;
    for (int i = 0; i < 300; ++i) {
        b.run(0.01f);
        worst = std::fmax(worst, std::fabs(b.vm->get_latest_state().altitude_m));
    }
    CHECK(b.vm->get_sensors()->get_active_baro() == baro2.get());
    std::printf("  worst altitude excursion after failover: %.3f m\n", worst);
    CHECK(worst < 0.3f);
}

int main() {
    test_arming_sticks();
    test_no_control_while_disarmed();
    test_ugv_no_heading_kick();
    test_alt_hold_captures_current_altitude();
    test_baro_failover_is_continuous();
    return test::finish("test_vehicle_safety");
}
