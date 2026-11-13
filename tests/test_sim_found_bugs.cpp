/**
 * Regression tests for flight-code bugs found by the ROS 2 simulator scenarios
 * (ros2/autopilot_sim/scenarios). Each one failed before its fix.
 *
 *   1. IMU0 returns NaN in a hover: the gyro consistency check compared the good
 *      IMU against the NaN one, marked it DEGRADED in the same tick, the
 *      failsafe saw "no usable IMU" and cut the motors at 5 m (quad_imu_failover,
 *      kind: nan). Now the quad keeps flying on IMU1.
 *   2. Baro dies in ALT_HOLD: the fallback used the raw stick (mid = 50 %
 *      collective) and the quad climbed away (quad_baro_failure). Now mid stick
 *      keeps the collective that was holding altitude. Still open loop, so it
 *      drifts slowly; the full fix is a second height source in the estimator.
 *   3. Rover in HEADING_HOLD ignored the steering stick (rover_drive).
 *   4. IMU0 stops sending while the quad is manoeuvring (found with Gazebo, where the
 *      airframe wobbles more): before IMU0's 50 ms timeout, IMU1 was compared with
 *      IMU0's frozen reading, marked DEGRADED, and the failover found no usable
 *      IMU, so the failsafe cut the motors. Now a stale IMU is left out.
 */
#include "tests/sitl_rig.hpp"
#include "tests/sitl_vehicle_rig.hpp"
#include "tests/test_common.hpp"
#include "vehicles/ugv/ugv_vehicle.hpp"
#include <cstdio>

using namespace autopilot;
using namespace autopilot::sitl;

static void climb_and_hold(Rig& r) {
    r.uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
    r.run(1.0);
    std::string err;
    CHECK(r.vm->arm(err));
    r.rc[3] = 0.8f;
    r.run(2.5);
    r.rc[3] = 0.5f;
    r.run(5.0);
}

int main() {
    std::printf("== bugs found by the simulator scenarios ==\n");
    {
        Rig r;
        climb_and_hold(r);
        const double alt0 = r.quad.altitude();
        r.imu0->set_fault(sim::Fault::NOT_A_NUMBER);
        r.run(3.0);
        auto* sm = r.vm->get_sensors();
        std::printf("  IMU0 NaN: armed %d, active %s, alt %.2f -> %.2f m\n", r.vm->is_armed(),
                    sm->get_active_imu() ? sm->get_active_imu()->name() : "none", alt0, r.quad.altitude());
        CHECK(r.vm->is_armed());
        CHECK(sm->get_active_imu() == r.imu1.get());
        CHECK(!r.quad.on_ground);
        CHECK_NEAR(r.quad.altitude(), alt0, 0.6);
    }
    {
        Rig r;
        climb_and_hold(r);
        r.imu0->set_fault(sim::Fault::DROPOUT);
        // Live gyro moves away from IMU0's frozen reading for 40 ms (in Gazebo the frame's
        // vibration did this; here a short 1 rad/s burst on IMU1 stands in for it).
        r.imu1->gyro_bias = math::Vector3f(1.0f, 0.0f, 0.0f);
        r.run(0.04);
        r.imu1->gyro_bias = math::Vector3f(0.0f, 0.0f, 0.0f);
        r.run(2.0);
        auto* sm = r.vm->get_sensors();
        std::printf("  IMU0 dropout while IMU1 shakes: armed %d, active %s, alt %.2f m\n", r.vm->is_armed(),
                    sm->get_active_imu() ? sm->get_active_imu()->name() : "none", r.quad.altitude());
        CHECK(r.vm->is_armed());
        CHECK(sm->get_active_imu() == r.imu1.get());
        CHECK(!r.quad.on_ground);
    }
    {
        Rig r;
        climb_and_hold(r);
        const double alt0 = r.quad.altitude();
        r.baro->set_fault(sim::Fault::DROPOUT);
        r.run(20.0);
        std::printf("  baro dead in ALT_HOLD: alt %.2f -> %.2f m after 20 s (touchdown %.2f m/s)\n", alt0,
                    r.quad.altitude(), r.quad.max_touchdown_speed);
        // Open-loop collective can't hold altitude exactly (that needs another height
        // source, e.g. GPS altitude in the estimator), but it must not fly away
        // (it was climbing past +6 m and accelerating) and any touchdown must be soft.
        CHECK(r.quad.altitude() < alt0 + 3.0);
        CHECK(r.quad.max_touchdown_speed < 1.5);
    }
    {
        VehicleRig r(vehicles::VehicleType::UGV_GROUND_ROVER);
        auto* ugv = r.vehicle<vehicles::UGVVehicle>();
        CHECK(ugv != nullptr);
        if (ugv) {
            ugv->set_mode(vehicles::UGVMode::HEADING_HOLD);
            r.run(1.0);
            CHECK(r.arm());
            r.rc[3] = r.rc[4] = 0.5f;   // drive
            r.run(3.0);
            const double yaw0 = r.body().yaw();
            r.rc[2] = 0.3f;             // steer right
            r.run(1.5);
            r.rc[2] = 0.0f;
            const double turned = VehicleRig::wrap(r.body().yaw() - yaw0) * VehicleRig::RAD2DEG;
            r.run(0.5);
            const double yaw1 = r.body().yaw();
            r.run(3.0);
            const double drift = VehicleRig::wrap(r.body().yaw() - yaw1) * VehicleRig::RAD2DEG;
            std::printf("  rover HEADING_HOLD: stick turned it %.1f deg, then held (drift %.1f deg)\n", turned, drift);
            CHECK(turned > 20.0);
            CHECK(std::fabs(drift) < 5.0);
        }
    }
    return test::finish("test_sim_found_bugs");
}
