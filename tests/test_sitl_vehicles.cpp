/**
 * One firmware, every vehicle: closed-loop SITL of the rover, the boat and the
 * ROV on the SAME VehicleManager / SensorManager / SensorFusion, switching the
 * vehicle type at runtime the way a ground station does (VEHICLE_TYPE).
 *
 *   1. rover:  heading hold straight, 90 deg turn, speed hold; tilt estimate stays
 *              right through the turns (centripetal accel is not tilt)
 *   2. switch to boat at the same spot: heading hold in a cross current, turn,
 *              speed hold (twin-thruster differential steering)
 *   3. switch to ROV: dive in DEPTH_HOLD, hold depth, GPS lost underwater, yaw
 *              without sideways drift, surge straight
 *   4. ROV with 4 thrusters: roll stick rolls the right way
 *   5. switch back to the multicopter: take off and hold altitude
 *   6. switching while armed is refused (physics untouched)
 *   7. thruster/steering geometry: each mixer axis produces only that motion
 */
#include "test_common.hpp"
#include "sitl_vehicle_rig.hpp"
#include <cstdio>

using namespace autopilot;
using vehicles::VehicleType;
using sitl::VehicleRig;

namespace {

/** Wrench (surge, sway, heave-up, roll, pitch, yaw) the sub's real geometry makes for a demand. */
struct Wrench { double fx, fy, up, l, m, n; };
Wrench sub_wrench(int thrusters, const control::ControlDemands& d) {
    control::ActuatorMatrixMixer mixer;
    mixer.configure_rov(static_cast<uint8_t>(thrusters));
    float out[hal::PWMOutput::MAX_CHANNELS]{};
    mixer.get_normalized_outputs(d, out, hal::PWMOutput::MAX_CHANNELS);
    sim::SubModel sub(thrusters);
    Wrench w{};
    for (size_t i = 0; i < sub.thrusters.size(); ++i) {
        const sim::V3 f = sub.thrusters[i].dir * out[i];
        const sim::V3 tq = sub.thrusters[i].pos.cross(f);
        w.fx += f.x; w.fy += f.y; w.up -= f.z;
        w.l += tq.x; w.m += tq.y; w.n += tq.z;
    }
    return w;
}

void test_geometry() {
    std::printf("[7] mixer rows vs physical thruster geometry\n");
    for (int n : {3, 4, 6, 8}) {
        control::ControlDemands d;
        d.yaw = 0.5f;                                   // pure yaw: no sideways push
        Wrench w = sub_wrench(n, d);
        CHECK(w.n > 0.05);
        CHECK(std::fabs(w.fy) < 1e-4 && std::fabs(w.fx) < 1e-4);
        d = {}; d.forward = 0.5f;                       // pure surge: no turn
        w = sub_wrench(n, d);
        CHECK(w.fx > 0.3);
        CHECK(std::fabs(w.n) < 1e-4 && std::fabs(w.fy) < 1e-4);
        d = {}; d.throttle = 0.5f;                      // heave up
        w = sub_wrench(n, d);
        CHECK(w.up > 0.3);
        CHECK(std::fabs(w.l) < 1e-4 && std::fabs(w.m) < 1e-4);
        if (n >= 6) {
            d = {}; d.lateral = 0.5f;                   // sway right, no yaw
            w = sub_wrench(n, d);
            CHECK(w.fy > 0.3);
            CHECK(std::fabs(w.n) < 1e-4 && std::fabs(w.fx) < 1e-4);
        }
        if (n >= 4) {
            d = {}; d.roll = 0.5f;                      // +roll = right side down
            w = sub_wrench(n, d);
            CHECK(w.l > 0.01);
            CHECK(std::fabs(w.up) < 1e-4);
        }
        if (n == 8) {
            d = {}; d.pitch = 0.5f;                     // +pitch = nose up
            w = sub_wrench(n, d);
            CHECK(w.m > 0.01);
        }
    }
    // Skid steer: yaw right (+) must drive the left side faster.
    hal::PosixPWMOutput pwm;
    pwm.init(50);
    pwm.enable_output(true);
    vehicles::UGVMixer skid(vehicles::UGVSteeringType::SKID_STEER);
    control::ControlDemands d;
    d.forward = 0.3f;
    d.yaw = 0.3f;
    skid.output_to_actuators(d, &pwm, true);
    CHECK(pwm.get_channel(0) > pwm.get_channel(1));
}

} // namespace

int main() {
    test_geometry();

    VehicleRig r(VehicleType::UGV_GROUND_ROVER);
    auto* const sensors = r.vm->get_sensors();
    auto* const fusion = r.vm->get_fusion();
    CHECK(std::string(r.body().name()) == "rover");

    // ------------------------------------------------------------ 1. rover
    std::printf("[1] rover\n");
    r.run(2.0);                                   // estimator settles at rest
    CHECK(r.arm());
    auto* ugv = r.vehicle<vehicles::UGVVehicle>();
    CHECK(ugv != nullptr);
    ugv->set_mode(vehicles::UGVMode::HEADING_HOLD);
    const double yaw0 = r.yaw_deg();
    r.rc[4] = 0.5f;                               // half throttle
    r.run(2.0);
    // Pulling away (2.5 m/s^2) briefly reads as nose-up until GPS-aided attitude
    // takes over (~0.5 s): the "acceleration/drag as tilt without GPS" limit.
    std::printf("  launch: worst tilt estimate error %.1f deg\n", r.max_tilt_err_deg);
    CHECK(r.max_tilt_err_deg < 6.0);
    r.max_tilt_err_deg = 0.0;
    r.run(6.0);
    const double dist = std::hypot(r.body().pos.x, r.body().pos.y);
    std::printf("  straight: %.1f m, heading drift %.1f deg, speed %.2f m/s\n", dist,
                VehicleRig::wrap((r.yaw_deg() - yaw0) / VehicleRig::RAD2DEG) * VehicleRig::RAD2DEG, r.body().ground_speed());
    CHECK(dist > 10.0);
    CHECK(std::fabs(VehicleRig::wrap((r.yaw_deg() - yaw0) / VehicleRig::RAD2DEG)) * VehicleRig::RAD2DEG < 3.0);

    ugv->set_target_heading(static_cast<float>(VehicleRig::wrap(r.body().yaw() + M_PI / 2)));
    r.run(6.0);
    double turned = VehicleRig::wrap(r.body().yaw() - yaw0 / VehicleRig::RAD2DEG) * VehicleRig::RAD2DEG;
    std::printf("  turn to +90: now %.1f deg from start, heading est err %.1f deg\n", turned, r.heading_err_deg());
    CHECK_NEAR(turned, 90.0, 5.0);
    CHECK(r.heading_err_deg() < 5.0);

    ugv->set_mode(vehicles::UGVMode::SPEED_HOLD);
    ugv->set_target_speed(2.0f);
    r.run(10.0);
    std::printf("  speed hold 2.0: %.2f m/s; worst tilt estimate error since launch %.1f deg\n",
                r.body().ground_speed(), r.max_tilt_err_deg);
    CHECK_NEAR(r.body().ground_speed(), 2.0, 0.3);
    CHECK(r.max_tilt_err_deg < 3.0);              // turning at speed is not read as tilt

    // stop (speed hold to 0 brakes with the motor), then disarm
    ugv->set_target_speed(0.0f);
    r.neutral();
    r.run(4.0);
    r.vm->disarm();
    r.run(0.5);
    CHECK(r.body().ground_speed() < 0.05);

    // ------------------------------------------------------------ 2. boat
    std::printf("[2] switch to boat\n");
    const sim::V3 where = r.body().pos;
    const double yaw_before = r.body().yaw();
    CHECK(r.switch_to(VehicleType::USV_SURFACE_BOAT));
    CHECK(std::string(r.body().name()) == "boat");
    CHECK(r.vm->get_sensors() == sensors && r.vm->get_fusion() == fusion);   // same stack
    CHECK(std::hypot(r.body().pos.x - where.x, r.body().pos.y - where.y) < 1e-9);
    CHECK(std::fabs(r.body().yaw() - yaw_before) < 1e-9);
    auto* boat = r.world->as<sim::BoatModel>();
    boat->current = sim::V3(0.0, 0.3, 0.0);       // 0.3 m/s current toward the east
    auto* usv = r.vehicle<vehicles::USVVehicle>();
    CHECK(usv != nullptr);
    r.max_tilt_err_deg = 0.0;
    r.run(1.0);
    CHECK(r.arm());
    usv->set_mode(vehicles::UGVMode::HEADING_HOLD);
    const double byaw0 = r.body().yaw();
    r.rc[4] = 0.6f;
    r.run(10.0);
    double drift = VehicleRig::wrap(r.body().yaw() - byaw0) * VehicleRig::RAD2DEG;
    std::printf("  heading hold in current: drift %.1f deg, speed %.2f m/s\n", drift, r.body().ground_speed());
    CHECK(std::fabs(drift) < 4.0);
    CHECK(r.body().ground_speed() > 1.0);

    usv->set_target_heading(static_cast<float>(VehicleRig::wrap(byaw0 - M_PI / 2)));   // turn to port
    r.run(8.0);
    turned = VehicleRig::wrap(r.body().yaw() - byaw0) * VehicleRig::RAD2DEG;
    std::printf("  turn to -90: %.1f deg\n", turned);
    CHECK_NEAR(turned, -90.0, 6.0);

    usv->set_mode(vehicles::UGVMode::SPEED_HOLD);
    usv->set_target_speed(1.5f);
    r.run(12.0);
    const auto& st = r.vm->get_latest_state();
    const float along = st.velocity_ned.x * std::cos(st.heading_rad) + st.velocity_ned.y * std::sin(st.heading_rad);
    std::printf("  speed hold 1.5: %.2f m/s along heading, worst tilt est error %.1f deg\n", along, r.max_tilt_err_deg);
    CHECK_NEAR(along, 1.5, 0.3);
    CHECK(r.max_tilt_err_deg < 3.0);
    usv->set_mode(vehicles::UGVMode::MANUAL);
    r.neutral();
    r.run(1.0);
    r.vm->disarm();

    // ------------------------------------------------------------ 3. ROV / sub
    std::printf("[3] switch to ROV (6 thrusters)\n");
    CHECK(r.switch_to(VehicleType::ROV_SUBMARINE));
    CHECK(std::string(r.body().name()) == "sub");
    CHECK(r.vm->get_sensors() == sensors && r.vm->get_fusion() == fusion);
    auto* rov = r.vehicle<vehicles::ROVVehicle>();
    r.max_tilt_err_deg = 0.0;
    r.run(2.0);
    CHECK(r.arm());
    rov->set_mode(vehicles::ROVMode::DEPTH_HOLD);
    const double d0 = r.depth();
    r.rc[3] = -0.7f;                              // heave stick down: descend
    r.run(5.0);
    r.rc[3] = 0.0f;                               // release: hold this depth
    r.run(6.0);                                   // settle (small overshoot on capture)
    const double held = r.depth();
    r.run(10.0);
    std::printf("  dived %.2f m, then depth moved %.3f m in 10 s\n", held - d0, r.depth() - held);
    CHECK(held - d0 > 1.0);
    CHECK(std::fabs(r.depth() - held) < 0.1);
    CHECK(!r.gps->has_3d_fix());                  // no GPS underwater
    CHECK(!r.vm->get_latest_state().position_valid);

    // yaw on the spot: must not push the vehicle sideways. (Thrusters are weaker in
    // reverse, so a little drift is real physics; the old mixer rows made a sideways
    // push as big as the yaw command.)
    auto& sub = r.body();
    const sim::V3 p_yaw = sub.pos;
    r.rc[2] = 0.3f;
    r.run(3.0);
    const double moved = std::hypot(sub.pos.x - p_yaw.x, sub.pos.y - p_yaw.y);
    std::printf("  yaw stick: yaw rate %.2f rad/s, moved %.2f m in 3 s\n", sub.omega.z, moved);
    CHECK(sub.omega.z > 0.3);
    CHECK(moved < 0.3);
    r.rc[2] = 0.0f;
    r.run(3.0);
    r.rc[4] = 0.5f;                               // surge forward: no turning
    r.run(4.0);
    const sim::V3 vb2 = sub.ned_to_body(sub.vel);
    std::printf("  surge: %.2f m/s forward, yaw rate %.3f rad/s, depth change %.3f m\n", vb2.x, sub.omega.z, r.depth() - held);
    CHECK(vb2.x > 0.4);
    CHECK(std::fabs(sub.omega.z) < 0.03);
    CHECK(std::fabs(r.depth() - held) < 0.2);
    CHECK(r.max_tilt_err_deg < 3.0);
    r.neutral();
    r.run(3.0);
    r.vm->disarm();

    // ------------------------------------------------------------ 4. ROV, 4 thrusters, roll
    std::printf("[4] ROV with 4 thrusters: roll stick\n");
    CHECK(r.vm->configure_rov_thrusters(4));
    r.world->sync();
    CHECK(r.world->as<sim::SubModel>()->modelled_layout == 4);
    rov = r.vehicle<vehicles::ROVVehicle>();
    r.run(1.0);
    CHECK(r.arm());
    rov->set_mode(vehicles::ROVMode::STABILIZE);
    r.rc[0] = 0.4f;                               // roll right: target 0.2 rad
    r.run(4.0);
    std::printf("  roll %.1f deg (target %.1f)\n", r.body().roll() * VehicleRig::RAD2DEG, 0.2 * VehicleRig::RAD2DEG);
    CHECK(r.body().roll() > 0.12 && r.body().roll() < 0.3);
    r.neutral();
    r.run(3.0);
    CHECK(std::fabs(r.body().roll()) < 0.05);
    r.vm->disarm();

    // ------------------------------------------------------------ 5. back to the air
    std::printf("[5] switch back to the multicopter\n");
    CHECK(r.switch_to(VehicleType::UAV_MULTICOPTER));
    CHECK(std::string(r.body().name()) == "quad");
    CHECK(r.vm->get_sensors() == sensors && r.vm->get_fusion() == fusion);
    auto* uav = r.vehicle<vehicles::UAVVehicle>();
    uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
    r.rc[3] = 0.5f;
    r.max_tilt_err_deg = 0.0;
    r.run(2.0);
    CHECK(r.arm());
    r.rc[3] = 0.8f;                               // climb
    r.run(2.5);
    r.rc[3] = 0.5f;                               // hold
    r.run(2.0);
    const double alt = r.body().altitude();
    r.run(5.0);
    std::printf("  climbed to %.2f m, drift %.3f m in 5 s\n", alt, r.body().altitude() - alt);
    CHECK(alt > 1.0);
    CHECK(std::fabs(r.body().altitude() - alt) < 0.3);

    // ------------------------------------------------------------ 6. no switch while armed
    std::printf("[6] switch while armed\n");
    const int swaps = r.world->swaps();
    CHECK(!r.switch_to(VehicleType::USV_SURFACE_BOAT));
    CHECK(r.vm->get_active_type() == VehicleType::UAV_MULTICOPTER);
    CHECK(r.world->swaps() == swaps);
    CHECK(r.world->swaps() == 4);                 // rover->boat, ->sub, ->sub(4 thr), ->quad

    return test::finish("test_sitl_vehicles");
}
