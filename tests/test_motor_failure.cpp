/**
 * Motor failure: detection, control re-allocation and the vehicle's reaction.
 *
 *   1. Mixer re-allocation math (hexa on 5, quad on 3, octa on 7, restore)
 *   2. Failed motor is commanded to idle; failsafe reason MOTOR commits a LAND
 *   3. Detector unit checks: ESC telemetry rule, nothing on the ground
 *   4. SITL hexa: motor dies in hover -> detected, keeps flying, pilot lands it
 *      (and, for comparison, the same failure with detection off: it crashes)
 *   5. SITL quad: motor dies -> detected, yaw abandoned, failsafe lands it upright
 *      (again compared with detection off)
 *      (an octa too: found and isolated just as fast)
 *   6. SITL hexa with ESC telemetry: detected from RPM
 *   7. No false alarm: healthy quad and hexa flown hard on the sticks
 */
#include "control/failsafe.hpp"
#include "control/motor_failure_detector.hpp"
#include "hal/posix/posix_hal.hpp"
#include "tests/sitl_rig.hpp"
#include "tests/test_common.hpp"
#include <cstdio>
#include <cstring>
#include <random>

using namespace autopilot;
using namespace autopilot::control;
using namespace autopilot::sitl;

namespace {

/** Torque/thrust the current rows produce for a demand, using the HEALTHY geometry
 *  (= what the airframe really does) and only the motors still alive. */
void effect(const ActuatorMatrixMixer& m, const float d[4], float out[4]) {
    for (int a = 0; a < 4; ++a) out[a] = 0.0f;
    for (uint8_t i = 0; i < m.get_num_channels(); ++i) {
        if (m.is_motor_failed(i)) continue;
        const auto& cur = m.get_actuator_config(i);
        const auto& nom = m.get_nominal_config(i);
        const float u = d[0] * cur.roll + d[1] * cur.pitch + d[2] * cur.yaw + d[3] * cur.heave;
        out[0] += u * nom.roll;
        out[1] += u * nom.pitch;
        out[2] += u * nom.yaw;
        out[3] += u * nom.heave;
    }
}

void check_allocation(ActuatorMatrixMixer& m, bool expect_yaw) {
    // Reference: the healthy frame's response to each unit demand.
    ActuatorMatrixMixer healthy = m;
    healthy.clear_motor_failures();
    for (int axis = 0; axis < 4; ++axis) {
        if (axis == 2 && !expect_yaw) continue;
        float d[4]{0, 0, 0, 0};
        d[axis] = 1.0f;
        float got[4], ref[4];
        effect(m, d, got);
        effect(healthy, d, ref);
        if (axis == 3) {
            // collective: same total thrust, no roll/pitch (yaw is allowed: it's given up first)
            CHECK_NEAR(got[3], ref[3], 1e-3);
            CHECK_NEAR(got[0], 0.0, 1e-3);
            CHECK_NEAR(got[1], 0.0, 1e-3);
        } else {
            // a pure torque on that axis, as strong as on the healthy frame
            // (without yaw control, the yaw side effect is free)
            for (int a = 0; a < 4; ++a) {
                if (a == 2 && !expect_yaw) continue;
                CHECK_NEAR(got[a], ref[a], 1e-3);
            }
        }
    }
}

double tilt_deg(const sim::QuadModel& q) {
    return std::acos(std::fmin(1.0, std::cos(q.roll()) * std::cos(q.pitch()))) * RAD2DEG;
}

/** Take off in ALT_HOLD and hover at ~3 m. */
void take_off(Rig& r) {
    r.uav->set_mode(vehicles::UAVFlightMode::ALT_HOLD);
    r.run(1.0);
    std::string err;
    CHECK(r.vm->arm(err));
    r.run(0.5);
    r.rc[3] = 0.8f;
    r.run(2.5);
    r.rc[3] = 0.5f;
    r.run(4.0);
}

struct FailureRun {
    double detect_s{-1};
    int detected{-1};
    double max_tilt{0}, min_alt{1e9}, max_alt{0};
};

/** Kill motor `m` and fly for `seconds` (or until the airframe is on the ground). */
FailureRun fail_and_fly(Rig& r, int m, double seconds) {
    FailureRun out;
    r.fail_motor(m);
    const double t0 = r.t;
    while (r.t - t0 < seconds && !r.quad.on_ground) {
        r.step();
        if (out.detect_s < 0 && r.uav->last_failed_motor() >= 0) {
            out.detect_s = r.t - t0;
            out.detected = r.uav->last_failed_motor();
        }
        out.max_tilt = std::fmax(out.max_tilt, tilt_deg(r.quad));
        out.min_alt = std::fmin(out.min_alt, r.quad.altitude());
        out.max_alt = std::fmax(out.max_alt, r.quad.altitude());
    }
    return out;
}

bool has_event(Rig& r, const char* needle) {
    vehicles::VehicleBase::Event ev;
    bool found = false;
    while (r.uav->pop_event(ev)) {
        std::printf("    event [%u] %s\n", ev.severity, ev.text);
        if (std::strstr(ev.text, needle)) found = true;
    }
    return found;
}

} // namespace

int main() {
    std::printf("== Motor failure: re-allocation, detection, SITL ==\n");

    // ---------------- 1. re-allocation math ----------------
    {
        ActuatorMatrixMixer hexa;
        hexa.configure_uav(6);
        CHECK(hexa.allocation_status() == AllocationStatus::NOMINAL);
        CHECK(hexa.set_motor_failed(0));
        CHECK(hexa.is_motor_failed(0));
        CHECK(hexa.failed_motor_mask() == 0x1);
        CHECK(hexa.allocation_status() == AllocationStatus::DEGRADED);
        check_allocation(hexa, true);
        // 5 motors must carry the thrust of 6: the busiest one (next to the dead
        // one) does 5/3 of its share, so the collective tops out at 0.6.
        CHECK_NEAR(hexa.collective_limit(), 0.6, 0.01);
        std::printf("  hexa on 5: %s, collective limit %.2f\n", allocation_status_name(hexa.allocation_status()),
                    hexa.collective_limit());

        ActuatorMatrixMixer quad;
        quad.configure_uav(4);
        CHECK(quad.set_motor_failed(2));
        CHECK(quad.allocation_status() == AllocationStatus::NO_YAW);
        check_allocation(quad, false);
        for (uint8_t i = 0; i < 4; ++i) {
            if (!quad.is_motor_failed(i)) CHECK_NEAR(quad.get_actuator_config(i).yaw, 0.0, 1e-6);
        }
        std::printf("  quad on 3: %s, collective limit %.2f\n", allocation_status_name(quad.allocation_status()),
                    quad.collective_limit());

        ActuatorMatrixMixer octa;
        octa.configure_uav(8);
        CHECK(octa.set_motor_failed(5));
        CHECK(octa.allocation_status() == AllocationStatus::DEGRADED);
        check_allocation(octa, true);

        // A second failure on the hexa (opposite side) still leaves a flyable frame.
        CHECK(hexa.set_motor_failed(3));
        CHECK(hexa.allocation_status() != AllocationStatus::NOMINAL);
        std::printf("  hexa on 4 (0 and 3 dead): %s\n", allocation_status_name(hexa.allocation_status()));

        // Restore: rows back to the original preset exactly.
        ActuatorMatrixMixer ref;
        ref.configure_uav(6);
        hexa.clear_motor_failures();
        CHECK(hexa.allocation_status() == AllocationStatus::NOMINAL);
        CHECK(hexa.failed_motor_mask() == 0);
        bool same = true;
        for (uint8_t i = 0; i < 6; ++i) {
            const auto& a = hexa.get_actuator_config(i);
            const auto& b = ref.get_actuator_config(i);
            same = same && a.enabled && a.roll == b.roll && a.pitch == b.pitch && a.yaw == b.yaw && a.heave == b.heave;
        }
        CHECK(same);

        // Un-failing the only failed motor also restores.
        CHECK(hexa.set_motor_failed(1));
        CHECK(hexa.set_motor_failed(1, false));
        CHECK(hexa.allocation_status() == AllocationStatus::NOMINAL);

        // ROV thrusters (bidirectional) are not handled: refused.
        ActuatorMatrixMixer rov(MixerPreset::ROV_6_VECTORED);
        CHECK(!rov.set_motor_failed(0));
        CHECK(!hexa.set_motor_failed(12));   // out of range
    }

    // ---------------- 2. outputs and failsafe ----------------
    {
        hal::PosixHAL hal;
        ActuatorMatrixMixer hexa;
        hexa.configure_uav(6);
        hexa.set_motor_failed(4);
        ControlDemands d;
        d.throttle = 0.5f;
        d.roll = 0.1f;
        d.pitch = -0.1f;
        d.yaw = 0.3f;
        hexa.output_to_actuators(d, &hal.pwm(), true);
        CHECK(hal.pwm().get_channel(4) == 1000);   // dead motor commanded to idle
        bool in_range = true;
        for (uint8_t i = 0; i < 6; ++i) {
            const float u = hexa.last_output(i);
            in_range = in_range && u >= 0.0f && u <= 1.0f;
        }
        CHECK(in_range);
        CHECK(hexa.last_output(4) == 0.0f);

        Failsafe fs;
        FailsafeInputs in;
        in.now_ms = 1000;
        in.last_rc_ms = 1000;
        in.rc_ever_received = true;
        in.attitude_valid = true;
        in.imu_usable = true;
        CHECK(fs.update(in) == FailsafeAction::NONE);
        in.actuator_failure = true;
        CHECK(fs.update(in) == FailsafeAction::LAND);
        CHECK(fs.reasons() & FS_MOTOR);
        in.actuator_failure = false;
        CHECK(fs.update(in) == FailsafeAction::LAND);   // committed
    }

    // ---------------- 3. detector unit checks ----------------
    {
        hal::PosixHAL hal;
        ActuatorMatrixMixer quad;
        quad.configure_uav(4);
        ControlDemands hover;
        hover.throttle = 0.5f;
        MotorFailureDetector det;
        const math::Vector3f still(0, 0, 0);
        uint32_t now = 1000;
        int got = -1;
        // Healthy ESCs at hover RPM: nothing.
        for (int k = 0; k < 100 && got < 0; ++k, now += 4) {
            quad.output_to_actuators(hover, &hal.pwm(), true);
            for (uint8_t i = 0; i < 4; ++i) det.report_esc_rpm(i, 7000.0f, now);
            got = det.update(quad, still, hover.throttle, 0.004f, true, now);
        }
        CHECK(got == -1);
        // ESC 3 reports 0 RPM while commanded: found within ~50 ms.
        int steps = 0;
        for (; steps < 100 && got < 0; ++steps, now += 4) {
            quad.output_to_actuators(hover, &hal.pwm(), true);
            for (uint8_t i = 0; i < 4; ++i) det.report_esc_rpm(i, i == 3 ? 0.0f : 7000.0f, now);
            got = det.update(quad, still, hover.throttle, 0.004f, true, now);
        }
        CHECK(got == 3);
        CHECK(det.last_cause() == MotorFailureDetector::Cause::ESC_TELEMETRY);
        CHECK(steps * 4 <= 70);
        // Same telemetry on the ground (not flying): never.
        det.reset();
        got = -1;
        for (int k = 0; k < 100 && got < 0; ++k, now += 4) {
            quad.output_to_actuators(hover, &hal.pwm(), true);
            for (uint8_t i = 0; i < 4; ++i) det.report_esc_rpm(i, 0.0f, now);
            got = det.update(quad, still, hover.throttle, 0.004f, false, now);
        }
        CHECK(got == -1);
        // Stale telemetry (ESC stopped reporting) is not evidence.
        det.reset();
        for (uint8_t i = 0; i < 4; ++i) det.report_esc_rpm(i, 0.0f, now);
        now += 1000;
        got = -1;
        for (int k = 0; k < 50 && got < 0; ++k, now += 4) {
            quad.output_to_actuators(hover, &hal.pwm(), true);
            got = det.update(quad, still, hover.throttle, 0.004f, true, now);
        }
        CHECK(got == -1);
    }

    // ---------------- 4. SITL hexa: keeps flying ----------------
    {
        Rig r(false, 6);
        CHECK(r.uav->get_rotor_count() == 6);
        take_off(r);
        const double alt0 = r.quad.altitude();
        CHECK(alt0 > 2.0);
        const FailureRun f = fail_and_fly(r, 2, 8.0);
        std::printf("  hexa, motor 3 dies at %.2f m: detected motor %d after %.0f ms, alt %.2f..%.2f m, max tilt %.1f deg, "
                    "yaw rate %.2f rad/s\n",
                    alt0, f.detected + 1, f.detect_s * 1000.0, f.min_alt, f.max_alt, f.max_tilt, r.quad.omega.z);
        CHECK(f.detected == 2);
        CHECK(f.detect_s > 0.0 && f.detect_s < 0.2);
        CHECK(r.uav->get_matrix_mixer()->allocation_status() == AllocationStatus::DEGRADED);
        CHECK(!r.uav->motor_emergency());
        CHECK(r.vm->is_armed());
        CHECK(!r.quad.on_ground);
        CHECK(f.max_tilt < 12.0);
        CHECK(f.min_alt > alt0 - 0.6);
        CHECK(f.max_alt < alt0 + 0.6);
        CHECK(std::fabs(r.quad.omega.z) < 1.5);   // yaw degraded, not lost
        CHECK(!r.uav->get_failsafe().active());   // pilot keeps control
        CHECK(has_event(r, "Motor 3 failed"));

        // Pilot still steers: a pitch stick moves it, then lands it on the stick.
        r.rc[1] = 0.3f;
        r.run(1.0);
        const double pitch_deg = r.quad.pitch() * RAD2DEG;
        r.rc[1] = 0.0f;
        r.run(2.0);
        CHECK(pitch_deg > 4.0);    // 0.3 stick = 9 deg nose up
        r.rc[3] = 0.2f;            // descend
        double t = 0;
        while (!r.quad.on_ground && t < 20.0) { r.step(); t += LOOP_MS * 1e-3; }
        // The land detector wants 1 s of "still" after touchdown.
        t = 0;
        while (!r.uav->is_landed() && t < 5.0) { r.step(); t += LOOP_MS * 1e-3; }
        std::printf("  hexa landed on 5 motors: touchdown %.2f m/s, pitch step %.1f deg, landed=%d\n",
                    r.quad.max_touchdown_speed, pitch_deg, r.uav->is_landed());
        CHECK(r.quad.on_ground);
        CHECK(r.quad.max_touchdown_speed < 1.5);
        CHECK(r.uav->is_landed());
        r.vm->disarm();

        // Re-arming clears the latched failure (the next flight re-checks in the air).
        r.rc[3] = 0.5f;
        r.run(0.2);
        std::string err;
        CHECK(r.vm->arm(err));
        CHECK(r.uav->get_matrix_mixer()->allocation_status() == AllocationStatus::NOMINAL);
        CHECK(r.uav->last_failed_motor() == -1);
        r.vm->disarm();
    }
    {
        // Same failure, detection off: the unchanged mixer can't hold it.
        Rig r(false, 6);
        r.uav->set_motor_failure_detection(false);
        take_off(r);
        const FailureRun f = fail_and_fly(r, 2, 8.0);
        std::printf("  hexa, detection OFF: on ground %d, touchdown %.2f m/s, max tilt %.0f deg\n", r.quad.on_ground,
                    r.quad.max_touchdown_speed, f.max_tilt);
        CHECK(r.quad.on_ground);
        CHECK(r.quad.max_touchdown_speed > 2.5);
    }

    // ---------------- 5. SITL quad: gives up yaw, lands ----------------
    {
        Rig r;
        take_off(r);
        const double alt0 = r.quad.altitude();
        FailureRun f = fail_and_fly(r, 1, 20.0);
        double max_spin = 0.0;
        // fail_and_fly stops at touchdown; let the land detector disarm.
        double t = 0;
        while (r.vm->is_armed() && t < 5.0) { r.step(); t += LOOP_MS * 1e-3; max_spin = std::fmax(max_spin, std::fabs(r.quad.omega.z)); }
        std::printf("  quad, motor 2 dies at %.2f m: detected motor %d after %.0f ms, max tilt %.0f deg, touchdown %.2f m/s, "
                    "disarmed=%d\n",
                    alt0, f.detected + 1, f.detect_s * 1000.0, f.max_tilt, r.quad.max_touchdown_speed, !r.vm->is_armed());
        CHECK(f.detected == 1);
        CHECK(f.detect_s > 0.0 && f.detect_s < 0.15);
        CHECK(r.uav->get_matrix_mixer()->allocation_status() == AllocationStatus::NO_YAW);
        CHECK(r.uav->motor_emergency());
        CHECK(r.uav->get_failsafe().reasons() & FS_MOTOR || !r.vm->is_armed());
        CHECK(f.max_tilt < 45.0);                 // stayed upright while spinning down
        CHECK(r.quad.on_ground);
        CHECK(r.quad.max_touchdown_speed < 1.5);
        CHECK(!r.vm->is_armed());                 // land detector disarmed it
        CHECK(has_event(r, "No yaw control left"));
    }
    {
        Rig r;
        r.uav->set_motor_failure_detection(false);
        take_off(r);
        const FailureRun f = fail_and_fly(r, 1, 20.0);
        std::printf("  quad, detection OFF: touchdown %.2f m/s, max tilt %.0f deg\n", r.quad.max_touchdown_speed,
                    f.max_tilt);
        CHECK(r.quad.max_touchdown_speed > 4.0 || f.max_tilt > 60.0);
    }

    {
        // Octa: losing one of 8 is milder, but it is still found and isolated.
        Rig r(false, 8);
        take_off(r);
        const FailureRun f = fail_and_fly(r, 6, 4.0);
        std::printf("  octa, motor 7 dies: detected motor %d after %.0f ms, max tilt %.1f deg\n", f.detected + 1,
                    f.detect_s * 1000.0, f.max_tilt);
        CHECK(f.detected == 6);
        CHECK(f.detect_s > 0.0 && f.detect_s < 0.2);
        CHECK(!r.quad.on_ground);
        CHECK(f.max_tilt < 8.0);
        CHECK(!r.uav->get_failsafe().active());
    }

    // ---------------- 6. ESC telemetry in the loop ----------------
    {
        Rig r(false, 6);
        r.esc_telemetry = true;
        take_off(r);
        const FailureRun f = fail_and_fly(r, 5, 2.0);
        std::printf("  hexa with ESC telemetry: detected motor %d after %.0f ms (%s)\n", f.detected + 1,
                    f.detect_s * 1000.0, motor_failure_cause_name(r.uav->motor_failure_detector().last_cause()));
        CHECK(f.detected == 5);
        CHECK(f.detect_s > 0.0 && f.detect_s < 0.12);
        CHECK(!r.quad.on_ground);
        CHECK(r.vm->is_armed());
    }

    // ---------------- 7. no false alarms in hard healthy flying ----------------
    for (uint8_t rotors : {4, 6}) {
        Rig r(false, rotors);
        take_off(r);
        std::mt19937 g(rotors);
        std::uniform_real_distribution<float> u(-1.0f, 1.0f);
        for (int i = 0; i < 60; ++i) {
            // full-stick bangs mixed with random sticks, every 0.25-0.75 s
            r.rc[0] = (i % 4 == 0) ? 1.0f : (i % 4 == 2 ? -1.0f : u(g));
            r.rc[1] = (i % 3 == 0) ? -1.0f : u(g);
            r.rc[2] = (i % 5 == 0) ? 1.0f : u(g);
            r.rc[3] = 0.5f + 0.5f * u(g);
            r.run(0.25 + 0.25 * (i % 3));
            if (r.quad.altitude() < 1.5) {   // keep it in the air
                r.rc[0] = r.rc[1] = 0.0f;
                r.rc[3] = 1.0f;
                r.run(1.0);
            }
        }
        std::printf("  healthy %s flown hard for %.0f s: failed motor %d, alt %.1f m\n",
                    rotors == 4 ? "quad" : "hexa", r.t, r.uav->last_failed_motor(), r.quad.altitude());
        CHECK(r.uav->last_failed_motor() == -1);
        CHECK(r.vm->is_armed());
        CHECK(!r.quad.on_ground);
    }

    return test::finish("test_motor_failure");
}
