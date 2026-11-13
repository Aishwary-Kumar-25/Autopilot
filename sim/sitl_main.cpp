/**
 * autopilot_sitl: real-time software-in-the-loop with a ground station.
 *
 * The real flight stack (VehicleManager, SensorManager, SensorFusion, the
 * vehicle controllers, failsafe, mixers) drives a simulated vehicle in real
 * time and talks MAVLink v2 over UDP to QGroundControl. The vehicle can be a
 * quad, rover, boat or ROV, and can be switched at runtime from QGC by setting
 * the VEHICLE_TYPE parameter (0 quad, 1 rover, 2 ROV, 3 boat) while disarmed:
 * the same firmware swaps its controller and mixer, and the simulator swaps the
 * physics body at the same spot (sim/sim_world.hpp).
 *
 *   ./autopilot_sitl                       # quad, sends to 127.0.0.1:14550 (QGC default)
 *   ./autopilot_sitl --vehicle rover       # or boat / sub
 *   ./autopilot_sitl --gcs 192.168.1.20:14550 --mode alt_hold
 *   ./autopilot_sitl --rc-neutral          # arm/hover without a joystick
 *   ./autopilot_sitl --signing-passphrase "my secret"   # only signed MAVLink accepted
 *   ./autopilot_sitl --frame 6 --fail-motor 3@20         # hexa, motor 3 dies 20 s after arming
 *   ./autopilot_sitl --bridge 9100 --speedup 0 --gcs off  # scripted tests / ROS 2 (sim/sim_bridge.hpp)
 *   ./autopilot_sitl --physics json                       # physics from Gazebo (sim/json_physics.hpp)
 *
 * Parameters and the signing key are saved in autopilot_sitl.eeprom (the
 * SITL's flash), so tuning set from QGC is still there next run.
 *
 * Pilot input: MANUAL_CONTROL from a joystick or QGC's virtual joystick
 * (Application Settings > General > Virtual Joystick). Without any RC the
 * pre-arm check refuses to arm, exactly like the real vehicle.
 *
 * See docs/MAVLINK_QGC.md.
 */
#include "comms/gcs_link.hpp"
#include "comms/settings.hpp"
#include "hal/posix/file_storage.hpp"
#include "comms/vehicle_params.hpp"
#include "hal/posix/posix_hal.hpp"
#include "hal/posix/udp_link.hpp"
#include "sim/json_physics.hpp"
#include "sim/sim_battery.hpp"
#include "sim/sim_bridge.hpp"
#include "sim/sim_world.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/vehicle_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

using namespace autopilot;

namespace {

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

constexpr uint32_t LOOP_US = 4000;   // 250 Hz flight loop
constexpr int PHYSICS_SUBSTEPS = 4;  // 1 kHz physics

void usage(const char* argv0) {
    std::printf(
        "usage: %s [options]\n"
        "  --gcs HOST:PORT     where to send MAVLink (default 127.0.0.1:14550); 'off' = no GCS link\n"
        "  --port N            local UDP port to bind (default: any)\n"
        "  --sysid N           MAVLink system id (default 1)\n"
        "  --vehicle V         quad | rover | boat | sub (default quad); switch later from\n"
        "                      QGC with the VEHICLE_TYPE parameter (0 quad 1 rover 2 sub 3 boat)\n"
        "  --mode N            0, 1 or 2 (or manual | stabilize | alt_hold); per vehicle:\n"
        "                        quad  MANUAL / STABILIZE / ALT_HOLD   (default 1)\n"
        "                        rover, boat  MANUAL / HEADING_HOLD / SPEED_HOLD\n"
        "                        sub   MANUAL / STABILIZE / DEPTH_HOLD\n"
        "  --current N,E       water current for the boat, m/s (default 0,0)\n"
        "  --frame N           multirotor rotor count: 4 quad, 6 hexa, 8 octa (sets UAV_ROTORS)\n"
        "  --fail-motor M@S    kill motor M (1-based) S seconds after arming, to watch the\n"
        "                      motor-failure handling (e.g. --fail-motor 3@20)\n"
        "  --esc-telemetry     simulated ESCs report RPM to the flight controller\n"
        "  --rc-neutral        feed centred sticks when no MANUAL_CONTROL arrives\n"
        "  --duration S        exit after S seconds (default: run until Ctrl-C)\n"
        "  --storage FILE      saved settings file (default autopilot_sitl.eeprom, 'none' = don't save)\n"
        "  --wipe              start from factory defaults (erases the settings file)\n"
        "  --signing-passphrase P  install MAVLink signing key SHA-256(P) (same as MAVProxy) and save it\n"
        "  --signing-off       remove the saved signing key\n"
        "  --quiet             no 1 Hz console status line\n"
        "\n"
        "Simulator bridge (scripted tests, ROS 2 package ros2/autopilot_sim, datasets):\n"
        "  --bridge PORT       telemetry + command channel on UDP PORT (e.g. 9100); simulated time starts\n"
        "                      when a client sends \"start\". See docs/ROS2_SIMULATOR.md\n"
        "  --bridge-rate HZ    telemetry rate in simulated time (default 50, max 250)\n"
        "  --bridge-any        accept bridge clients from other machines (default: this PC only)\n"
        "  --speedup X         run X times faster than real time; 0 = as fast as possible (default 1)\n"
        "  --seed N            sensor-noise random seed (default 42)\n"
        "\n"
        "External physics (Gazebo + ardupilot_gazebo, or any ArduPilot-JSON simulator):\n"
        "  --physics json[:HOST:PORT]  lock-step with the simulator (default 127.0.0.1:9002)\n"
        "  --json-pwm MIN:MAX  ESC range the external model expects (default 1100:1900 = iris)\n",
        argv0);
}

} // namespace

int main(int argc, char** argv) {
    std::string gcs_host = "127.0.0.1";
    uint16_t gcs_port = 14550, local_port = 0;
    uint8_t sysid = 1;
    uint32_t mode = 1;
    bool rc_neutral = false, quiet = false, wipe = false, signing_off = false;
    std::string storage_path = "autopilot_sitl.eeprom", passphrase;
    vehicles::VehicleType vtype = vehicles::VehicleType::UAV_MULTICOPTER;
    bool vtype_given = false;
    double current_n = 0.0, current_e = 0.0;
    double duration_s = 0.0;
    int frame = 0;                 // 0 = keep UAV_ROTORS as configured / saved
    int fail_motor = -1;           // 0-based, -1 = none
    double fail_after_s = 0.0;
    bool esc_telemetry = false;
    bool gcs_off = false;
    int bridge_port = -1;
    double bridge_rate = 50.0;
    bool bridge_any = false;
    double speedup = 1.0;
    unsigned seed = 42;
    bool json_physics = false;
    sim::JsonPhysics::Config json_cfg;
    json_cfg.pwm_min = 1100;
    json_cfg.pwm_max = 1900;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(2); }
            return argv[++i];
        };
        if (a == "--gcs") {
            const std::string v = next();
            if (v == "off" || v == "none" || v == "false") { gcs_off = true; continue; }
            const auto colon = v.rfind(':');
            gcs_host = v.substr(0, colon);
            if (colon != std::string::npos) gcs_port = static_cast<uint16_t>(std::atoi(v.c_str() + colon + 1));
        } else if (a == "--port") {
            local_port = static_cast<uint16_t>(std::atoi(next()));
        } else if (a == "--sysid") {
            sysid = static_cast<uint8_t>(std::atoi(next()));
        } else if (a == "--mode") {
            const std::string m = next();
            mode = (m == "manual" || m == "0") ? 0 : (m == "alt_hold" || m == "2") ? 2 : 1;
        } else if (a == "--vehicle") {
            const std::string v = next();
            if (v == "quad" || v == "uav" || v == "copter") vtype = vehicles::VehicleType::UAV_MULTICOPTER;
            else if (v == "rover" || v == "ugv" || v == "car") vtype = vehicles::VehicleType::UGV_GROUND_ROVER;
            else if (v == "boat" || v == "usv") vtype = vehicles::VehicleType::USV_SURFACE_BOAT;
            else if (v == "sub" || v == "rov") vtype = vehicles::VehicleType::ROV_SUBMARINE;
            else { usage(argv[0]); return 2; }
            vtype_given = true;
        } else if (a == "--current") {
            const std::string v = next();
            if (std::sscanf(v.c_str(), "%lf,%lf", &current_n, &current_e) != 2) { usage(argv[0]); return 2; }
        } else if (a == "--frame") {
            frame = std::atoi(next());
            if (frame < 3 || frame > 16) { usage(argv[0]); return 2; }
        } else if (a == "--fail-motor") {
            const std::string v = next();
            int m = 0;
            if (std::sscanf(v.c_str(), "%d@%lf", &m, &fail_after_s) < 1 || m < 1) { usage(argv[0]); return 2; }
            fail_motor = m - 1;
        } else if (a == "--esc-telemetry") {
            esc_telemetry = true;
        } else if (a == "--rc-neutral") {
            rc_neutral = true;
        } else if (a == "--duration") {
            duration_s = std::atof(next());
        } else if (a == "--storage") {
            storage_path = next();
        } else if (a == "--wipe") {
            wipe = true;
        } else if (a == "--signing-passphrase") {
            passphrase = next();
        } else if (a == "--signing-off") {
            signing_off = true;
        } else if (a == "--quiet") {
            quiet = true;
        } else if (a == "--bridge") {
            bridge_port = std::atoi(next());
        } else if (a == "--bridge-rate") {
            bridge_rate = std::fmax(1.0, std::fmin(250.0, std::atof(next())));
        } else if (a == "--bridge-any") {
            bridge_any = true;
        } else if (a == "--speedup") {
            speedup = std::fmax(0.0, std::atof(next()));
        } else if (a == "--seed") {
            seed = static_cast<unsigned>(std::strtoul(next(), nullptr, 10));
        } else if (a == "--physics") {
            const std::string v = next();
            if (v.rfind("json", 0) != 0) { usage(argv[0]); return 2; }
            json_physics = true;
            if (v.size() > 5 && v[4] == ':') {
                const std::string hp = v.substr(5);
                const auto colon = hp.rfind(':');
                json_cfg.host = hp.substr(0, colon);
                if (colon != std::string::npos) json_cfg.port = static_cast<uint16_t>(std::atoi(hp.c_str() + colon + 1));
            }
        } else if (a == "--json-pwm") {
            unsigned lo = 0, hi = 0;
            if (std::sscanf(next(), "%u:%u", &lo, &hi) != 2 || lo >= hi) { usage(argv[0]); return 2; }
            json_cfg.pwm_min = static_cast<uint16_t>(lo);
            json_cfg.pwm_max = static_cast<uint16_t>(hi);
        } else {
            usage(argv[0]);
            return a == "--help" || a == "-h" ? 0 : 2;
        }
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // ---- flight stack + simulated vehicle ----
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);   // sim clock, paced to wall clock below
    hal.time().advance_time_ms(1);

    vehicles::VehicleManager vm(&hal, vtype);
    if (!vm.init()) {
        std::fprintf(stderr, "VehicleManager init failed\n");
        return 1;
    }
    sim::SimWorld world(vm, &hal.time(), json_physics);   // physics body follows the active vehicle type
    sim::SimContext& ctx = world.ctx;
    ctx.rng.seed(seed);
    sim::SimEnv env;
    env.current = sim::V3(current_n, current_e, 0.0);
    std::unique_ptr<sim::SimBridge> bridge;   // created below, once the sensors exist
    auto apply_world_settings = [&] {
        if (auto* boat = world.as<sim::BoatModel>()) boat->current = env.current;
        if (auto* quad = world.as<sim::QuadModel>()) {
            quad->wind = env.wind;
            for (int i = 0; i < sim::QuadModel::MAX_MOTORS; ++i) quad->motor_health[i] = env.motor_health[i];
        }
        if (bridge) bridge->apply_env();
        if (auto* sub = world.as<sim::SubModel>()) {
            if (sub->modelled_layout != vm.get_active_actuator_count())
                std::printf("  note: no sub physics for %u thrusters, simulating the %d-thruster frame\n",
                            vm.get_active_actuator_count(), sub->modelled_layout);
        }
    };
    apply_world_settings();
    auto* sm = vm.get_sensors();
    sim::SimBridge::Sensors sens;
    sens.imu[0] = std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU0");
    sens.imu[1] = std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU1");
    sens.baro = std::make_shared<sim::SimBaro>(&ctx);
    sens.mag = std::make_shared<sim::SimCompass>(&ctx);
    sens.gps = std::make_shared<sim::SimGPS>(&ctx);   // 10 Hz, 100 ms latency
    sm->register_sensor(sens.imu[0]);
    sm->register_sensor(sens.imu[1]);
    sm->register_sensor(sens.baro);
    sm->register_sensor(sens.mag);
    sm->register_sensor(sens.gps);
    sim::SimBattery battery;
    if (bridge_port >= 0) {
        bridge = std::make_unique<sim::SimBridge>(vm, world, env, sens, battery);
        if (!bridge->open(static_cast<uint16_t>(bridge_port), bridge_rate, bridge_any)) {
            std::fprintf(stderr, "bridge: %s\n", bridge->error().c_str());
            return 1;
        }
    }
    std::unique_ptr<sim::JsonPhysics> physics;
    if (json_physics) {
        physics = std::make_unique<sim::JsonPhysics>(json_cfg);
        if (!physics->open()) {
            std::fprintf(stderr, "physics: %s\n", physics->error().c_str());
            return 1;
        }
    }

    // ---- ground station link ----
    hal::UdpLink udp(gcs_host, gcs_port, local_port);
    if (!gcs_off && !udp.init()) {
        std::fprintf(stderr, "UDP: %s\n", udp.last_error().c_str());
        return 1;
    }
    comms::ParamStore params;
    comms::register_vehicle_params(params, vm);

    // ---- saved settings (the SITL's "flash") ----
    std::unique_ptr<hal::FileStorage> storage;
    if (storage_path != "none") {
        storage = std::make_unique<hal::FileStorage>(storage_path);
        if (wipe) storage->wipe();
        if (!storage->open()) {
            std::fprintf(stderr, "storage: can't read %s\n", storage_path.c_str());
            return 1;
        }
    }
    comms::Settings settings(&params, storage.get());
    const auto loaded = settings.load();
    if (vtype_given && vm.get_active_type() != vtype) {   // --vehicle beats a saved VEHICLE_TYPE
        std::string why;
        vm.switch_vehicle(vtype, why);
    }
    if (frame > 0 && vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER) {
        vm.configure_uav_rotors(static_cast<uint8_t>(frame));   // --frame beats a saved UAV_ROTORS
    }
    comms::set_vehicle_mode(vm, mode);   // after load: a saved VEHICLE_TYPE may have replaced the vehicle
    if (world.sync()) apply_world_settings();   // physics body for the vehicle actually running

    comms::mavlink::Signing signing;
    signing.load(storage.get());
    if (!passphrase.empty()) {
        uint8_t key[comms::mavlink::Signing::KEY_LEN];
        comms::mavlink::Signing::key_from_passphrase(passphrase.c_str(), key);
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        signing.set_key(key, comms::mavlink::Signing::timestamp_from_unix(
                                 static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(now).count())));
    } else if (signing_off) {
        signing.disable();
    }
    if (storage && (!passphrase.empty() || signing_off)) {
        signing.save(storage.get());
        storage->flush();
    }

    comms::GcsLink::Config cfg;
    cfg.sysid = sysid;
    cfg.trusted_link = true;   // SITL on this PC: SETUP_SIGNING may install a key (as over USB)
    comms::GcsLink link(&udp, &vm, &params, cfg);
    link.set_signing(&signing);
    link.set_settings(&settings);
    if (!gcs_off) link.send_statustext(comms::mavlink::MAV_SEVERITY_INFO, "E-Club autopilot SITL ready");

    if (gcs_off) {
        std::printf("autopilot_sitl: no ground-station link (--gcs off), %zu params\n", params.count());
    } else {
        std::printf("autopilot_sitl: MAVLink v2 sysid %u, UDP local port %u -> %s:%u (%zu params)\n", sysid,
                    udp.local_port(), gcs_host.c_str(), gcs_port, params.count());
    }
    if (bridge) std::printf("  simulator bridge: UDP %u, telemetry %.0f Hz (sim time)\n", bridge->port(), bridge_rate);
    if (physics) {
        std::printf("  physics: external, ArduPilot JSON protocol -> %s:%u, ESC range %u..%u us\n",
                    json_cfg.host.c_str(), json_cfg.port, json_cfg.pwm_min, json_cfg.pwm_max);
    }
    if (speedup != 1.0) {
        if (speedup <= 0.0) std::printf("  speed: as fast as possible\n");
        else std::printf("  speed: %.1fx real time\n", speedup);
    }
    if (storage) {
        std::printf("  settings: %s (%u saved values loaded%s)\n", storage_path.c_str(),
                    static_cast<unsigned>(loaded.loaded), loaded.found ? "" : ", none saved yet");
    } else {
        std::printf("  settings: not saved (--storage none)\n");
    }
    std::printf("  MAVLink signing: %s\n", signing.enabled() ? "ON (unsigned messages are ignored)" : "off");
    if (!gcs_off) std::printf("  open QGroundControl (it listens on UDP 14550) - the vehicle appears automatically\n");
    if (rc_neutral) std::printf("  --rc-neutral: centred sticks are fed when no joystick is connected\n");

    // ---- main loop: one flight-controller tick every LOOP_US of simulated time ----
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    uint64_t ticks = 0;
    uint32_t last_manual = 0, last_manual_ms = 0;
    uint64_t armed_ticks = 0;
    bool motor_killed = false;
    int8_t reported_failure = -1;
    const uint64_t end_ticks = duration_s > 0 ? static_cast<uint64_t>(duration_s * 1e6 / LOOP_US) : 0;
    const uint64_t t0_us = hal.time().micros();
    auto sim_seconds = [&] { return (hal.time().micros() - t0_us) * 1e-6; };

    // External physics: lock-step frames; the simulator's clock drives ours.
    sim::JsonFrame pframe;
    double phys_t_last = -1.0;      // timestamp of the previous physics frame
    double phys_acc_us = 0.0;       // physics time not yet added to the HAL clock
    uint32_t phys_waits = 0;
    auto physics_frame = [&]() -> bool {
        // Outputs with the bridge's motor faults applied (a dead ESC = 1000 us = no thrust).
        uint16_t us[16]{};
        int n = 0;
        hal::PWMOutput* pwm = hal.get_pwm();
        if (pwm && pwm->is_enabled()) {
            n = std::min<int>(16, vm.get_active_actuator_count());
            for (int i = 0; i < n; ++i) {
                const double raw = pwm->get_channel(static_cast<uint8_t>(i));
                const double h = env.motor_health[i];
                us[i] = static_cast<uint16_t>(h >= 1.0 ? raw : 1000.0 + (raw - 1000.0) * std::fmax(0.0, h));
            }
        }
        world.body().step_pwm(n ? us : nullptr, n, 0.0);   // remember outputs (battery load)
        bool fresh = true;
        while (!g_stop && !env.quit) {
            if (physics->exchange(n ? us : nullptr, n, pframe, 1000, fresh)) break;
            fresh = false;   // same frame_count again: a lock-step simulator repeats its state
            if (++phys_waits == 1 || phys_waits % 10 == 0) {
                std::printf("  waiting for the physics simulator at %s:%u (is Gazebo running and unpaused?)\n",
                            json_cfg.host.c_str(), json_cfg.port);
                std::fflush(stdout);
            }
        }
        if (g_stop || env.quit) return false;
        phys_waits = 0;
        static_cast<sim::ExternalBody&>(world.body())
            .set_state(pframe.gyro, pframe.accel_body, pframe.pos, pframe.vel, pframe.quat);
        if (phys_t_last >= 0.0 && pframe.timestamp > phys_t_last) phys_acc_us += (pframe.timestamp - phys_t_last) * 1e6;
        if (phys_t_last >= 0.0 && pframe.timestamp < phys_t_last - 1e-6) {
            std::printf("  physics simulator restarted (time went back), continuing\n");
        }
        phys_t_last = pframe.timestamp;
        return true;
    };

    // With a bridge, simulated time starts when the client says "start" (after it
    // queued its "at <t>" commands), so scripts line up exactly even at --speedup 0.
    if (bridge && !bridge->started()) {
        std::printf("  waiting for a bridge client on UDP %u to send \"start\" ...\n", bridge->port());
        std::fflush(stdout);
        while (!g_stop && !env.quit && !bridge->started()) {
            bridge->update(0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        next_tick = clock::now();
    }

    while (!g_stop && !env.quit && (end_ticks == 0 || ticks < end_ticks)) {
        const uint32_t now_ms = hal.time().millis();
        if (!gcs_off) link.update();

        // Pilot input. Priority: bridge sticks > joystick (MANUAL_CONTROL) > --rc-neutral.
        // "rc off" from the bridge cuts all of them (RC-loss failsafe test).
        if (link.stats().manual_control_msgs != last_manual) {
            last_manual = link.stats().manual_control_msgs;
            last_manual_ms = now_ms;
        }
        const bool joystick = last_manual != 0 && now_ms - last_manual_ms < 500;
        if (!env.rc_lost) {
            if (env.sticks_active) {
                vm.set_user_inputs(env.sticks[0], env.sticks[1], env.sticks[2], env.sticks[3], env.sticks[4],
                                   env.sticks[5]);
            } else if (rc_neutral && !joystick) {
                // centred sticks; a multicopter in ALT_HOLD reads mid throttle as "hold"
                const bool alt_hold = vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER &&
                                      comms::get_vehicle_mode(vm) == 2;
                vm.set_user_inputs(0.0f, 0.0f, 0.0f, alt_hold ? 0.5f : 0.0f);
            }
        }

        // Simulated ESC telemetry (RPM) into the flight controller.
        auto* uav = dynamic_cast<vehicles::UAVVehicle*>(vm.get_active_vehicle());
        auto* quad = world.as<sim::QuadModel>();
        if (esc_telemetry && uav && quad) {
            for (int m = 0; m < quad->n_motors; ++m) uav->report_esc_rpm(static_cast<uint8_t>(m), float(quad->rpm(m)));
        }

        vm.step(LOOP_US * 1e-6f);

        // Motor failure injection: --fail-motor M@S, S seconds after arming.
        armed_ticks = vm.is_armed() ? armed_ticks + 1 : 0;
        const bool airborne = quad ? !quad->on_ground : world.body().altitude() > 0.3;
        if (fail_motor >= 0 && !motor_killed && fail_motor < 16 && armed_ticks * LOOP_US * 1e-6 >= fail_after_s &&
            airborne) {
            env.motor_health[fail_motor] = 0.0;
            apply_world_settings();
            motor_killed = true;
            std::printf("  SIM: motor %d killed at %.2f m\n", fail_motor + 1, world.body().altitude());
        }
        if (uav && uav->last_failed_motor() != reported_failure) {
            reported_failure = uav->last_failed_motor();
            if (reported_failure >= 0) {
                std::printf("  FC: motor %d declared failed (%s), mixer %s%s\n", reported_failure + 1,
                            control::motor_failure_cause_name(uav->motor_failure_detector().last_cause()),
                            control::allocation_status_name(uav->get_matrix_mixer()->allocation_status()),
                            uav->motor_emergency() ? ", landing" : "");
            }
        }

        if (world.sync()) {   // VEHICLE_TYPE / frame changed from the GCS: new physics body
            std::printf("  simulator: now a %s (%s)\n", world.body().name(), vm.get_active_type_name());
            apply_world_settings();
        }

        // Bridge: commands (incl. scheduled ones) and telemetry, at the time of this tick.
        if (bridge) bridge->update(sim_seconds());

        // Physics for one flight-loop period.
        if (physics) {
            while (phys_acc_us < LOOP_US && !g_stop && !env.quit) {
                if (!physics_frame()) break;
            }
            const uint32_t adv_ms = static_cast<uint32_t>(std::min<double>(phys_acc_us, LOOP_US) / 1000.0);
            hal.time().advance_time_ms(std::max<uint32_t>(1, adv_ms));
            phys_acc_us = std::fmax(0.0, phys_acc_us - std::max<uint32_t>(1, adv_ms) * 1000.0);
        } else {
            for (int s = 0; s < PHYSICS_SUBSTEPS; ++s) world.step(LOOP_US * 1e-6 / PHYSICS_SUBSTEPS);
            hal.time().advance_time_ms(LOOP_US / 1000);
        }
        battery.step(world.body().load(), LOOP_US * 1e-6);
        vm.set_battery_voltage(static_cast<float>(battery.voltage));
        link.set_battery(static_cast<float>(battery.voltage), static_cast<float>(battery.current_a),
                         battery.remaining_pct());

        ++ticks;
        // After a flight: keep what the fusion learned (drag coefficient) for next boot.
        if (ticks % 250 == 0) comms::sync_learned_params(params, vm);

        if (!quiet && ticks % 250 == 0) {
            const auto& st = vm.get_latest_state();
            const sim::SimBody& b = world.body();
            std::printf("[%6.1fs] %-5s %s %-12s alt %6.2f m  spd %4.1f m/s  r %6.1f p %6.1f y %6.1f deg  bat %.2f V  gcs %s  rx %u tx %u\n",
                        sim_seconds(), b.name(), vm.is_armed() ? "ARMED   " : "disarmed",
                        comms::vehicle_mode_name(vm.get_active_type(), comms::get_vehicle_mode(vm)), b.altitude(),
                        b.ground_speed(),
                        st.euler_rad.x * 57.2958, st.euler_rad.y * 57.2958, st.euler_rad.z * 57.2958,
                        battery.voltage, link.gcs_connected() ? "yes" : "no", link.stats().rx_messages,
                        link.stats().tx_messages);
            std::fflush(stdout);
        }

        // Pacing: the external simulator paces itself; otherwise follow the wall clock / speedup.
        if (!physics && speedup > 0.0) {
            next_tick += std::chrono::microseconds(static_cast<int64_t>(LOOP_US / speedup));
            std::this_thread::sleep_until(next_tick);
        }
    }
    // Clean exit = "power off while disarmed": commit anything still pending.
    if (storage && !vm.is_armed()) {
        if (signing.enabled()) signing.save(storage.get());
        settings.save();
        storage->flush();
    }
    std::printf("autopilot_sitl: exit after %.1f s (simulated)\n", sim_seconds());
    return 0;
}
