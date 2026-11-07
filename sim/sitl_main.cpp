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
#include "sim/sim_world.hpp"
#include "vehicles/vehicle_manager.hpp"

#include <chrono>
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

/** 4S 5000 mAh pack: open-circuit voltage falls with charge used, sags with current. */
struct SimBattery {
    double used_mah = 0.0;
    double capacity_mah = 5000.0;
    double current_a = 0.0, voltage = 16.8;
    void step(double load, double dt) {   // load: fraction of full power, 0..1
        current_a = 0.4 + 40.0 * std::pow(std::fmax(0.0, load), 1.5);
        used_mah += current_a * dt * (1000.0 / 3600.0);
        const double soc = std::fmax(0.0, 1.0 - used_mah / capacity_mah);
        voltage = 13.6 + 3.2 * soc - 0.012 * current_a;
    }
    int remaining_pct() const { return static_cast<int>(std::lround(100.0 * std::fmax(0.0, 1.0 - used_mah / capacity_mah))); }
};

void usage(const char* argv0) {
    std::printf(
        "usage: %s [options]\n"
        "  --gcs HOST:PORT     where to send MAVLink (default 127.0.0.1:14550)\n"
        "  --port N            local UDP port to bind (default: any)\n"
        "  --sysid N           MAVLink system id (default 1)\n"
        "  --vehicle V         quad | rover | boat | sub (default quad); switch later from\n"
        "                      QGC with the VEHICLE_TYPE parameter (0 quad 1 rover 2 sub 3 boat)\n"
        "  --mode N            0, 1 or 2 (or manual | stabilize | alt_hold); per vehicle:\n"
        "                        quad  MANUAL / STABILIZE / ALT_HOLD   (default 1)\n"
        "                        rover, boat  MANUAL / HEADING_HOLD / SPEED_HOLD\n"
        "                        sub   MANUAL / STABILIZE / DEPTH_HOLD\n"
        "  --current N,E       water current for the boat, m/s (default 0,0)\n"
        "  --rc-neutral        feed centred sticks when no MANUAL_CONTROL arrives\n"
        "  --duration S        exit after S seconds (default: run until Ctrl-C)\n"
        "  --storage FILE      saved settings file (default autopilot_sitl.eeprom, 'none' = don't save)\n"
        "  --wipe              start from factory defaults (erases the settings file)\n"
        "  --signing-passphrase P  install MAVLink signing key SHA-256(P) (same as MAVProxy) and save it\n"
        "  --signing-off       remove the saved signing key\n"
        "  --quiet             no 1 Hz console status line\n",
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

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(2); }
            return argv[++i];
        };
        if (a == "--gcs") {
            const std::string v = next();
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
    sim::SimWorld world(vm, &hal.time());   // physics body follows the active vehicle type
    sim::SimContext& ctx = world.ctx;
    auto apply_world_settings = [&] {
        if (auto* boat = world.as<sim::BoatModel>()) boat->current = sim::V3(current_n, current_e, 0.0);
        if (auto* sub = world.as<sim::SubModel>()) {
            if (sub->modelled_layout != vm.get_active_actuator_count())
                std::printf("  note: no sub physics for %u thrusters, simulating the %d-thruster frame\n",
                            vm.get_active_actuator_count(), sub->modelled_layout);
        }
        if (world.as<sim::QuadModel>() && vm.get_active_actuator_count() != 4)
            std::printf("  note: quad physics models 4 rotors; outputs beyond 4 are not simulated\n");
    };
    apply_world_settings();
    auto* sm = vm.get_sensors();
    sm->register_sensor(std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU0"));
    sm->register_sensor(std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU1"));
    sm->register_sensor(std::make_shared<sim::SimBaro>(&ctx));
    sm->register_sensor(std::make_shared<sim::SimCompass>(&ctx));
    sm->register_sensor(std::make_shared<sim::SimGPS>(&ctx)); // 10 Hz, 100 ms latency
    SimBattery battery;

    // ---- ground station link ----
    hal::UdpLink udp(gcs_host, gcs_port, local_port);
    if (!udp.init()) {
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
    link.send_statustext(comms::mavlink::MAV_SEVERITY_INFO, "E-Club autopilot SITL ready");

    std::printf("autopilot_sitl: MAVLink v2 sysid %u, UDP local port %u -> %s:%u (%zu params)\n", sysid,
                udp.local_port(), gcs_host.c_str(), gcs_port, params.count());
    if (storage) {
        std::printf("  settings: %s (%u saved values loaded%s)\n", storage_path.c_str(),
                    static_cast<unsigned>(loaded.loaded), loaded.found ? "" : ", none saved yet");
    } else {
        std::printf("  settings: not saved (--storage none)\n");
    }
    std::printf("  MAVLink signing: %s\n", signing.enabled() ? "ON (unsigned messages are ignored)" : "off");
    std::printf("  open QGroundControl (it listens on UDP 14550) - the vehicle appears automatically\n");
    if (rc_neutral) std::printf("  --rc-neutral: centred sticks are fed when no joystick is connected\n");

    // ---- real-time loop ----
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    uint64_t ticks = 0;
    uint32_t last_manual = 0, last_manual_ms = 0;
    const uint64_t end_ticks = duration_s > 0 ? static_cast<uint64_t>(duration_s * 1e6 / LOOP_US) : 0;

    while (!g_stop && (end_ticks == 0 || ticks < end_ticks)) {
        const uint32_t now_ms = hal.time().millis();
        link.update();

        // Joystick present? Otherwise optionally hold the sticks centred.
        if (link.stats().manual_control_msgs != last_manual) {
            last_manual = link.stats().manual_control_msgs;
            last_manual_ms = now_ms;
        }
        const bool joystick = last_manual != 0 && now_ms - last_manual_ms < 500;
        if (rc_neutral && !joystick) {
            // centred sticks; a multicopter in ALT_HOLD reads mid throttle as "hold"
            const bool alt_hold = vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER &&
                                  comms::get_vehicle_mode(vm) == 2;
            vm.set_user_inputs(0.0f, 0.0f, 0.0f, alt_hold ? 0.5f : 0.0f);
        }

        vm.step(LOOP_US * 1e-6f);

        if (world.sync()) {   // VEHICLE_TYPE / frame changed from the GCS: new physics body
            std::printf("  simulator: now a %s (%s)\n", world.body().name(), vm.get_active_type_name());
            apply_world_settings();
        }
        for (int s = 0; s < PHYSICS_SUBSTEPS; ++s) world.step(LOOP_US * 1e-6 / PHYSICS_SUBSTEPS);
        battery.step(world.body().load(), LOOP_US * 1e-6);
        vm.set_battery_voltage(static_cast<float>(battery.voltage));
        link.set_battery(static_cast<float>(battery.voltage), static_cast<float>(battery.current_a),
                         battery.remaining_pct());

        hal.time().advance_time_ms(LOOP_US / 1000);
        ++ticks;
        // After a flight: keep what the fusion learned (drag coefficient) for next boot.
        if (ticks % 250 == 0) comms::sync_learned_params(params, vm);

        if (!quiet && ticks % 250 == 0) {
            const auto& st = vm.get_latest_state();
            const sim::SimBody& b = world.body();
            std::printf("[%6.1fs] %-5s %s %-12s alt %6.2f m  spd %4.1f m/s  r %6.1f p %6.1f y %6.1f deg  bat %.2f V  gcs %s  rx %u tx %u\n",
                        ticks * LOOP_US * 1e-6, b.name(), vm.is_armed() ? "ARMED   " : "disarmed",
                        comms::vehicle_mode_name(vm.get_active_type(), comms::get_vehicle_mode(vm)), b.altitude(),
                        b.ground_speed(),
                        st.euler_rad.x * 57.2958, st.euler_rad.y * 57.2958, st.euler_rad.z * 57.2958,
                        battery.voltage, link.gcs_connected() ? "yes" : "no", link.stats().rx_messages,
                        link.stats().tx_messages);
            std::fflush(stdout);
        }

        next_tick += std::chrono::microseconds(LOOP_US);
        std::this_thread::sleep_until(next_tick);
    }
    // Clean exit = "power off while disarmed": commit anything still pending.
    if (storage && !vm.is_armed()) {
        if (signing.enabled()) signing.save(storage.get());
        settings.save();
        storage->flush();
    }
    std::printf("autopilot_sitl: exit after %.1f s\n", ticks * LOOP_US * 1e-6);
    return 0;
}
