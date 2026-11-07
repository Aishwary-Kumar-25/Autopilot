/**
 * autopilot_sitl: real-time software-in-the-loop with a ground station.
 *
 * The real flight stack (VehicleManager, SensorManager, SensorFusion, UAV
 * controller, failsafe, mixer) flies the simulated quad from sim/quad_sim.hpp
 * in real time, and talks MAVLink v2 over UDP to QGroundControl.
 *
 *   ./autopilot_sitl                       # sends to 127.0.0.1:14550 (QGC default)
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
#include "sim/quad_sim.hpp"
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
    void step(const double cmd[4], double dt) {
        double mean = 0.0;
        for (int i = 0; i < 4; ++i) mean += std::fmax(0.0, cmd[i]) / 4.0;
        current_a = 0.4 + 40.0 * std::pow(mean, 1.5);
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
        "  --mode M            manual | stabilize | alt_hold (default stabilize)\n"
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
            mode = (m == "manual") ? 0 : (m == "alt_hold") ? 2 : 1;
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

    // ---- flight stack + simulated airframe ----
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);   // sim clock, paced to wall clock below
    hal.time().advance_time_ms(1);
    sim::QuadModel quad;
    sim::SimContext ctx;
    ctx.model = &quad;
    ctx.time = &hal.time();

    vehicles::VehicleManager vm(&hal, vehicles::VehicleType::UAV_MULTICOPTER);
    if (!vm.init()) {
        std::fprintf(stderr, "VehicleManager init failed\n");
        return 1;
    }
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
    comms::set_vehicle_mode(vm, mode);   // after load: a saved VEHICLE_TYPE may have replaced the vehicle

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
    double cmd[4] = {0, 0, 0, 0};

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
            const bool alt_hold = comms::get_vehicle_mode(vm) == 2;
            vm.set_user_inputs(0.0f, 0.0f, 0.0f, alt_hold ? 0.5f : 0.0f);
        }

        vm.step(LOOP_US * 1e-6f);

        const bool uav = vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER;
        for (int i = 0; i < 4; ++i) {
            const uint16_t us = (uav && hal.pwm().is_enabled()) ? hal.pwm().get_channel(i) : 1000;
            cmd[i] = (static_cast<double>(us) - 1000.0) / 1000.0;
        }
        for (int s = 0; s < PHYSICS_SUBSTEPS; ++s) quad.step(cmd, LOOP_US * 1e-6 / PHYSICS_SUBSTEPS);
        battery.step(cmd, LOOP_US * 1e-6);
        vm.set_battery_voltage(static_cast<float>(battery.voltage));
        link.set_battery(static_cast<float>(battery.voltage), static_cast<float>(battery.current_a),
                         battery.remaining_pct());

        hal.time().advance_time_ms(LOOP_US / 1000);
        ++ticks;

        if (!quiet && ticks % 250 == 0) {
            const auto& st = vm.get_latest_state();
            std::printf("[%6.1fs] %s %-9s alt %5.2f m  r %6.1f p %6.1f y %6.1f deg  bat %.2f V  gcs %s  rx %u tx %u\n",
                        ticks * LOOP_US * 1e-6, vm.is_armed() ? "ARMED   " : "disarmed",
                        comms::vehicle_mode_name(vm.get_active_type(), comms::get_vehicle_mode(vm)), quad.altitude(),
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
