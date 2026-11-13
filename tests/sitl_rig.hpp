#pragma once
/**
 * Shared SITL rig for closed-loop tests: the real flight stack (VehicleManager,
 * shared SensorManager + SensorFusion, UAV controller, failsafe, matrix mixer)
 * flying the simulated multirotor in sim/quad_sim.hpp (quad by default, or a
 * hexa/octa built from the same mixer geometry).
 */
#include "hal/posix/posix_hal.hpp"
#include "sim/quad_sim.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/vehicle_manager.hpp"
#include <cmath>
#include <memory>

namespace autopilot {
namespace sitl {
constexpr double RAD2DEG = 57.29577951;
constexpr uint32_t LOOP_MS = 4;     // 250 Hz flight loop
constexpr int PHYSICS_SUBSTEPS = 4; // 1 kHz physics

struct Rig {
    hal::PosixHAL hal;
    sim::QuadModel quad;
    sim::SimContext ctx;
    std::shared_ptr<sim::SimIMU> imu0, imu1;
    std::shared_ptr<sim::SimBaro> baro;
    std::shared_ptr<sim::SimGPS> gps; // null unless the rig was built with_gps
    std::unique_ptr<vehicles::VehicleManager> vm;
    vehicles::UAVVehicle* uav{nullptr};
    double t{0.0};
    bool rc_on{true};
    float rc[4]{0, 0, 0, 0.5f};   // roll, pitch, yaw, throttle sticks

    /** rotors: 4 = the default quad; 6/8 = hexa/octa-X from the same mixer presets. */
    explicit Rig(bool with_gps = false, uint8_t rotors = 4) {
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
        baro = std::make_shared<sim::SimBaro>(&ctx);
        vm->get_sensors()->register_sensor(baro);
        vm->get_sensors()->register_sensor(std::make_shared<sim::SimCompass>(&ctx));
        if (with_gps) {
            gps = std::make_shared<sim::SimGPS>(&ctx);
            vm->get_sensors()->register_sensor(gps);
        }
        uav = dynamic_cast<vehicles::UAVVehicle*>(vm->get_active_vehicle());
        if (rotors != 4 && uav) {
            vm->configure_uav_rotors(rotors);
            quad.configure_frame(*uav->get_matrix_mixer());   // hexa/octa geometry from the mixer
        }
    }

    /** Kill (health 0) or damage a motor in the simulated airframe. */
    void fail_motor(int i, double health = 0.0) { quad.motor_health[i] = health; }
    /** Feed every motor's RPM to the flight controller, like ESC telemetry would. */
    bool esc_telemetry{false};

    void step() {
        if (rc_on) vm->set_user_inputs(rc[0], rc[1], rc[2], rc[3]);
        if (esc_telemetry && uav) {
            for (int i = 0; i < quad.n_motors; ++i) uav->report_esc_rpm(static_cast<uint8_t>(i), float(quad.rpm(i)));
        }
        vm->step(LOOP_MS * 1e-3f);
        double cmd[sim::QuadModel::MAX_MOTORS]{};
        for (int i = 0; i < quad.n_motors; ++i) {
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
} // namespace sitl
} // namespace autopilot
