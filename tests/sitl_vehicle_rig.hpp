#pragma once
/**
 * SITL rig for every vehicle type: ONE VehicleManager (one firmware build, one
 * SensorManager, one SensorFusion) driving sim::SimWorld, which swaps the physics
 * body (quad / rover / boat / sub) whenever the firmware switches profile.
 */
#include "hal/posix/posix_hal.hpp"
#include "sim/sim_world.hpp"
#include "vehicles/vehicle_manager.hpp"
#include <cmath>
#include <memory>
#include <string>

namespace autopilot {
namespace sitl {

struct VehicleRig {
    static constexpr uint32_t LOOP_MS = 4;      // 250 Hz flight loop
    static constexpr int SUBSTEPS = 4;          // 1 kHz physics
    static constexpr double RAD2DEG = 57.29577951;

    hal::PosixHAL hal;
    std::unique_ptr<vehicles::VehicleManager> vm;
    std::unique_ptr<sim::SimWorld> world;
    std::shared_ptr<sim::SimGPS> gps;
    double t{0.0};
    // roll, pitch, yaw, throttle, forward, lateral sticks
    float rc[6]{0, 0, 0, 0, 0, 0};
    double max_tilt_err_deg{0.0};   // worst roll/pitch estimate error seen (tracked while armed)

    explicit VehicleRig(vehicles::VehicleType type) {
        hal.time().set_manual_time_mode(true);
        hal.time().advance_time_ms(1000);
        vm = std::make_unique<vehicles::VehicleManager>(&hal, type);
        vm->init();
        world = std::make_unique<sim::SimWorld>(*vm, &hal.time());
        auto* sm = vm->get_sensors();
        sm->register_sensor(std::make_shared<sim::SimIMU>(&world->ctx, "SIM_IMU0"));
        sm->register_sensor(std::make_shared<sim::SimIMU>(&world->ctx, "SIM_IMU1"));
        sm->register_sensor(std::make_shared<sim::SimBaro>(&world->ctx));
        sm->register_sensor(std::make_shared<sim::SimCompass>(&world->ctx));
        gps = std::make_shared<sim::SimGPS>(&world->ctx);
        sm->register_sensor(gps);
    }

    sim::SimBody& body() { return world->body(); }
    template <class V> V* vehicle() { return dynamic_cast<V*>(vm->get_active_vehicle()); }

    void step() {
        vm->set_user_inputs(rc[0], rc[1], rc[2], rc[3], rc[4], rc[5]);
        vm->step(LOOP_MS * 1e-3f);
        world->sync();
        for (int s = 0; s < SUBSTEPS; ++s) world->step(LOOP_MS * 1e-3 / SUBSTEPS);
        hal.time().advance_time_ms(LOOP_MS);
        t += LOOP_MS * 1e-3;
        if (vm->is_armed()) max_tilt_err_deg = std::fmax(max_tilt_err_deg, tilt_err_deg());
    }
    void run(double seconds) {
        const double end = t + seconds;
        while (t < end) step();
    }
    void neutral() { for (float& c : rc) c = 0.0f; }

    bool arm() {
        std::string err;
        const bool ok = vm->arm(err);
        if (!ok) std::printf("  arm refused: %s\n", err.c_str());
        return ok;
    }
    bool switch_to(vehicles::VehicleType type) {
        std::string why;
        const bool ok = vm->switch_vehicle(type, why);
        world->sync();
        return ok;
    }

    // ---- truth vs estimate ----
    static double wrap(double a) {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
    double tilt_err_deg() const {
        const auto& s = vm->get_latest_state();
        const auto& b = world->body();
        return std::fmax(std::fabs(wrap(s.euler_rad.x - b.roll())), std::fabs(wrap(s.euler_rad.y - b.pitch()))) * RAD2DEG;
    }
    double heading_err_deg() const {
        return std::fabs(wrap(vm->get_latest_state().heading_rad - world->body().yaw())) * RAD2DEG;
    }
    double yaw_deg() const { return world->body().yaw() * RAD2DEG; }
    double depth() const { return world->body().pos.z; }
};

} // namespace sitl
} // namespace autopilot
