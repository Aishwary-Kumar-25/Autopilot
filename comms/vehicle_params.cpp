#include "comms/vehicle_params.hpp"

namespace autopilot {
namespace comms {

using vehicles::UAVPid;
using vehicles::UAVVehicle;
using vehicles::VehicleManager;
using vehicles::VehicleType;

namespace {

enum class Gain : uint8_t { P, I, D };

struct PidParam {
    const char* name;
    UAVPid loop;
    Gain gain;
    float fallback;   // used when the active vehicle isn't a UAV at registration
    float max;
};

// Defaults mirror UAVVehicle::setup_pids(); the live UAV's gains win if present.
const PidParam PID_PARAMS[] = {
    {"MC_ROLL_P",       UAVPid::ROLL_ANGLE,  Gain::P, 4.5f,   20.0f},
    {"MC_PITCH_P",      UAVPid::PITCH_ANGLE, Gain::P, 4.5f,   20.0f},
    {"MC_ROLLRATE_P",   UAVPid::ROLL_RATE,   Gain::P, 0.15f,  2.0f},
    {"MC_ROLLRATE_I",   UAVPid::ROLL_RATE,   Gain::I, 0.10f,  2.0f},
    {"MC_ROLLRATE_D",   UAVPid::ROLL_RATE,   Gain::D, 0.004f, 0.1f},
    {"MC_PITCHRATE_P",  UAVPid::PITCH_RATE,  Gain::P, 0.15f,  2.0f},
    {"MC_PITCHRATE_I",  UAVPid::PITCH_RATE,  Gain::I, 0.10f,  2.0f},
    {"MC_PITCHRATE_D",  UAVPid::PITCH_RATE,  Gain::D, 0.004f, 0.1f},
    {"MC_YAWRATE_P",    UAVPid::YAW_RATE,    Gain::P, 0.20f,  2.0f},
    {"MC_YAWRATE_I",    UAVPid::YAW_RATE,    Gain::I, 0.05f,  2.0f},
    {"MC_ALT_P",        UAVPid::ALTITUDE,    Gain::P, 1.0f,   5.0f},
    {"MC_CLIMB_P",      UAVPid::CLIMB_RATE,  Gain::P, 0.3f,   2.0f},
    {"MC_CLIMB_I",      UAVPid::CLIMB_RATE,  Gain::I, 0.1f,   2.0f},
    {"MC_CLIMB_D",      UAVPid::CLIMB_RATE,  Gain::D, 0.02f,  0.5f},
};

UAVVehicle* active_uav(VehicleManager& vm) { return dynamic_cast<UAVVehicle*>(vm.get_active_vehicle()); }
vehicles::ROVVehicle* active_rov(VehicleManager& vm) {
    return dynamic_cast<vehicles::ROVVehicle*>(vm.get_active_vehicle());
}

float read_gain(const math::PID& pid, Gain g) {
    const auto& c = pid.config();
    return g == Gain::P ? c.kp : g == Gain::I ? c.ki : c.kd;
}

} // namespace

bool set_vehicle_mode(VehicleManager& vm, uint32_t mode) {
    if (mode >= NUM_VEHICLE_MODES) return false;
    auto* v = vm.get_active_vehicle();
    if (auto* u = dynamic_cast<UAVVehicle*>(v)) u->set_mode(static_cast<vehicles::UAVFlightMode>(mode));
    else if (auto* g = dynamic_cast<vehicles::UGVVehicle*>(v)) g->set_mode(static_cast<vehicles::UGVMode>(mode));
    else if (auto* r = dynamic_cast<vehicles::ROVVehicle*>(v)) r->set_mode(static_cast<vehicles::ROVMode>(mode));
    else return false;
    return true;
}

uint32_t get_vehicle_mode(VehicleManager& vm) {
    auto* v = vm.get_active_vehicle();
    if (auto* u = dynamic_cast<UAVVehicle*>(v)) return static_cast<uint32_t>(u->get_mode());
    if (auto* g = dynamic_cast<vehicles::UGVVehicle*>(v)) return static_cast<uint32_t>(g->get_mode());
    if (auto* r = dynamic_cast<vehicles::ROVVehicle*>(v)) return static_cast<uint32_t>(r->get_mode());
    return 0;
}

const char* vehicle_mode_name(VehicleType type, uint32_t mode) {
    static const char* const UAV[] = {"MANUAL", "STABILIZE", "ALT_HOLD"};
    static const char* const UGV[] = {"MANUAL", "HEADING_HOLD", "SPEED_HOLD"};
    static const char* const ROV[] = {"MANUAL", "STABILIZE", "DEPTH_HOLD"};
    if (mode >= NUM_VEHICLE_MODES) return "UNKNOWN";
    switch (type) {
        case VehicleType::UGV_GROUND_ROVER: return UGV[mode];
        case VehicleType::ROV_SUBMARINE: return ROV[mode];
        case VehicleType::UAV_MULTICOPTER:
        default: return UAV[mode];
    }
}

void register_vehicle_params(ParamStore& store, VehicleManager& vm) {
    using T = ParamStore::Type;
    VehicleManager* m = &vm;

    // Flight mode as a parameter too: works from any GCS's parameter editor, even
    // ones that only know their own firmware's mode names.
    store.add_live("FLTMODE", T::INT32, 0, NUM_VEHICLE_MODES - 1,
        [m] { return static_cast<float>(get_vehicle_mode(*m)); },
        [m](float v) { return set_vehicle_mode(*m, static_cast<uint32_t>(v)); });

    // ---- vehicle profile / frame (live: other code can change these too) ----
    store.add_live("VEHICLE_TYPE", T::INT32, 0, 2,
        [m] { return static_cast<float>(m->get_active_type()); },
        [m](float v) {
            if (m->is_armed()) return false;   // never swap the controller of a flying vehicle
            std::string reason;
            return m->switch_vehicle(static_cast<VehicleType>(static_cast<int>(v)), reason);
        });
    store.add_live("UAV_ROTORS", T::INT32, 3, 16,
        [m] { auto* u = active_uav(*m); return u ? static_cast<float>(u->get_rotor_count()) : 4.0f; },
        [m](float v) {
            auto* u = active_uav(*m);
            if (!u || m->is_armed()) return false;
            return u->set_rotor_count(static_cast<uint8_t>(v));
        });
    store.add_live("ROV_THRUSTERS", T::INT32, 2, 16,
        [m] { auto* r = active_rov(*m); return r ? static_cast<float>(r->get_thruster_count()) : 6.0f; },
        [m](float v) {
            auto* r = active_rov(*m);
            if (!r || m->is_armed()) return false;
            return r->set_thruster_count(static_cast<uint8_t>(v));
        });

    // ---- failsafe (applies to whichever vehicle is active) ----
    const auto* veh = vm.get_active_vehicle();
    const control::Failsafe::Config fs = veh ? veh->get_failsafe().config() : control::Failsafe::Config{};
    store.add("FS_RC_TIMEOUT", T::INT32, static_cast<float>(fs.rc_timeout_ms), 100, 5000, [m](float v) {
        if (auto* a = m->get_active_vehicle()) {
            auto c = a->get_failsafe().config();
            c.rc_timeout_ms = static_cast<uint32_t>(v);
            a->get_failsafe().set_config(c);
        }
        return true;
    });
    store.add("FS_BATT_LOW_V", T::FLOAT, fs.battery_low_v, 0.0f, 60.0f, [m](float v) {
        if (auto* a = m->get_active_vehicle()) {
            auto c = a->get_failsafe().config();
            c.battery_low_v = v;
            a->get_failsafe().set_config(c);
        }
        return true;
    });

    // ---- multicopter tuning (stored; applied whenever a UAV is active) ----
    UAVVehicle* uav = active_uav(vm);
    store.add("MC_HOVER_THR", T::FLOAT, uav ? uav->get_hover_throttle() : 0.5f, 0.15f, 0.85f, [m](float v) {
        if (auto* u = active_uav(*m)) u->set_hover_throttle(v);
        return true;
    });
    for (const PidParam& p : PID_PARAMS) {
        const float def = uav ? read_gain(uav->pid(p.loop), p.gain) : p.fallback;
        const UAVPid loop = p.loop;
        const Gain gain = p.gain;
        store.add(p.name, T::FLOAT, def, 0.0f, p.max, [m, loop, gain](float v) {
            if (auto* u = active_uav(*m)) {
                math::PID& pid = u->pid(loop);
                const auto c = pid.config();
                pid.set_gains(gain == Gain::P ? v : c.kp, gain == Gain::I ? v : c.ki,
                              gain == Gain::D ? v : c.kd, c.kff);
            }
            return true;
        });
    }
}

} // namespace comms
} // namespace autopilot
