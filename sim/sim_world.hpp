#pragma once
/**
 * SimWorld: the physics side of SITL for ANY vehicle type.
 *
 * Test/SITL only. It owns one SimBody (quad, rover, boat or sub) and keeps it
 * matched to whatever vehicle profile the firmware's VehicleManager is running.
 * When the profile is switched at runtime (VEHICLE_TYPE from the ground station,
 * or ROV_THRUSTERS / UAV_ROTORS changing the frame), the body is swapped for the
 * matching one at the same spot and heading. The simulated sensors read the body
 * through SimContext, so the same sensor objects (and the firmware's single
 * SensorManager + SensorFusion) keep running across the switch.
 */
#include "hal/hal.hpp"
#include "sim/quad_sim.hpp"
#include "sim/sub_sim.hpp"
#include "sim/surface_sim.hpp"
#include "vehicles/vehicle_manager.hpp"
#include <memory>

namespace autopilot {
namespace sim {

class SimWorld {
public:
    SimContext ctx;

    SimWorld(vehicles::VehicleManager& vm, hal::Time* time) : _vm(vm) {
        ctx.time = time;
        sync();
    }

    SimBody& body() { return *_body; }
    const SimBody& body() const { return *_body; }
    template <class T> T* as() { return dynamic_cast<T*>(_body.get()); }

    /** Number of body swaps so far (0 = still the initial body). */
    int swaps() const { return _swaps; }

    /**
     * Match the body to the firmware's active profile. Returns true if it was
     * replaced. Call after anything that may switch the vehicle type.
     */
    bool sync() {
        const auto type = _vm.get_active_type();
        const int frame = frame_size();
        if (_body && type == _type && frame == _frame) return false;
        double n = 0, e = 0, yaw = 0;
        if (_body) {
            n = _body->pos.x;
            e = _body->pos.y;
            yaw = _body->yaw();
            ++_swaps;
        }
        _body = make_body(type, frame);
        _body->place(n, e, yaw);
        _type = type;
        _frame = frame;
        ctx.model = _body.get();
        return true;
    }

    /** One physics step from the PWM the firmware is currently outputting. */
    void step(double dt) {
        hal::HAL* h = _vm.get_hal();
        hal::PWMOutput* pwm = h ? h->get_pwm() : nullptr;
        if (!pwm || !pwm->is_enabled()) {
            _body->step_pwm(nullptr, 0, dt);
            return;
        }
        uint16_t us[hal::PWMOutput::MAX_CHANNELS];
        const int n = _vm.get_active_actuator_count();
        for (int i = 0; i < n; ++i) us[i] = pwm->get_channel(static_cast<uint8_t>(i));
        _body->step_pwm(us, n, dt);
    }

    static std::unique_ptr<SimBody> make_body(vehicles::VehicleType type, int frame) {
        switch (type) {
            case vehicles::VehicleType::UGV_GROUND_ROVER: return std::make_unique<RoverModel>();
            case vehicles::VehicleType::USV_SURFACE_BOAT: return std::make_unique<BoatModel>();
            case vehicles::VehicleType::ROV_SUBMARINE: return std::make_unique<SubModel>(frame);
            case vehicles::VehicleType::UAV_MULTICOPTER:
            default: return std::make_unique<QuadModel>();
        }
    }

private:
    vehicles::VehicleManager& _vm;
    std::unique_ptr<SimBody> _body;
    vehicles::VehicleType _type{vehicles::VehicleType::UAV_MULTICOPTER};
    int _frame{0};
    int _swaps{0};

    int frame_size() { return _vm.get_active_actuator_count(); }
};

} // namespace sim
} // namespace autopilot
