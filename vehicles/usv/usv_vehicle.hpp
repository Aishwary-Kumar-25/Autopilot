#pragma once

#include "vehicles/ugv/ugv_vehicle.hpp"

namespace autopilot {
namespace vehicles {

/**
 * Surface boat (USV). A boat steers and holds speed like a rover, so it runs the
 * same UGV controller (MANUAL / HEADING_HOLD / SPEED_HOLD) on the same shared
 * sensor stack; only the actuators differ. Default is twin thrusters
 * (catamaran-style differential thrust: ch0 port, ch1 starboard), which can turn
 * on the spot; a rudder + single prop boat uses ACKERMANN (ch0 rudder, ch1 motor).
 *
 * This is how ArduPilot Rover handles boats (FRAME_CLASS = boat): one vehicle
 * family, different frame. Here it is a separate VehicleType so the ground station
 * shows a boat (MAV_TYPE_SURFACE_BOAT) and the type can be switched at runtime.
 */
class USVVehicle : public UGVVehicle {
public:
    explicit USVVehicle(hal::HAL* hal, UGVSteeringType steering_type = UGVSteeringType::SKID_STEER)
        : UGVVehicle(hal, steering_type, VehicleType::USV_SURFACE_BOAT) {}

    const char* get_type_name() const override { return "USV Surface Boat"; }
};

} // namespace vehicles
} // namespace autopilot
