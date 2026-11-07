#pragma once
/**
 * The ground-station-visible parameter set for the flight stack: vehicle
 * profile, frame size, failsafe thresholds and the multicopter PID gains.
 *
 * Stored values are pushed into whatever vehicle is active; GcsLink calls
 * ParamStore::apply_all() when the active vehicle object changes, so tuning
 * survives a profile switch. Profile / frame changes are refused while armed.
 */
#include "comms/params.hpp"
#include "vehicles/vehicle_manager.hpp"

namespace autopilot {
namespace comms {

void register_vehicle_params(ParamStore& store, vehicles::VehicleManager& vm);

/**
 * Flight modes, shared by every vehicle as MAVLink custom_mode 0..2:
 *   UAV  MANUAL / STABILIZE / ALT_HOLD
 *   UGV  MANUAL / HEADING_HOLD / SPEED_HOLD
 *   ROV  MANUAL / STABILIZE / DEPTH_HOLD
 */
constexpr uint32_t NUM_VEHICLE_MODES = 3;
bool set_vehicle_mode(vehicles::VehicleManager& vm, uint32_t mode);
uint32_t get_vehicle_mode(vehicles::VehicleManager& vm);
const char* vehicle_mode_name(vehicles::VehicleType type, uint32_t mode);

} // namespace comms
} // namespace autopilot
