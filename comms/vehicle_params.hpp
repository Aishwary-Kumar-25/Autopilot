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
 * Copy values the flight code learned in flight into their parameters, so they are
 * saved and used from the next boot. Today: the rotor-drag coefficient that the
 * fusion calibrates in the air (EKF_DRAG_MCOEF). Only while disarmed, and only when
 * it moved by more than 2 %. Call it from the main loop now and then (e.g. 1 Hz).
 */
void sync_learned_params(ParamStore& store, vehicles::VehicleManager& vm);

/**
 * Flight modes, shared by every vehicle as MAVLink custom_mode 0..2:
 *   UAV  MANUAL / STABILIZE / ALT_HOLD
 *   UGV  MANUAL / HEADING_HOLD / SPEED_HOLD   (boat: same as UGV)
 *   ROV  MANUAL / STABILIZE / DEPTH_HOLD
 */
constexpr uint32_t NUM_VEHICLE_MODES = 3;
bool set_vehicle_mode(vehicles::VehicleManager& vm, uint32_t mode);
uint32_t get_vehicle_mode(vehicles::VehicleManager& vm);
const char* vehicle_mode_name(vehicles::VehicleType type, uint32_t mode);

} // namespace comms
} // namespace autopilot
