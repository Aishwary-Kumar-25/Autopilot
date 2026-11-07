#pragma once

#include "state_types.hpp"
#include "madgwick.hpp"
#include "complementary.hpp"
#include "drivers/sensor_manager.hpp"

namespace autopilot {
namespace fusion {

/**
 * @brief Universal Vehicle-Agnostic Sensor Fusion Engine.
 * Fuses any combination of plug-and-play sensors (IMU, Baro, Mag, GPS)
 * into a single unified VehicleState.
 */
class SensorFusion {
public:
    SensorFusion(drivers::SensorManager* sensor_manager);

    void reset();

    /**
     * @brief High-rate state update loop (called at 100Hz - 400Hz).
     */
    void update(float dt);

    const VehicleState& get_state() const { return _state; }

    /**
     * @brief Transform a 3D vector from Body frame to NED frame.
     */
    math::Vector3f body_to_ned(const math::Vector3f& v_body) const;

    /**
     * @brief Transform a 3D vector from NED frame to Body frame.
     */
    math::Vector3f ned_to_body(const math::Vector3f& v_ned) const;

private:
    drivers::SensorManager* _sensors{nullptr};
    MadgwickFilter _attitude_filter{0.15f};
    AltitudeEstimator _alt_estimator{2.5f};
    VehicleState _state;

    void update_attitude(float dt);
    void update_altitude(float dt);
    void update_position(float dt);
};

} // namespace fusion
} // namespace autopilot
