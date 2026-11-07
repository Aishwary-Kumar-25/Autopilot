#pragma once

#include "math/vector3.hpp"
#include "math/quaternion.hpp"
#include <cstdint>

namespace autopilot {
namespace fusion {

struct VehicleState {
    // Attitude
    math::Quaternion attitude_q{1.0f, 0.0f, 0.0f, 0.0f};
    math::Vector3f euler_rad{0.0f, 0.0f, 0.0f};       // Roll, Pitch, Yaw (radians)
    math::Vector3f angular_velocity{0.0f, 0.0f, 0.0f}; // p, q, r (rad/s in body frame)

    // Acceleration
    math::Vector3f linear_accel_body{0.0f, 0.0f, 0.0f}; // Body frame (m/s^2)
    math::Vector3f linear_accel_ned{0.0f, 0.0f, 0.0f};  // Earth NED frame (m/s^2)

    // Altitude & Vertical Dynamics
    float altitude_m{0.0f};          // Relative to ground / reference
    float vertical_velocity_m_s{0.0f}; // Upwards positive (+), downwards negative (-)

    // Position & Velocity (NED)
    math::Vector3f position_ned{0.0f, 0.0f, 0.0f};  // North, East, Down (meters)
    math::Vector3f velocity_ned{0.0f, 0.0f, 0.0f};  // North, East, Down (m/s)

    // Heading
    float heading_rad{0.0f};

    // Quality flags
    bool attitude_valid{false};
    bool altitude_valid{false};
    bool position_valid{false};
    uint8_t attitude_source{0};   // 0 = primary (Madgwick), 1 = backup (complementary)
    uint8_t tilt_aiding{0};       // primary tilt reference: 0 = accelerometer, 1 = rotor-drag model, 2 = GPS
    math::Vector3f gyro_bias{0.0f, 0.0f, 0.0f}; // online gyro bias estimate (rad/s), already removed
    uint32_t last_update_ms{0};
};

} // namespace fusion
} // namespace autopilot
