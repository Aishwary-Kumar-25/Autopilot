#pragma once

#include "math/vector3.hpp"
#include "math/quaternion.hpp"

namespace autopilot {
namespace fusion {

class MadgwickFilter {
public:
    MadgwickFilter(float beta = 0.1f);

    void set_beta(float beta) { _beta = beta; }
    float get_beta() const { return _beta; }

    void reset();

    /**
     * @brief 6-DOF update using Gyro and Accel only.
     */
    void update_imu(const math::Vector3f& gyro, const math::Vector3f& accel, float dt);

    /**
     * @brief 9-DOF update using Gyro, Accel, and Magnetometer.
     */
    void update_9dof(const math::Vector3f& gyro, const math::Vector3f& accel,
                     const math::Vector3f& mag, float dt);

    const math::Quaternion& get_quaternion() const { return _q; }
    void get_euler_angles(float& roll, float& pitch, float& yaw) const;

private:
    math::Quaternion _q{1.0f, 0.0f, 0.0f, 0.0f};
    float _beta{0.1f}; // Algorithm gain
};

} // namespace fusion
} // namespace autopilot
