#pragma once

#include "vector3.hpp"
#include <cmath>

namespace autopilot {
namespace math {

class Quaternion {
public:
    float q0{1.0f}; // scalar (w)
    float q1{0.0f}; // x
    float q2{0.0f}; // y
    float q3{0.0f}; // z

    constexpr Quaternion() = default;
    constexpr Quaternion(float w, float x, float y, float z) : q0(w), q1(x), q2(y), q3(z) {}

    static Quaternion from_euler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);

        Quaternion q;
        q.q0 = cr * cp * cy + sr * sp * sy;
        q.q1 = sr * cp * cy - cr * sp * sy;
        q.q2 = cr * sp * cy + sr * cp * sy;
        q.q3 = cr * cp * sy - sr * sp * cy;
        return q;
    }

    void to_euler(float& roll, float& pitch, float& yaw) const {
        // roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (std::abs(sinp) >= 1.0f) {
            pitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sinp); // clamp to 90 deg
        } else {
            pitch = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    Quaternion operator*(const Quaternion& r) const {
        return Quaternion(
            q0 * r.q0 - q1 * r.q1 - q2 * r.q2 - q3 * r.q3,
            q0 * r.q1 + q1 * r.q0 + q2 * r.q3 - q3 * r.q2,
            q0 * r.q2 - q1 * r.q3 + q2 * r.q0 + q3 * r.q1,
            q0 * r.q3 + q1 * r.q2 - q2 * r.q1 + q3 * r.q0
        );
    }

    Quaternion conjugate() const {
        return Quaternion(q0, -q1, -q2, -q3);
    }

    float norm() const {
        return std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    }

    void normalize() {
        float n = norm();
        if (n > 1e-6f) {
            float inv = 1.0f / n;
            q0 *= inv;
            q1 *= inv;
            q2 *= inv;
            q3 *= inv;
        } else {
            q0 = 1.0f;
            q1 = 0.0f;
            q2 = 0.0f;
            q3 = 0.0f;
        }
    }

    Vector3f rotate_vector(const Vector3f& v) const {
        // v' = q * v * q^-1
        Quaternion vq(0.0f, v.x, v.y, v.z);
        Quaternion res = (*this) * vq * conjugate();
        return Vector3f(res.q1, res.q2, res.q3);
    }
};

} // namespace math
} // namespace autopilot
