#include "madgwick.hpp"
#include <cmath>

namespace autopilot {
namespace fusion {

MadgwickFilter::MadgwickFilter(float beta) : _beta(beta) {}

void MadgwickFilter::reset() {
    _q = math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

void MadgwickFilter::get_euler_angles(float& roll, float& pitch, float& yaw) const {
    _q.to_euler(roll, pitch, yaw);
}

void MadgwickFilter::update_imu(const math::Vector3f& gyro, const math::Vector3f& accel, float dt) {
    if (dt <= 1e-6f) return;

    float q0 = _q.q0, q1 = _q.q1, q2 = _q.q2, q3 = _q.q3;

    // Rate of change of quaternion from gyroscope
    float qDot0 = 0.5f * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
    float qDot1 = 0.5f * ( q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
    float qDot2 = 0.5f * ( q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
    float qDot3 = 0.5f * ( q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

    // Compute feedback only if accelerometer measurement valid
    float a_norm = accel.length();
    if (a_norm > 1e-4f) {
        float ax = accel.x / a_norm;
        float ay = accel.y / a_norm;
        float az = accel.z / a_norm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        // Gradient descent algorithm corrective step
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float s_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (s_norm > 1e-4f) {
            float s_inv = 1.0f / s_norm;
            s0 *= s_inv;
            s1 *= s_inv;
            s2 *= s_inv;
            s3 *= s_inv;

            // Apply feedback step
            qDot0 -= _beta * s0;
            qDot1 -= _beta * s1;
            qDot2 -= _beta * s2;
            qDot3 -= _beta * s3;
        }
    }

    // Integrate rate of change of quaternion
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    _q = math::Quaternion(q0, q1, q2, q3);
    _q.normalize();
}

void MadgwickFilter::update_9dof(const math::Vector3f& gyro, const math::Vector3f& accel,
                                const math::Vector3f& mag, float dt) {
    float m_norm = mag.length();
    if (m_norm < 1e-4f) {
        update_imu(gyro, accel, dt);
        return;
    }

    float q0 = _q.q0, q1 = _q.q1, q2 = _q.q2, q3 = _q.q3;

    float qDot0 = 0.5f * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
    float qDot1 = 0.5f * ( q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
    float qDot2 = 0.5f * ( q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
    float qDot3 = 0.5f * ( q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

    float a_norm = accel.length();
    if (a_norm > 1e-4f) {
        float ax = accel.x / a_norm;
        float ay = accel.y / a_norm;
        float az = accel.z / a_norm;

        float mx = mag.x / m_norm;
        float my = mag.y / m_norm;
        float mz = mag.z / m_norm;

        // Reference direction of Earth's magnetic field
        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;

        float hx = mx * q0 * q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1 * q1 + 2.0f * q1 * my * q2 + 2.0f * q1 * mz * q3 - mx * q2 * q2 - mx * q3 * q3;
        float hy = _2q0mx * q3 + my * q0 * q0 - _2q0mz * q1 + 2.0f * q1 * mx * q2 - my * q1 * q1 + my * q2 * q2 + 2.0f * q2 * mz * q3 - my * q3 * q3;
        float _2bx = std::sqrt(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0 * q0 + 2.0f * q1 * mx * q3 - mz * q1 * q1 + 2.0f * q2 * my * q3 - mz * q2 * q2 + mz * q3 * q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        // Gradient descent algorithm
        float s0 = -2.0f * q2 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q1 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q2 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
        float s1 = 2.0f * q3 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q0 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
        float s2 = -2.0f * q0 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q3 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
        float s3 = 2.0f * q1 * (2.0f * q1 * q3 - 2.0f * q0 * q2 - ax) + 2.0f * q2 * (2.0f * q0 * q1 + 2.0f * q2 * q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q1 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);

        float s_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (s_norm > 1e-4f) {
            float s_inv = 1.0f / s_norm;
            s0 *= s_inv;
            s1 *= s_inv;
            s2 *= s_inv;
            s3 *= s_inv;

            qDot0 -= _beta * s0;
            qDot1 -= _beta * s1;
            qDot2 -= _beta * s2;
            qDot3 -= _beta * s3;
        }
    }

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    _q = math::Quaternion(q0, q1, q2, q3);
    _q.normalize();
}

} // namespace fusion
} // namespace autopilot
