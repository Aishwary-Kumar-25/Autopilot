#pragma once

#include <cmath>
#include <cstdint>

namespace autopilot {
namespace math {

template <typename T = float>
class Vector3 {
public:
    T x{0};
    T y{0};
    T z{0};

    constexpr Vector3() = default;
    constexpr Vector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    Vector3 operator*(T scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    Vector3 operator/(T scalar) const {
        T inv = static_cast<T>(1) / scalar;
        return Vector3(x * inv, y * inv, z * inv);
    }

    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3& operator*=(T scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    T dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    T length_squared() const {
        return x * x + y * y + z * z;
    }

    T length() const {
        return std::sqrt(length_squared());
    }

    void normalize() {
        T len = length();
        if (len > static_cast<T>(1e-6)) {
            T inv = static_cast<T>(1) / len;
            x *= inv;
            y *= inv;
            z *= inv;
        }
    }

    Vector3 normalized() const {
        Vector3 v = *this;
        v.normalize();
        return v;
    }

    void zero() {
        x = 0;
        y = 0;
        z = 0;
    }
};

using Vector3f = Vector3<float>;

} // namespace math
} // namespace autopilot
