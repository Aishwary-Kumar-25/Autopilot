#pragma once

#include "vector3.hpp"

namespace autopilot {
namespace math {

class Matrix3f {
public:
    float m[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    constexpr Matrix3f() = default;

    static Matrix3f identity() {
        return Matrix3f();
    }

    Vector3f operator*(const Vector3f& v) const {
        return Vector3f(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    Matrix3f transpose() const {
        Matrix3f res;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                res.m[i][j] = m[j][i];
            }
        }
        return res;
    }
};

} // namespace math
} // namespace autopilot
