#pragma once

#include "math/vector3.hpp"
#include <cstdint>

namespace autopilot {
namespace drivers {

/**
 * Compass calibration (hardware independent).
 *
 * A magnetometer on a vehicle doesn't measure the Earth field alone: steel parts
 * and wiring add a constant offset (hard iron) and stretch the field differently
 * per axis (soft iron). Rotating the vehicle through all orientations traces an
 * ellipsoid instead of a sphere centred on zero. This fits an axis-aligned
 * ellipsoid
 *     a x^2 + b y^2 + c z^2 + d x + e y + f z = 1
 * by linear least squares, giving
 *     offset = (-d/2a, -e/2b, -f/2c)                       (hard iron)
 *     scale  = per-axis factors that map the ellipsoid to a sphere (soft iron, diagonal)
 * so that  corrected = (raw - offset) * scale.
 *
 * Usage: add_sample() while the user rotates the vehicle, then fit(). The result
 * is rejected if the samples don't cover enough orientations or fit poorly.
 * Memory: fixed 64 bytes of sums (no sample buffer), so it can run on the MCU.
 */
class CompassCalibrator {
public:
    struct Result {
        bool ok{false};
        math::Vector3f offset{0, 0, 0};
        math::Vector3f scale{1, 1, 1};
        float field_radius{0.0f};  // fitted field strength after correction (gauss)
        float rms_residual{0.0f};  // ~rms of (|corrected| - radius) / radius over the samples
        const char* failure{""};
    };

    static constexpr uint32_t MIN_SAMPLES = 100;
    static constexpr float MAX_RMS_RESIDUAL = 0.05f;   // 5 % of the field strength
    static constexpr float MIN_COVERAGE = 0.7f;        // each axis must span >= 0.7 * 2r

    void reset();
    void add_sample(const math::Vector3f& raw);
    uint32_t sample_count() const { return _n; }

    /** Fit and validate. Call after enough samples were added. */
    Result fit() const;

    /** Apply a result to a raw reading. */
    static math::Vector3f apply(const Result& r, const math::Vector3f& raw) {
        const math::Vector3f d = raw - r.offset;
        return math::Vector3f(d.x * r.scale.x, d.y * r.scale.y, d.z * r.scale.z);
    }

private:
    // Normal equations for the 6 unknowns, accumulated per sample:
    //   phi = [x^2, y^2, z^2, x, y, z],  A = sum(phi phi^T),  b = sum(phi)
    double _A[6][6]{};
    double _b[6]{};
    // Per-axis extents for the coverage check.
    math::Vector3f _min{1e9f, 1e9f, 1e9f};
    math::Vector3f _max{-1e9f, -1e9f, -1e9f};
    uint32_t _n{0};
};

} // namespace drivers
} // namespace autopilot
