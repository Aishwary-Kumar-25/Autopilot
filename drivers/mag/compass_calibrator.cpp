#include "compass_calibrator.hpp"
#include <algorithm>
#include <cmath>

namespace autopilot {
namespace drivers {

void CompassCalibrator::reset() {
    *this = CompassCalibrator{};
}

void CompassCalibrator::add_sample(const math::Vector3f& raw) {
    if (!std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.z)) return;
    const double phi[6] = {double(raw.x) * raw.x, double(raw.y) * raw.y, double(raw.z) * raw.z,
                           double(raw.x), double(raw.y), double(raw.z)};
    for (int i = 0; i < 6; ++i) {
        _b[i] += phi[i];
        for (int j = i; j < 6; ++j) _A[i][j] += phi[i] * phi[j];
    }
    _min = math::Vector3f(std::min(_min.x, raw.x), std::min(_min.y, raw.y), std::min(_min.z, raw.z));
    _max = math::Vector3f(std::max(_max.x, raw.x), std::max(_max.y, raw.y), std::max(_max.z, raw.z));
    ++_n;
}

// Solve the 6x6 symmetric system M p = v with Gaussian elimination + partial pivoting.
static bool solve6(double M[6][6], double v[6], double p[6]) {
    for (int c = 0; c < 6; ++c) {
        int piv = c;
        for (int r = c + 1; r < 6; ++r) if (std::fabs(M[r][c]) > std::fabs(M[piv][c])) piv = r;
        if (std::fabs(M[piv][c]) < 1e-12) return false;
        if (piv != c) {
            for (int k = 0; k < 6; ++k) std::swap(M[c][k], M[piv][k]);
            std::swap(v[c], v[piv]);
        }
        for (int r = c + 1; r < 6; ++r) {
            const double f = M[r][c] / M[c][c];
            for (int k = c; k < 6; ++k) M[r][k] -= f * M[c][k];
            v[r] -= f * v[c];
        }
    }
    for (int r = 5; r >= 0; --r) {
        double s = v[r];
        for (int k = r + 1; k < 6; ++k) s -= M[r][k] * p[k];
        p[r] = s / M[r][r];
    }
    return true;
}

CompassCalibrator::Result CompassCalibrator::fit() const {
    Result res;
    if (_n < MIN_SAMPLES) {
        res.failure = "not enough samples";
        return res;
    }

    double M[6][6], v[6], p[6];
    for (int i = 0; i < 6; ++i) {
        v[i] = _b[i];
        for (int j = 0; j < 6; ++j) M[i][j] = i <= j ? _A[i][j] : _A[j][i];
    }
    double Mcopy[6][6];
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) Mcopy[i][j] = M[i][j];
    if (!solve6(M, v, p)) {
        res.failure = "degenerate samples (rotate through more orientations)";
        return res;
    }
    const double a = p[0], b = p[1], c = p[2];
    if (!(a > 0.0 && b > 0.0 && c > 0.0)) {
        res.failure = "fit is not an ellipsoid";
        return res;
    }
    const double ox = -p[3] / (2.0 * a), oy = -p[4] / (2.0 * b), oz = -p[5] / (2.0 * c);
    // a(x-ox)^2 + b(y-oy)^2 + c(z-oz)^2 = G
    const double G = 1.0 + a * ox * ox + b * oy * oy + c * oz * oz;
    if (!(G > 0.0)) {
        res.failure = "fit is not an ellipsoid";
        return res;
    }
    const double rx = std::sqrt(G / a), ry = std::sqrt(G / b), rz = std::sqrt(G / c);
    const double r = std::cbrt(rx * ry * rz);   // keep the volume: geometric-mean radius

    // Algebraic residual: sum (phi.p - 1)^2 = p'Ap - 2p'b + n. For a point near the
    // surface, phi.p - 1 ~= 2 * (|d|/r - 1) * G-normalised, so halve it.
    double pAp = 0.0, pb = 0.0;
    for (int i = 0; i < 6; ++i) {
        pb += p[i] * _b[i];
        for (int j = 0; j < 6; ++j) pAp += p[i] * Mcopy[i][j] * p[j];
    }
    const double sse = std::max(0.0, pAp - 2.0 * pb + double(_n));
    res.rms_residual = static_cast<float>(0.5 * std::sqrt(sse / double(_n)) / G);

    res.offset = math::Vector3f(float(ox), float(oy), float(oz));
    res.scale = math::Vector3f(float(r / rx), float(r / ry), float(r / rz));
    res.field_radius = float(r);

    // Coverage: every axis must have seen most of its full +-r swing.
    const float span_x = _max.x - _min.x, span_y = _max.y - _min.y, span_z = _max.z - _min.z;
    if (span_x < MIN_COVERAGE * 2.0f * float(rx) || span_y < MIN_COVERAGE * 2.0f * float(ry) ||
        span_z < MIN_COVERAGE * 2.0f * float(rz)) {
        res.failure = "poor coverage (rotate through all orientations)";
        return res;
    }
    if (res.rms_residual > MAX_RMS_RESIDUAL) {
        res.failure = "poor fit (magnetic interference while calibrating?)";
        return res;
    }
    // Earth field is 0.25-0.65 G; allow margin for local anomalies.
    if (r < 0.1 || r > 1.5) {
        res.failure = "field strength out of range";
        return res;
    }
    const float smin = std::min({res.scale.x, res.scale.y, res.scale.z});
    const float smax = std::max({res.scale.x, res.scale.y, res.scale.z});
    if (smin < 0.6f || smax > 1.6f) {
        res.failure = "soft-iron scale implausible";
        return res;
    }
    res.ok = true;
    return res;
}

} // namespace drivers
} // namespace autopilot
