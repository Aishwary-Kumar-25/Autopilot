#include "ahrs_interface.hpp"
#include <cmath>

namespace autopilot {
namespace fusion {

static constexpr float G = 9.80665f;
static constexpr float PI_F = 3.14159265f;

static float wrap_pi(float a) {
    while (a > PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

// ---------------- backup complementary filter ----------------

void ComplementaryAttitude::tilt_from_accel(const math::Vector3f& a, float& r, float& p) {
    // specific force at rest is -g along body "down", so gravity direction = -a
    r = std::atan2(-a.y, -a.z);
    p = std::atan2(a.x, std::sqrt(a.y * a.y + a.z * a.z));
}

void ComplementaryAttitude::update(const math::Vector3f& g, const math::Vector3f& a, const math::Vector3f* mag, float dt) {
    // Euler-rate kinematics
    const float sr = std::sin(roll), cr = std::cos(roll);
    float cp = std::cos(pitch);
    if (std::fabs(cp) < 0.05f) cp = cp < 0 ? -0.05f : 0.05f;
    const float tp = std::sin(pitch) / cp;
    roll += (g.x + sr * tp * g.y + cr * tp * g.z) * dt;
    pitch += (cr * g.y - sr * g.z) * dt;
    yaw += (sr / cp * g.y + cr / cp * g.z) * dt;

    const float alpha = tau_s > 0.0f ? tau_s / (tau_s + dt) : 0.0f;
    const float an = a.length();
    if (an > 0.5f * G && an < 1.5f * G) {
        float ra, pa;
        tilt_from_accel(a, ra, pa);
        roll += (1.0f - alpha) * wrap_pi(ra - roll);
        pitch += (1.0f - alpha) * (pa - pitch);
    }
    if (mag && mag->length() > 0.05f) {
        // tilt-compensated heading
        const float sr2 = std::sin(roll), cr2 = std::cos(roll), sp2 = std::sin(pitch), cp2 = std::cos(pitch);
        const float mx = mag->x * cp2 + mag->y * sr2 * sp2 + mag->z * cr2 * sp2;
        const float my = mag->y * cr2 - mag->z * sr2;
        const float yaw_m = std::atan2(-my, mx);
        yaw += (1.0f - alpha) * wrap_pi(yaw_m - yaw);
    }
    roll = wrap_pi(roll);
    yaw = wrap_pi(yaw);
}

// ---------------- SensorFusion ----------------

SensorFusion::SensorFusion(drivers::SensorManager* sensor_manager)
    : _sensors(sensor_manager) {}

void SensorFusion::reset() {
    _attitude_filter.reset();
    _backup = ComplementaryAttitude{};
    _alt_estimator.reset(0.0f);
    _state = VehicleState{};
    _aligned = false;
    _disagree_time = 0.0f;
    _source = 0;
    _have_home = false;
    _baro_ref_set = false;
    _baro_ref_src = nullptr;
    _last_gps_ms = 0;
    _gps_lin_accel = math::Vector3f(0, 0, 0);
    for (int i = 0; i < SF_BUCKETS; ++i) {
        _sf_sum[i] = math::Vector3f(0, 0, 0);
        _sf_slot[i] = 0;
    }
    _gps_count = _gps_head = 0;
    _have_gps_corr = false;
    _gps_aiding = false;
}

math::Vector3f SensorFusion::body_to_ned(const math::Vector3f& v_body) const {
    return _state.attitude_q.rotate_vector(v_body);
}

math::Vector3f SensorFusion::ned_to_body(const math::Vector3f& v_ned) const {
    return _state.attitude_q.conjugate().rotate_vector(v_ned);
}

void SensorFusion::update_attitude(float dt) {
    if (!_sensors) return;

    auto* imu = _sensors->get_active_imu();
    if (!imu || !_sensors->has_healthy_imu()) {
        _state.attitude_valid = false;
        return;
    }

    const auto& gyro = imu->get_gyro();
    const auto& accel = imu->get_accel();
    _now_ms = imu->last_update_ms();
    _state.angular_velocity = gyro;
    _state.linear_accel_body = accel;

    auto* compass = _sensors->get_active_compass();
    const bool use_mag = compass && _sensors->has_healthy_compass();
    const math::Vector3f mag = use_mag ? compass->get_field_gauss() : math::Vector3f(0, 0, 0);
    _mag_in_use = use_mag;

    if (!_aligned) {
        if (accel.length() < 0.5f * G) return;
        ComplementaryAttitude init;
        init.tau_s = 0.0f;
        init.update(math::Vector3f(0, 0, 0), accel, use_mag ? &mag : nullptr, 0.0f);
        _attitude_filter.set_quaternion(math::Quaternion::from_euler(init.roll, init.pitch, init.yaw));
        _backup.roll = init.roll;
        _backup.pitch = init.pitch;
        _backup.yaw = init.yaw;
        ComplementaryAttitude::tilt_from_accel(accel, _acc_roll_lp, _acc_pitch_lp);
        _aligned = true;
        _time_since_align = 0.0f;
        _attitude_filter.set_beta(BETA_ALIGN);
    }
    _time_since_align += dt;
    if (_time_since_align > ALIGN_SETTLE_S) _attitude_filter.set_beta(BETA_FLIGHT);

    // GPS-aided while time-aligned GPS tilt corrections keep arriving.
    _gps_aiding = _have_gps_corr && _time_since_align > ALIGN_SETTLE_S &&
                  _now_ms - _last_gps_corr_ms < GPS_AIDING_TIMEOUT_MS;

    // Manoeuvre acceleration from GPS velocity, removed from the accelerometer for the
    // backup filter and the selector's tilt reference.
    const math::Vector3f lin_body = _attitude_filter.get_quaternion().conjugate().rotate_vector(_gps_lin_accel);
    const math::Vector3f accel_ref = accel - lin_body;

    if (_gps_aiding) {
        // Gyro only here; tilt comes from gps_tilt_correction(), heading from the mag.
        _attitude_filter.update_imu(gyro, math::Vector3f(0, 0, 0), dt);
        if (use_mag) mag_heading_correction(mag, dt);
    } else {
        // Madgwick expects the measured direction of gravity ("down" in body) = -specific force.
        const math::Vector3f down = -accel_ref;
        if (use_mag) {
            _attitude_filter.update_9dof(gyro, down, mag, dt);
        } else {
            _attitude_filter.update_imu(gyro, down, dt);
        }
    }
    _backup.update(gyro, accel_ref, use_mag ? &mag : nullptr, dt);

    // Integrate the earth-frame specific force for the GPS tilt comparison.
    {
        const math::Vector3f f_ned = _attitude_filter.get_quaternion().rotate_vector(accel);
        const uint32_t slot = _now_ms / SF_BUCKET_MS + 1;
        const int i = static_cast<int>(slot % SF_BUCKETS);
        if (_sf_slot[i] != slot) {
            _sf_slot[i] = slot;
            _sf_sum[i] = math::Vector3f(0, 0, 0);
        }
        if (std::isfinite(f_ned.x) && std::isfinite(f_ned.y) && std::isfinite(f_ned.z)) _sf_sum[i] += f_ned * dt;
    }

    // ---- selector ----
    math::Quaternion q_primary = _attitude_filter.get_quaternion();
    const bool primary_ok = std::isfinite(q_primary.q0) && std::isfinite(q_primary.q1) &&
                            std::isfinite(q_primary.q2) && std::isfinite(q_primary.q3);
    float ra, pa;
    ComplementaryAttitude::tilt_from_accel(accel_ref, ra, pa);
    const float k = std::min(1.0f, dt / 1.0f); // ~1 s low-pass on the accel tilt reference
    _acc_roll_lp += k * wrap_pi(ra - _acc_roll_lp);
    _acc_pitch_lp += k * (pa - _acc_pitch_lp);

    uint8_t src = _source;
    if (!primary_ok) {
        src = 1;
        _attitude_filter.set_quaternion(math::Quaternion::from_euler(_backup.roll, _backup.pitch, _backup.yaw));
    } else {
        float pr, pp, py;
        q_primary.to_euler(pr, pp, py);
        const float disagree = std::max(std::fabs(wrap_pi(pr - _backup.roll)), std::fabs(pp - _backup.pitch));
        _disagree_time = disagree > SWITCH_DEG * PI_F / 180.0f ? _disagree_time + dt : 0.0f;
        if (_disagree_time > SWITCH_TIME_S) {
            const float err_p = std::fabs(wrap_pi(pr - _acc_roll_lp)) + std::fabs(pp - _acc_pitch_lp);
            const float err_b = std::fabs(wrap_pi(_backup.roll - _acc_roll_lp)) + std::fabs(_backup.pitch - _acc_pitch_lp);
            src = err_b < err_p ? 1 : 0;
        } else if (_disagree_time == 0.0f) {
            src = 0; // estimators agree again: prefer primary
        }
    }
    if (src != _source) {
        _source = src;
        ++_switches;
    }

    _state.attitude_q = _source ? math::Quaternion::from_euler(_backup.roll, _backup.pitch, _backup.yaw) : q_primary;
    _state.attitude_q.to_euler(_state.euler_rad.x, _state.euler_rad.y, _state.euler_rad.z);
    _state.attitude_source = _source;
    _state.heading_rad = _state.euler_rad.z < 0.0f ? _state.euler_rad.z + 2.0f * PI_F : _state.euler_rad.z;

    // Earth-frame (NED) acceleration: rotate specific force and add gravity back.
    math::Vector3f earth_accel = body_to_ned(accel);
    earth_accel.z += G;
    _state.linear_accel_ned = earth_accel;

    _state.attitude_valid = std::isfinite(_state.euler_rad.x) && std::isfinite(_state.euler_rad.y);
}

void SensorFusion::mag_heading_correction(const math::Vector3f& mag, float dt) {
    // Tilt-compensated magnetic heading, applied as a rotation about earth "down" only,
    // so magnetic disturbances can never tilt the attitude.
    math::Quaternion q = _attitude_filter.get_quaternion();
    float roll, pitch, yaw;
    q.to_euler(roll, pitch, yaw);
    const float sr = std::sin(roll), cr = std::cos(roll), sp = std::sin(pitch), cp = std::cos(pitch);
    const float mx = mag.x * cp + mag.y * sr * sp + mag.z * cr * sp;
    const float my = mag.y * cr - mag.z * sr;
    if (mx * mx + my * my < 1e-6f) return;
    const float err = wrap_pi(std::atan2(-my, mx) - yaw);
    const float half = 0.5f * std::min(1.0f, MAG_YAW_GAIN * dt) * err;
    const math::Quaternion dq(std::cos(half), 0.0f, 0.0f, std::sin(half));
    q = dq * q;
    q.normalize();
    _attitude_filter.set_quaternion(q);
}

void SensorFusion::gps_tilt_correction(const GpsVel& k) {
    // Store the sample, then look back for one ~GPS_WINDOW_MS older.
    const GpsVel* prev_sample = _gps_count ? &_gps_hist[(_gps_head + GPS_HIST - 1) % GPS_HIST] : nullptr;
    const uint32_t gps_dt_ms = prev_sample ? k.t_ms - prev_sample->t_ms : 0;
    _gps_hist[_gps_head] = k;
    _gps_head = (_gps_head + 1) % GPS_HIST;
    if (_gps_count < GPS_HIST) ++_gps_count;
    if (!prev_sample || gps_dt_ms == 0 || gps_dt_ms > GPS_MAX_WINDOW_MS) return;

    const GpsVel* j = nullptr;
    for (int n = 1; n < _gps_count; ++n) {
        const GpsVel& c = _gps_hist[(_gps_head + GPS_HIST - 1 - n) % GPS_HIST];
        const uint32_t age = k.t_ms - c.t_ms;
        if (age >= GPS_WINDOW_MS) {
            if (age <= GPS_MAX_WINDOW_MS) j = &c;
            break;
        }
    }
    if (!j || j->t_ms < _gps_delay_ms) return;

    // Earth-frame specific force integrated over the same (latency-shifted) window.
    const uint32_t s0 = (j->t_ms - _gps_delay_ms) / SF_BUCKET_MS + 1;
    const uint32_t s1 = (k.t_ms - _gps_delay_ms) / SF_BUCKET_MS + 1;
    if (s1 <= s0 || s1 - s0 >= static_cast<uint32_t>(SF_BUCKETS)) return;
    math::Vector3f f_ins(0, 0, 0);
    for (uint32_t slot = s0; slot < s1; ++slot) {
        const int i = static_cast<int>(slot % SF_BUCKETS);
        if (_sf_slot[i] != slot) return; // gap in IMU data: skip this comparison
        f_ins += _sf_sum[i];
    }

    // What GPS says the specific force integral was: dv - g*T (NED, gravity +z).
    const float T = static_cast<float>((s1 - s0) * SF_BUCKET_MS) * 1e-3f;
    math::Vector3f f_gps = k.v - j->v;
    f_gps.z = (k.has_vz && j->has_vz) ? f_gps.z - G * T : f_ins.z; // no vertical velocity: horizontal only

    const float n_ins = f_ins.length(), n_gps = f_gps.length();
    if (n_ins < 0.5f * G * T || n_ins > 1.5f * G * T || n_gps < 0.5f * G * T || n_gps > 1.5f * G * T) return;

    // Rotate the estimate (in the earth frame) so f_ins turns toward f_gps.
    math::Vector3f e = (f_ins / n_ins).cross(f_gps / n_gps);
    if (_mag_in_use) e.z = 0.0f; // heading belongs to the magnetometer
    const math::Vector3f phi = e * std::min(1.0f, GPS_TILT_GAIN * gps_dt_ms * 1e-3f);
    math::Quaternion q = math::Quaternion(1.0f, 0.5f * phi.x, 0.5f * phi.y, 0.5f * phi.z) * _attitude_filter.get_quaternion();
    q.normalize();
    if (!(std::isfinite(q.q0) && std::isfinite(q.q1) && std::isfinite(q.q2) && std::isfinite(q.q3))) return;
    _attitude_filter.set_quaternion(q);

    // The stored integrals were computed with the old attitude: rotate them too, so the
    // next (overlapping) window doesn't count the same error again.
    for (int i = 0; i < SF_BUCKETS; ++i) _sf_sum[i] += phi.cross(_sf_sum[i]);

    _last_gps_corr_ms = _now_ms;
    _have_gps_corr = true;
}

void SensorFusion::update_altitude(float dt) {
    if (!_sensors || !_state.attitude_valid) return;

    // NED z is down, so upward acceleration is -z
    const float accel_z_up = -_state.linear_accel_ned.z;
    _alt_estimator.predict(accel_z_up, dt);

    auto* baro = _sensors->get_active_baro();
    if (baro && _sensors->has_healthy_baro()) {
        float measured_alt = baro->get_altitude_m();
        if (!_baro_ref_set) {
            // altitude is reported relative to the take-off point
            _baro_ref_alt = measured_alt;
            _baro_ref_set = true;
            _baro_ref_src = baro;
            _alt_estimator.reset(0.0f);
        } else if (baro != _baro_ref_src) {
            // Failover to another barometer: two baros can disagree by metres (own
            // ground reference, offset, temperature). Re-reference the new one to the
            // current estimate so the altitude stays continuous instead of stepping,
            // which would make ALT_HOLD climb or dive.
            _baro_ref_alt = measured_alt - _alt_estimator.get_altitude();
            _baro_ref_src = baro;
        }
        _alt_estimator.correct(measured_alt - _baro_ref_alt, dt);
        _state.altitude_valid = true;
    } else {
        _state.altitude_valid = false;
    }

    _state.altitude_m = _alt_estimator.get_altitude();
    _state.vertical_velocity_m_s = _alt_estimator.get_vertical_velocity();
    _state.position_ned.z = -_state.altitude_m;
    _state.velocity_ned.z = -_state.vertical_velocity_m_s;
}

void SensorFusion::update_position(float dt) {
    if (!_sensors || !_state.attitude_valid) return;

    auto* gps = _sensors->get_active_gps();
    const bool gps_ok = gps && _sensors->has_healthy_gps() && gps->has_3d_fix();

    // Dead-reckon with accel between GPS samples, but only briefly: without GPS the
    // integrated accel bias makes velocity grow without bound.
    if (_have_home && _now_ms - _last_gps_ms < DEAD_RECKON_MS) {
        _state.velocity_ned.x += _state.linear_accel_ned.x * dt;
        _state.velocity_ned.y += _state.linear_accel_ned.y * dt;
        _state.position_ned.x += _state.velocity_ned.x * dt;
        _state.position_ned.y += _state.velocity_ned.y * dt;
    } else {
        _state.velocity_ned.x = _state.velocity_ned.y = 0.0f; // unknown
    }

    if (!gps_ok) {
        _state.position_valid = false;
        _gps_lin_accel = math::Vector3f(0, 0, 0);
        return;
    }
    const auto& d = gps->get_data();
    if (!_have_home) {
        _home_lat = d.latitude_deg;
        _home_lon = d.longitude_deg;
        _have_home = true;
        _state.position_ned.x = _state.position_ned.y = 0.0f;
    }

    if (d.timestamp_ms != _last_gps_ms) {
        // GPS-derived linear acceleration (horizontal), used to clean the gravity reference
        if (_last_gps_ms != 0) {
            const float gdt = static_cast<float>(d.timestamp_ms - _last_gps_ms) * 1e-3f;
            if (gdt > 0.02f && gdt < 1.0f) {
                math::Vector3f a = (d.velocity_ned - _prev_gps_vel) / gdt;
                a.z = d.has_velocity_ned ? a.z : 0.0f;
                _gps_lin_accel = _gps_lin_accel + (a - _gps_lin_accel) * 0.5f;
            }
        }
        gps_tilt_correction(GpsVel{d.timestamp_ms, d.velocity_ned, d.has_velocity_ned});
        _prev_gps_vel = d.velocity_ned;
        _last_gps_ms = d.timestamp_ms;
    }

    const double R = 6378137.0, d2r = 3.14159265358979323846 / 180.0;
    const float n = static_cast<float>((d.latitude_deg - _home_lat) * d2r * R);
    const float e = static_cast<float>((d.longitude_deg - _home_lon) * d2r * R * std::cos(_home_lat * d2r));
    const float kp = std::min(1.0f, 0.5f * dt), kv = std::min(1.0f, 1.0f * dt);
    _state.position_ned.x += kp * (n - _state.position_ned.x);
    _state.position_ned.y += kp * (e - _state.position_ned.y);
    _state.velocity_ned.x += kv * (d.velocity_ned.x - _state.velocity_ned.x);
    _state.velocity_ned.y += kv * (d.velocity_ned.y - _state.velocity_ned.y);
    _state.position_valid = true;
}

void SensorFusion::update(float dt) {
    update_attitude(dt);
    update_altitude(dt);
    update_position(dt);
}

} // namespace fusion
} // namespace autopilot
