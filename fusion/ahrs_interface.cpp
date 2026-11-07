#include "ahrs_interface.hpp"
#include <algorithm>
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

// Gyro-only quaternion propagation, the same Euler step Madgwick uses, so the
// difference between it and the filter output is exactly the applied correction.
static math::Quaternion propagate_gyro(const math::Quaternion& q, const math::Vector3f& w, float dt) {
    math::Quaternion r(q.q0 + 0.5f * dt * (-q.q1 * w.x - q.q2 * w.y - q.q3 * w.z),
                       q.q1 + 0.5f * dt * (q.q0 * w.x + q.q2 * w.z - q.q3 * w.y),
                       q.q2 + 0.5f * dt * (q.q0 * w.y - q.q1 * w.z + q.q3 * w.x),
                       q.q3 + 0.5f * dt * (q.q0 * w.z + q.q1 * w.y - q.q2 * w.x));
    r.normalize();
    return r;
}

static bool finite3(const math::Vector3f& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

SensorFusion::SensorFusion(drivers::SensorManager* sensor_manager)
    : _sensors(sensor_manager) {}

void SensorFusion::set_drag_model(float mcoef, float bcoef) {
    if (std::isfinite(mcoef) && mcoef > 0.01f) _drag_mcoef = mcoef;
    if (std::isfinite(bcoef) && bcoef >= 0.0f) _drag_bcoef = bcoef;
}

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
    _tilt_aiding = TiltAiding::ACCEL;
    _gyro_bias = math::Vector3f(0, 0, 0);
    _q_pred_valid = false;
    _bias_imu = nullptr;
    _drag_active = false;
    _drag_vel = _drag_innov = _drag_vm_lp = _drag_lin_body = math::Vector3f(0, 0, 0);
    _gps_lin_valid = false;
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

    // The learned bias belongs to one IMU: start again after a failover.
    if (imu != _bias_imu) {
        _bias_imu = imu;
        _gyro_bias = math::Vector3f(0, 0, 0);
        _q_pred_valid = false;
    }

    const math::Vector3f& gyro_raw = imu->get_gyro();
    const math::Vector3f gyro = gyro_raw - _gyro_bias;
    const auto& accel = imu->get_accel();
    _now_ms = imu->last_update_ms();
    _state.angular_velocity = gyro;
    _state.gyro_bias = _gyro_bias;
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
    const bool settled = _time_since_align > ALIGN_SETTLE_S;
    if (settled) _attitude_filter.set_beta(BETA_FLIGHT);

    // Learn the gyro bias from whatever corrected the primary since the last tick
    // (accel, mag, drag or GPS), before this tick's propagation.
    const bool drag_quiet = _tilt_aiding != TiltAiding::DRAG ||
                            (_drag_time_s > DRAG_SETTLE_S && _drag_innov.length() < BIAS_LEARN_MAX_DRAG_INNOV);
    if (_bias_learning && _q_pred_valid && settled && drag_quiet && gyro.length() < BIAS_LEARN_MAX_RATE) {
        learn_gyro_bias();
    }

    // GPS-aided while time-aligned GPS tilt corrections keep arriving. In the air the
    // drag model takes over sooner than a gyro-only coast would.
    const bool mc_in_air = _mc_in_air && _drag_enabled;
    const uint32_t gps_timeout = mc_in_air ? GPS_AIDING_TIMEOUT_DRAG_MS : GPS_AIDING_TIMEOUT_MS;
    _gps_aiding = _have_gps_corr && settled && _now_ms - _last_gps_corr_ms < gps_timeout;
    // Drag aiding starts at lift-off (not after ALIGN_SETTLE_S): Madgwick would read the
    // drag of a wind-blown take-off as tilt.
    const bool drag_tilt = mc_in_air && !_gps_aiding;
    if (!mc_in_air) _drag_active = false;

    // Manoeuvre acceleration (GPS-derived, else from the drag model), removed from the
    // accelerometer for the backup filter and the selector's tilt reference.
    math::Vector3f lin_body(0, 0, 0);
    if (_gps_lin_valid) {
        lin_body = _attitude_filter.get_quaternion().conjugate().rotate_vector(_gps_lin_accel);
    } else if (_drag_active) {
        lin_body = _drag_lin_body;
    }
    const math::Vector3f accel_ref = accel - lin_body;

    const math::Quaternion q_start = _attitude_filter.get_quaternion();
    if (_gps_aiding || drag_tilt) {
        // Gyro only here; tilt comes from GPS or the drag observer, heading from the mag.
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
    // The observer runs whenever a multirotor flies (so it is converged the moment GPS
    // drops out) but only steers the attitude without GPS.
    if (mc_in_air) drag_observer(gyro, accel, drag_tilt, dt);
    _tilt_aiding = _gps_aiding ? TiltAiding::GPS : (drag_tilt ? TiltAiding::DRAG : TiltAiding::ACCEL);
    _q_pred = propagate_gyro(q_start, gyro, dt);
    _q_pred_valid = true;

    // Backup: raw (boot-calibrated) gyro, independent of the learned bias.
    _backup.update(gyro_raw, accel_ref, use_mag ? &mag : nullptr, dt);

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
        _q_pred_valid = false; // a reset, not a drift correction
        _drag_active = false;
        if (!finite3(_gyro_bias)) _gyro_bias = math::Vector3f(0, 0, 0);
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
    _state.tilt_aiding = static_cast<uint8_t>(_tilt_aiding);
    _state.heading_rad = _state.euler_rad.z < 0.0f ? _state.euler_rad.z + 2.0f * PI_F : _state.euler_rad.z;

    // Earth-frame (NED) acceleration: rotate specific force and add gravity back.
    math::Vector3f earth_accel = body_to_ned(accel);
    earth_accel.z += G;
    _state.linear_accel_ned = earth_accel;

    _state.attitude_valid = std::isfinite(_state.euler_rad.x) && std::isfinite(_state.euler_rad.y);
}

void SensorFusion::learn_gyro_bias() {
    // Correction since the last tick = filter output vs. gyro-only propagation.
    math::Quaternion dq = _attitude_filter.get_quaternion() * _q_pred.conjugate();
    if (dq.q0 < 0.0f) dq = math::Quaternion(-dq.q0, -dq.q1, -dq.q2, -dq.q3);
    const math::Vector3f phi_e(2.0f * dq.q1, 2.0f * dq.q2, 2.0f * dq.q3); // earth frame, rad
    const math::Vector3f phi_b = _q_pred.conjugate().rotate_vector(phi_e);
    if (!finite3(phi_b) || phi_b.length() > BIAS_LEARN_MAX_STEP) return;
    // A gyro reading b too high makes every correction ~ -b*dt in the body frame.
    _gyro_bias -= phi_b * BIAS_GAIN;
    _gyro_bias.x = std::clamp(_gyro_bias.x, -BIAS_MAX_RAD_S, BIAS_MAX_RAD_S);
    _gyro_bias.y = std::clamp(_gyro_bias.y, -BIAS_MAX_RAD_S, BIAS_MAX_RAD_S);
    _gyro_bias.z = std::clamp(_gyro_bias.z, -BIAS_MAX_RAD_S, BIAS_MAX_RAD_S);
}

float SensorFusion::drag_velocity_from_accel(float f) const {
    // Invert f = -(mcoef*v + k*|v|*v), k = rho / (2*bcoef), for one body axis.
    const float a = std::fabs(f);
    float v;
    if (_drag_bcoef > 0.0f) {
        const float k = 0.5f * 1.225f / _drag_bcoef;
        v = (-_drag_mcoef + std::sqrt(_drag_mcoef * _drag_mcoef + 4.0f * k * a)) / (2.0f * k);
    } else {
        v = a / _drag_mcoef;
    }
    return f > 0.0f ? -v : v;
}

void SensorFusion::drag_observer(const math::Vector3f& gyro, const math::Vector3f& accel, bool apply_tilt, float dt) {
    if (!finite3(accel) || dt <= 0.0f) return;
    math::Quaternion q = _attitude_filter.get_quaternion();
    // What the drag model reads off the accelerometer: body X/Y airspeed.
    const math::Vector3f vm(drag_velocity_from_accel(accel.x), drag_velocity_from_accel(accel.y), 0.0f);
    const float vz_meas = _state.altitude_valid ? -_state.vertical_velocity_m_s : 0.0f;

    if (!_drag_active) {
        // Start from the drag-measured velocity (just after take-off it's ~0 anyway).
        _drag_vel = q.rotate_vector(vm);
        _drag_vel.z = vz_meas;
        _drag_innov = math::Vector3f(0, 0, 0);
        _drag_vm_lp = vm;
        _drag_lin_body = math::Vector3f(0, 0, 0);
        _drag_active = true;
        _drag_time_s = 0.0f;
        for (auto& t : _cal) t.active = false;
        _cal_clock = 0.0f;
        return;
    }

    // Attitude-independent body acceleration from the drag model (for the backup and
    // the selector): a_b = d(v_b)/dt + w x v_b, with body Z velocity taken as 0.
    {
        const float k = dt / (DRAG_INNOV_TAU_S + dt);
        const math::Vector3f prev = _drag_vm_lp;
        _drag_vm_lp += (vm - _drag_vm_lp) * k;
        const math::Vector3f dv = (_drag_vm_lp - prev) / dt;
        const math::Vector3f a(dv.x - gyro.z * _drag_vm_lp.y, dv.y + gyro.z * _drag_vm_lp.x, 0.0f);
        _drag_lin_body += (a - _drag_lin_body) * (dt / (0.2f + dt));
    }

    // Accelerometer noise (vibration) from sample-to-sample differences: the motion of
    // the vehicle barely changes between samples, noise does.
    if (_have_prev_accel) {
        const float dx = accel.x - _prev_accel.x, dy = accel.y - _prev_accel.y;
        const float var = 0.25f * (dx * dx + dy * dy); // var(diff) = 2 var; two axes
        _accel_noise_var += (var - _accel_noise_var) * std::min(1.0f, dt / 2.0f);
    }
    _prev_accel = accel;
    _have_prev_accel = true;
    const float noise = std::sqrt(_accel_noise_var);
    const float gs = noise > DRAG_NOISE_REF ? std::max(DRAG_MIN_GAIN_SCALE, std::sqrt(DRAG_NOISE_REF / noise)) : 1.0f;

    // Predict: integrate earth-frame specific force + gravity.
    const math::Vector3f F = q.rotate_vector(accel);
    _drag_vel += (F + math::Vector3f(0, 0, G)) * dt;

    // Innovation in the body frame (drag only constrains body X/Y), low-passed.
    const math::Vector3f vb = q.conjugate().rotate_vector(_drag_vel);
    const math::Vector3f inn(vm.x - vb.x, vm.y - vb.y, 0.0f);
    _drag_innov += (inn - _drag_innov) * (dt / (DRAG_INNOV_TAU_S + dt));
    const math::Vector3f e = q.rotate_vector(_drag_innov);

    _drag_time_s += dt;
    if (_drag_cal_enabled && _drag_time_s > DRAG_SETTLE_S) drag_calibration(gyro, accel, dt);

    const bool settling = _drag_time_s < DRAG_SETTLE_S;
    _drag_vel += e * ((settling ? DRAG_SETTLE_VEL_GAIN : DRAG_VEL_GAIN * gs) * dt);
    _drag_vel.z += (vz_meas - _drag_vel.z) * (DRAG_VEL_GAIN * dt);
    if (!finite3(_drag_vel)) {
        _drag_active = false;
        return;
    }

    if (!apply_tilt || settling) return;
    const float fn2 = F.length_squared();
    if (fn2 < 0.09f * G * G || vb.length() > DRAG_MAX_AIRSPEED) return; // free fall / outside the model
    // A tilt error d makes the integrated velocity drift at d x F: rotate against it.
    math::Vector3f phi = F.cross(math::Vector3f(e.x, e.y, 0.0f)) * (DRAG_TILT_GAIN * gs * gs * dt / fn2);
    if (_mag_in_use) phi.z = 0.0f; // heading belongs to the magnetometer
    q = math::Quaternion(1.0f, 0.5f * phi.x, 0.5f * phi.y, 0.5f * phi.z) * q;
    q.normalize();
    if (std::isfinite(q.q0) && std::isfinite(q.q1) && std::isfinite(q.q2) && std::isfinite(q.q3)) {
        _attitude_filter.set_quaternion(q);
    }
}

void SensorFusion::drag_calibration(const math::Vector3f& gyro, const math::Vector3f& accel, float dt) {
    // Full NED airspeed from the drag model (body X/Y, low-passed) plus the baro's
    // vertical speed for the unknown body Z part; q is a given attitude.
    const float vz = _state.altitude_valid ? -_state.vertical_velocity_m_s : 0.0f;
    auto drag_vel_ned = [&](const math::Quaternion& q) {
        const math::Vector3f h = q.rotate_vector(math::Vector3f(_drag_vm_lp.x, _drag_vm_lp.y, 0.0f));
        const math::Vector3f z = q.rotate_vector(math::Vector3f(0, 0, 1));
        const float vbz = z.z > 0.5f ? (vz - h.z) / z.z : 0.0f;
        return h + z * vbz;
    };
    // Start a new window every WINDOW / TRACKS seconds, from the current primary attitude.
    _cal_clock += dt;
    const float stagger = DRAG_CAL_WINDOW_S / DRAG_CAL_TRACKS;
    if (_cal_clock >= stagger) {
        _cal_clock -= stagger;
        for (auto& t : _cal) {
            if (t.active) continue;
            t.active = true;
            t.t = 0.0f;
            t.q = _attitude_filter.get_quaternion();
            t.dv_int = math::Vector3f(0, 0, 0);
            t.vm0 = drag_vel_ned(t.q);
            t.a0 = t.q.rotate_vector(accel) + math::Vector3f(0, 0, G);
            break;
        }
    }
    for (auto& t : _cal) {
        if (!t.active) continue;
        if (gyro.length() > BIAS_LEARN_MAX_RATE) { t.active = false; continue; } // flips etc.: drop
        t.q = propagate_gyro(t.q, gyro, dt);
        t.dv_int += (t.q.rotate_vector(accel) + math::Vector3f(0, 0, G)) * dt;
        t.t += dt;
        if (t.t < DRAG_CAL_WINDOW_S) continue;
        t.active = false;
        // The low-passed drag airspeed lags by ~tau * acceleration: undo that, or
        // windows that end mid-manoeuvre would bias the coefficient.
        const math::Vector3f a1 = t.q.rotate_vector(accel) + math::Vector3f(0, 0, G);
        const math::Vector3f dvm = drag_vel_ned(t.q) - t.vm0 + (a1 - t.a0) * DRAG_INNOV_TAU_S;
        const math::Vector3f dvi(t.dv_int.x, t.dv_int.y, 0.0f);   // horizontal only
        const float n2 = dvi.length_squared();
        if (n2 < DRAG_CAL_MIN_DV * DRAG_CAL_MIN_DV || !finite3(dvm) || !finite3(dvi)) continue;
        // drag airspeed = true airspeed * (true mcoef / assumed mcoef)
        const float ratio = (dvm.x * dvi.x + dvm.y * dvi.y) / n2;
        if (!(ratio > 0.5f && ratio < 2.0f)) continue; // gust, take-off, ...: not drag
        const float c_old = _drag_mcoef;
        _drag_mcoef = std::clamp(c_old * std::pow(ratio, DRAG_CAL_GAIN), DRAG_MCOEF_MIN, DRAG_MCOEF_MAX);
        ++_drag_cal_windows;
        // The drag airspeed just changed scale by c_old/c_new. Shift the observer and
        // the other windows by the same amount, so the change is not an innovation
        // (which would kick the tilt).
        const float s = c_old / _drag_mcoef;
        const math::Vector3f dvb((s - 1.0f) * _drag_vm_lp.x, (s - 1.0f) * _drag_vm_lp.y, 0.0f);
        _drag_vel += _attitude_filter.get_quaternion().rotate_vector(dvb);
        _drag_vm_lp.x *= s;
        _drag_vm_lp.y *= s;
        for (auto& o : _cal) {
            if (o.active) o.vm0 = o.vm0 * s;
        }
    }
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
        _gps_lin_valid = false;
        return;
    }
    _gps_lin_valid = true;
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
