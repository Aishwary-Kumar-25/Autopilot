#pragma once
/**
 * Software-in-the-loop quadcopter model + simulated sensors with fault injection.
 *
 * Header-only and test-only: it is NOT part of the flight firmware. It closes the
 * loop around the real flight code: the autopilot writes PWM through the HAL, the
 * model turns PWM into motor thrust, integrates 6-DOF rigid body dynamics, and the
 * simulated sensors report what real chips would (specific force, body rates,
 * barometric altitude, body-frame magnetic field), with noise.
 *
 * Frames: NED earth, FRD body (same convention as drivers/sensor_interface.hpp).
 * Default motor order and spin directions match the Quad-X mixer:
 *   0 FR (CCW), 1 RL (CCW), 2 FL (CW), 3 RR (CW)
 * Any other multirotor layout (hexa, octa, ...) can be taken from the mixer's
 * own allocation rows with set_layout(), so the simulated frame always matches
 * the firmware's geometry.
 *
 * Airframe (a typical 450-class quad): 1.2 kg, 4 x 6 N max thrust (hover ~49%),
 * 0.127 m arm, motor time constant 30 ms.
 *
 * Fault injection: motor_health[i] scales motor i's thrust (0 = dead motor or
 * ESC, 0.5 = damaged prop), and every motor reports its RPM like an ESC with
 * telemetry would (rpm()).
 */
#include "drivers/sensor_interface.hpp"
#include "sim/sim_body.hpp"
#include "hal/hal.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <algorithm>
#include "control/actuator_matrix_mixer.hpp"

namespace autopilot {
namespace sim {

class QuadModel : public SimBody {
public:
    // Airframe
    double mass = 1.2;
    V3 inertia{0.012, 0.012, 0.020};
    double arm = 0.127;           // m, centre to motor
    double max_thrust = 6.0;      // N per motor
    double kq = 0.016;            // yaw torque / thrust (m)
    double motor_tau = 0.03;      // s
    double drag = 0.25;           // N per m/s of airspeed (linear: rotor / momentum drag)
    double drag_cda = 0.0;        // m^2, bluff-body drag area (quadratic); 0 = off
    double rho = 1.225;           // kg/m^3
    V3 wind;                      // NED, m/s (air moves at this velocity)
    double yaw_drag = 4e-4;       // N m per (rad/s)^2: frame + prop drag against a fast spin
    double max_rpm = 12000.0;     // RPM at full thrust (telemetry only)

    // Geometry: motor positions (FRD, m) and yaw reaction sign (+1 CCW prop).
    static constexpr int MAX_MOTORS = 16;
    int n_motors = 4;
    double mx[MAX_MOTORS]{}, my[MAX_MOTORS]{}, ys[MAX_MOTORS]{};
    double motor_health[MAX_MOTORS];   // thrust scale, 1 = healthy, 0 = dead

    double thrust[MAX_MOTORS]{};
    bool on_ground = true;
    double max_touchdown_speed = 0.0;

    QuadModel() {
        for (double& h : motor_health) h = 1.0;
        const double a = arm * 0.70710678;
        static const double px[4] = {+1, -1, +1, -1}, py[4] = {+1, -1, -1, +1}, ysq[4] = {+1, +1, -1, -1};
        for (int i = 0; i < 4; ++i) { mx[i] = px[i] * a; my[i] = py[i] * a; ys[i] = ysq[i]; }
    }

    /**
     * Take the motor layout from the mixer's allocation rows: a motor's roll
     * coefficient is -y, its pitch coefficient +x (FRD), and the sign of its yaw
     * coefficient is its prop's reaction torque. Each motor sits arm_m from the centre.
     */
    void set_layout(const control::ActuatorMatrixMixer& mixer, double arm_m) {
        arm = arm_m;
        n_motors = std::min<int>(mixer.get_num_channels(), MAX_MOTORS);
        for (int i = 0; i < n_motors; ++i) {
            const auto& a = mixer.get_actuator_config(static_cast<uint8_t>(i));
            const double x = a.pitch, y = -a.roll, n = std::sqrt(x * x + y * y);
            mx[i] = n > 1e-6 ? arm_m * x / n : 0.0;
            my[i] = n > 1e-6 ? arm_m * y / n : 0.0;
            ys[i] = a.yaw >= 0.0f ? 1.0 : -1.0;
        }
    }

    /**
     * Match the airframe to the firmware's multirotor mixer. Quad-X keeps the
     * default 450-class quad; other rotor counts get a bigger frame with the same
     * motors (0.20 m arms, mass growing with the rotor count so hover stays ~49%).
     */
    void configure_frame(const control::ActuatorMatrixMixer& mixer) {
        if (mixer.get_num_channels() == 4) return;
        const double k = mixer.get_num_channels() / 4.0;
        set_layout(mixer, 0.20);
        mass = 1.2 * k;
        inertia = V3(0.019 * k, 0.019 * k, 0.032 * k);
    }

    /**
     * What an ESC with telemetry would report for motor i (RPM). A motor with
     * health 0 is a dead motor/ESC: it reports 0 at once (a burnt motor or a
     * desynced ESC stops commutating). Partial health = damaged prop: the motor
     * still turns at its commanded speed, only the thrust is missing.
     */
    double rpm(int i) const {
        if (motor_health[i] <= 0.0) return 0.0;
        return max_rpm * std::sqrt(std::fmax(0.0, thrust[i] / std::fmax(1e-6, motor_health[i])) / max_thrust);
    }

    const char* name() const override { return "quad"; }
    double load() const override {
        double m = 0;
        for (int i = 0; i < n_motors; ++i) m += thrust[i] / (n_motors * max_thrust);
        return m;
    }
    void place(double north, double east, double yaw) override {
        SimBody::place(north, east, yaw);
        for (double& t : thrust) t = 0.0;
        on_ground = true;
    }
    /** Multirotor ESCs on channels 0..n_motors-1 (unidirectional, 1000 us = stopped). */
    void step_pwm(const uint16_t* pwm_us, int n, double dt) override {
        double cmd[MAX_MOTORS]{};
        for (int i = 0; i < n_motors; ++i) cmd[i] = unidir(pwm_us, n, i);
        step(cmd, dt);
    }

    /** Advance by dt with motor commands in [0,1] (n_motors of them). */
    void step(const double* cmd, double dt) {
        V3 torque;
        double total = 0;
        for (int i = 0; i < n_motors; ++i) {
            const double c = std::fmax(0.0, std::fmin(1.0, cmd[i])) * std::fmax(0.0, motor_health[i]);
            thrust[i] = lag(thrust[i], c * max_thrust, motor_tau, dt);
            total += thrust[i];
            const V3 r(mx[i], my[i], 0.0);
            torque = torque + r.cross(V3(0, 0, -thrust[i]));
            torque.z += ys[i] * kq * thrust[i];
        }
        torque = torque - omega * 0.002; // small rotational damping
        torque.z -= yaw_drag * omega.z * std::fabs(omega.z);

        // Translational dynamics (NED)
        const V3 air = vel - wind;   // air-relative velocity
        const double airspeed = std::sqrt(air.dot(air));
        V3 force = body_to_ned(V3(0, 0, -total)) + V3(0, 0, mass * g) - air * drag -
                   air * (0.5 * rho * drag_cda * airspeed);
        accel_ned = force * (1.0 / mass);

        // Ground contact at altitude 0
        if (on_ground) {
            if (accel_ned.z < 0) {           // net upward force: lift off
                on_ground = false;
            } else {
                accel_ned = V3();
                vel = V3();
                omega = V3();
                return;
            }
        }
        vel = vel + accel_ned * dt;
        pos = pos + vel * dt;
        if (pos.z >= 0.0) {                  // touchdown
            max_touchdown_speed = std::fmax(max_touchdown_speed, vel.z);
            pos.z = 0.0;
            vel = V3();
            accel_ned = V3();
            omega = V3();
            // settle level on the landing gear
            set_yaw(yaw());
            on_ground = true;
            return;
        }

        // Rotational dynamics: I w' = tau - w x (I w)
        const V3 Iw(inertia.x * omega.x, inertia.y * omega.y, inertia.z * omega.z);
        const V3 rhs = torque - omega.cross(Iw);
        omega = omega + V3(rhs.x / inertia.x, rhs.y / inertia.y, rhs.z / inertia.z) * dt;

        integrate_attitude(dt);
    }
};

// ---------------- simulated sensors with fault injection ----------------

enum class Fault { NONE, STUCK, DROPOUT, GYRO_OFFSET, NOT_A_NUMBER };

struct SimContext {
    SimBody* model{nullptr};
    hal::Time* time{nullptr};
    std::mt19937 rng{42};
    double noise(double sd) { return std::normal_distribution<double>(0.0, sd)(rng); }
};

class SimIMU : public drivers::IMUSensor {
public:
    SimIMU(SimContext* ctx, const char* name) : _ctx(ctx), _name(name) { _health = drivers::SensorHealth::HEALTHY; }
    drivers::SensorType type() const override { return drivers::SensorType::IMU; }
    const char* name() const override { return _name; }
    bool init() override { return true; }
    void set_fault(Fault f) { _fault = f; }
    // Residual bias left after the boot calibration (e.g. temperature drift in flight).
    math::Vector3f gyro_bias{0, 0, 0};
    // Accelerometer noise, m/s^2 rms per sample (0.05 = clean bench; motor vibration on
    // a real frame after the chip's DLPF is more like 0.5-2).
    double accel_noise = 0.05;

    bool update() override {
        if (_fault == Fault::DROPOUT) return false;      // bus silent: no new samples
        if (_fault != Fault::STUCK) {
            const SimBody& m = *_ctx->model;
            const V3 f = m.specific_force();
            _accel = math::Vector3f(float(f.x + _ctx->noise(accel_noise)), float(f.y + _ctx->noise(accel_noise)),
                                    float(f.z + _ctx->noise(accel_noise)));
            _gyro = math::Vector3f(float(m.omega.x + _ctx->noise(0.004)), float(m.omega.y + _ctx->noise(0.004)),
                                   float(m.omega.z + _ctx->noise(0.004)));
            _gyro += gyro_bias;
            if (_fault == Fault::GYRO_OFFSET) _gyro.x += 0.8f;
            if (_fault == Fault::NOT_A_NUMBER) _gyro.y = std::numeric_limits<float>::quiet_NaN();
        }
        record_success(_ctx->time->millis());   // a stuck chip still "answers" on the bus
        return true;
    }
    const math::Vector3f& get_accel() const override { return _accel; }
    const math::Vector3f& get_gyro() const override { return _gyro; }
    float get_temperature() const override { return 30.0f; }
    void calibrate_gyro_bias(uint16_t) override {}
    const math::Vector3f& get_gyro_bias() const override { return _bias; }

private:
    SimContext* _ctx;
    const char* _name;
    Fault _fault{Fault::NONE};
    math::Vector3f _accel{0, 0, -9.80665f}, _gyro{0, 0, 0}, _bias{0, 0, 0};
};

class SimBaro : public drivers::BarometerSensor {
public:
    explicit SimBaro(SimContext* ctx) : _ctx(ctx) { _health = drivers::SensorHealth::HEALTHY; }
    drivers::SensorType type() const override { return drivers::SensorType::BAROMETER; }
    const char* name() const override { return "SIM_BARO"; }
    bool init() override { return true; }
    void set_fault(Fault f) { _fault = f; }
    bool update() override {
        if (_fault == Fault::DROPOUT) return false;
        // 50 Hz like a real baro: only produce a new sample every 20 ms
        const uint32_t now = _ctx->time->millis();
        if (_sample_count > 0 && now - _last_ms < 20) return true;
        _last_ms = now;
        _alt = float(_ctx->model->altitude() + _ctx->noise(0.08));
        _pressure = float(101325.0 * std::pow(1.0 - _alt / 44330.0, 1.0 / 0.190295));
        record_success(now);
        return true;
    }
    float get_pressure_pa() const override { return _pressure; }
    float get_altitude_m() const override { return _alt; }
    float get_temperature_c() const override { return 20.0f; }
    void set_ground_reference(float) override {}

private:
    SimContext* _ctx;
    Fault _fault{Fault::NONE};
    uint32_t _last_ms{0};
    float _alt{0.0f}, _pressure{101325.0f};
};

class SimCompass : public drivers::CompassSensor {
public:
    explicit SimCompass(SimContext* ctx) : _ctx(ctx) { _health = drivers::SensorHealth::HEALTHY; }
    drivers::SensorType type() const override { return drivers::SensorType::MAGNETOMETER; }
    const char* name() const override { return "SIM_MAG"; }
    bool init() override { return true; }
    bool update() override {
        const uint32_t now = _ctx->time->millis();
        if (_sample_count > 0 && now - _last_ms < 10) return true;
        _last_ms = now;
        const V3 b = _ctx->model->ned_to_body(V3(0.2, 0.0, 0.4)); // gauss, northern hemisphere
        _field = math::Vector3f(float(b.x + _ctx->noise(0.003)), float(b.y + _ctx->noise(0.003)),
                                float(b.z + _ctx->noise(0.003)));
        _heading = heading_from_field(_field);
        record_success(now);
        return true;
    }
    const math::Vector3f& get_field_gauss() const override { return _field; }
    float get_heading_rad() const override { return _heading; }

private:
    SimContext* _ctx;
    uint32_t _last_ms{0};
    math::Vector3f _field{0.2f, 0.0f, 0.4f};
    float _heading{0.0f};
};

/**
 * GPS receiver with NED velocity (like a u-blox over DroneCAN Fix2): 10 Hz, with
 * a fixed output latency (real receivers report ~50-200 ms late) and noise.
 */
class SimGPS : public drivers::GPSSensor {
public:
    static constexpr double HOME_LAT = 26.5123, HOME_LON = 80.2329; // arbitrary
    uint32_t period_ms = 100;
    uint32_t latency_ms = 100;
    double vel_noise = 0.05, pos_noise = 0.3;

    explicit SimGPS(SimContext* ctx) : _ctx(ctx) { _health = drivers::SensorHealth::HEALTHY; }
    const char* name() const override { return "SIM_GPS"; }
    bool init() override { return true; }
    void set_fault(Fault f) { _fault = f; }

    bool update() override {
        if (_fault == Fault::DROPOUT) return false;
        const uint32_t now = _ctx->time->millis();
        // Truth history, sampled every call, to emulate output latency.
        _hist[_head] = {now, _ctx->model->pos, _ctx->model->vel};
        _head = (_head + 1) % HIST;
        if (_sample_count > 0 && now - _last_ms < period_ms) return true;
        _last_ms = now;
        // Newest truth sample that is at least latency_ms old.
        const Snap* s = nullptr;
        for (const Snap& h : _hist) {
            if (h.t_ms != 0 && now - h.t_ms >= latency_ms && (!s || h.t_ms > s->t_ms)) s = &h;
        }
        if (!s) return true;
        if (!_ctx->model->gps_available()) {     // e.g. submerged: receiver reports no fix
            _data.has_fix = false;
            _data.fix_type = 0;
            _data.satellites = 0;
            _data.has_velocity_ned = false;
            _data.timestamp_ms = now;
            record_success(now);
            return true;
        }
        const double R = 6378137.0, d2r = 3.14159265358979323846 / 180.0;
        _data.has_fix = true;
        _data.fix_type = 3;
        _data.satellites = 14;
        _data.latitude_deg = HOME_LAT + (s->pos.x + _ctx->noise(pos_noise)) / R / d2r;
        _data.longitude_deg = HOME_LON + (s->pos.y + _ctx->noise(pos_noise)) / (R * std::cos(HOME_LAT * d2r)) / d2r;
        _data.altitude_msl_m = float(120.0 - s->pos.z);
        _data.velocity_ned = math::Vector3f(float(s->vel.x + _ctx->noise(vel_noise)),
                                            float(s->vel.y + _ctx->noise(vel_noise)),
                                            float(s->vel.z + _ctx->noise(vel_noise)));
        _data.has_velocity_ned = true;
        _data.speed_m_s = std::sqrt(_data.velocity_ned.x * _data.velocity_ned.x +
                                    _data.velocity_ned.y * _data.velocity_ned.y);
        _data.timestamp_ms = now;
        record_success(now);
        return true;
    }
    const drivers::GPSData& get_data() const override { return _data; }
    bool has_3d_fix() const override { return _data.fix_type >= 3; }

private:
    struct Snap { uint32_t t_ms{0}; V3 pos, vel; };
    static constexpr int HIST = 128;
    SimContext* _ctx;
    Fault _fault{Fault::NONE};
    Snap _hist[HIST]{};
    int _head{0};
    uint32_t _last_ms{0};
    drivers::GPSData _data;
};

} // namespace sim
} // namespace autopilot
