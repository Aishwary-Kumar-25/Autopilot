#pragma once
/**
 * Minimal scriptable sensors for unit tests: the test sets the values, update()
 * publishes them (or fails on demand). Unlike register mocks, these let a test
 * drive SensorManager / SensorFusion / vehicles with exact inputs.
 */
#include "drivers/sensor_interface.hpp"
#include "hal/hal.hpp"

namespace test {

using autopilot::math::Vector3f;
namespace drv = autopilot::drivers;

class FakeIMU : public drv::IMUSensor {
public:
    explicit FakeIMU(autopilot::hal::Time* t, const char* n = "FakeIMU") : _t(t), _name(n) {}
    const char* name() const override { return _name; }
    bool init() override { record_success(_t->millis()); return true; }
    bool update() override {
        if (fail) { record_failure(); return false; }
        // Tiny deterministic noise: a bit-identical stream is (correctly) flagged as a stuck sensor.
        _seed = _seed * 1664525u + 1013904223u;
        const float n = (static_cast<float>(_seed >> 8) / 16777216.0f - 0.5f) * 2e-4f;
        _accel_out = accel + Vector3f(n, -n, n);
        _gyro_out = gyro + Vector3f(-n, n, n);
        record_success(_t->millis());
        return true;
    }
    const Vector3f& get_accel() const override { return _accel_out; }
    const Vector3f& get_gyro() const override { return _gyro_out; }
    float get_temperature() const override { return 25.0f; }
    void calibrate_gyro_bias(uint16_t) override {}
    const Vector3f& get_gyro_bias() const override { return _bias; }

    Vector3f accel{0.0f, 0.0f, -9.80665f};
    Vector3f gyro{0.0f, 0.0f, 0.0f};
    bool fail{false};

private:
    autopilot::hal::Time* _t;
    const char* _name;
    Vector3f _bias{0, 0, 0};
    Vector3f _accel_out{0.0f, 0.0f, -9.80665f};
    Vector3f _gyro_out{0, 0, 0};
    uint32_t _seed{12345};
};

class FakeBaro : public drv::BarometerSensor {
public:
    explicit FakeBaro(autopilot::hal::Time* t, const char* n = "FakeBaro") : _t(t), _name(n) {}
    const char* name() const override { return _name; }
    bool init() override { record_success(_t->millis()); return true; }
    bool update() override {
        if (fail) { record_failure(); return false; }
        record_success(_t->millis());
        return true;
    }
    float get_pressure_pa() const override { return 101325.0f; }
    float get_altitude_m() const override { return altitude; }
    float get_temperature_c() const override { return 20.0f; }
    void set_ground_reference(float) override {}

    float altitude{0.0f};
    bool fail{false};

private:
    autopilot::hal::Time* _t;
    const char* _name;
};

class FakeGPS : public drv::GPSSensor {
public:
    explicit FakeGPS(autopilot::hal::Time* t) : _t(t) {}
    const char* name() const override { return "FakeGPS"; }
    bool init() override { record_success(_t->millis()); return true; }
    bool update() override { data.timestamp_ms = _t->millis(); record_success(_t->millis()); return true; }
    const drv::GPSData& get_data() const override { return data; }
    bool has_3d_fix() const override { return data.has_fix && data.fix_type >= 3; }
    drv::GPSData data;

private:
    autopilot::hal::Time* _t;
};

} // namespace test
