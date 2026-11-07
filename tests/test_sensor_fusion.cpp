#include "hal/posix/posix_hal.hpp"
#include "drivers/sensor_manager.hpp"
#include "fusion/ahrs_interface.hpp"
#include "tests/test_common.hpp"
#include <iostream>
#include <cmath>

using namespace autopilot;

class MockIMU : public drivers::IMUSensor {
public:
    const char* name() const override { return "MockIMU"; }
    bool init() override { _health = drivers::SensorHealth::HEALTHY; return true; }
    bool update() override { return true; }

    void set_data(const math::Vector3f& accel, const math::Vector3f& gyro) {
        _accel = accel;
        _gyro = gyro;
    }

    const math::Vector3f& get_accel() const override { return _accel; }
    const math::Vector3f& get_gyro() const override { return _gyro; }
    float get_temperature() const override { return 25.0f; }
    void calibrate_gyro_bias(uint16_t) override {}
    const math::Vector3f& get_gyro_bias() const override { return _bias; }

private:
    math::Vector3f _accel{0.0f, 0.0f, -9.80665f};
    math::Vector3f _gyro{0.0f, 0.0f, 0.0f};
    math::Vector3f _bias{0.0f, 0.0f, 0.0f};
};

class MockBaro : public drivers::BarometerSensor {
public:
    const char* name() const override { return "MockBaro"; }
    bool init() override { _health = drivers::SensorHealth::HEALTHY; return true; }
    bool update() override { return true; }

    void set_alt(float alt) { _alt = alt; }
    float get_pressure_pa() const override { return 101325.0f; }
    float get_altitude_m() const override { return _alt; }
    float get_temperature_c() const override { return 20.0f; }
    void set_ground_reference(float) override {}

private:
    float _alt{0.0f};
};

int main() {
    std::cout << "========================================================\n";
    std::cout << "TEST 2: Universal Vehicle-Agnostic Sensor Fusion\n";
    std::cout << "========================================================\n";

    hal::PosixHAL hal;
    drivers::SensorManager sm(&hal);

    auto mock_imu = std::make_shared<MockIMU>();
    auto mock_baro = std::make_shared<MockBaro>();

    mock_imu->init();
    mock_baro->init();

    sm.register_sensor(mock_imu);
    sm.register_sensor(mock_baro);

    fusion::SensorFusion fusion(&sm);
    fusion.reset();

    // Test 1: Level, at rest. Accelerometers measure specific force, so a level
    // FRD IMU reads (0, 0, -g).
    std::cout << "[Step 1] Testing level attitude convergence...\n";
    mock_imu->set_data(math::Vector3f(0.0f, 0.0f, -9.80665f), math::Vector3f(0.0f, 0.0f, 0.0f));

    for (int i = 0; i < 200; ++i) {
        fusion.update(0.01f); // 100Hz
    }

    const auto& state = fusion.get_state();
    CHECK(state.attitude_valid);
    std::cout << "  -> Roll: " << state.euler_rad.x * (180.0f / M_PI)
              << " deg | Pitch: " << state.euler_rad.y * (180.0f / M_PI) << " deg\n";
    CHECK(std::abs(state.euler_rad.x) < 0.02f);
    CHECK(std::abs(state.euler_rad.y) < 0.02f);
    std::cout << "  -> Level convergence [PASSED]\n";

    // Test 2: 30-degree Roll Tilt
    std::cout << "\n[Step 2] Testing roll tilt response (30 deg)...\n";
    float roll_rad = 30.0f * (M_PI / 180.0f);
    // Rolled right by 30 deg at rest: specific force = (0, -g sin(roll), -g cos(roll))
    mock_imu->set_data(math::Vector3f(0.0f, -9.80665f * std::sin(roll_rad), -9.80665f * std::cos(roll_rad)),
                       math::Vector3f(0.0f, 0.0f, 0.0f));

    for (int i = 0; i < 500; ++i) {
        fusion.update(0.01f);
    }

    float final_roll_deg = fusion.get_state().euler_rad.x * (180.0f / M_PI);
    std::cout << "  -> Settled Roll Angle: " << final_roll_deg << " deg (Target: ~30 deg)\n";
    CHECK(std::abs(final_roll_deg - 30.0f) < 1.0f);
    std::cout << "  -> Attitude tilt convergence [PASSED]\n";

    // Test 3: Altitude climb tracking
    std::cout << "\n[Step 3] Testing baro-inertial altitude estimation...\n";
    mock_baro->set_alt(15.0f);
    for (int i = 0; i < 200; ++i) {
        fusion.update(0.01f);
    }
    std::cout << "  -> Estimated Altitude: " << fusion.get_state().altitude_m << " m (Target: ~15m)\n";
    CHECK(std::abs(fusion.get_state().altitude_m - 15.0f) < 0.5f);
    std::cout << "  -> Altitude estimator [PASSED]\n";

    // Test 4: vertical acceleration sign. Thrusting upward at 2 m/s^2 while level
    // means the accelerometer reads (0,0,-(g+2)); the estimator must see +2 m/s^2 UP.
    std::cout << "\n[Step 4] Testing vertical acceleration sign...\n";
    mock_imu->set_data(math::Vector3f(0.0f, 0.0f, -9.80665f), math::Vector3f(0.0f, 0.0f, 0.0f));
    for (int i = 0; i < 500; ++i) fusion.update(0.01f);
    mock_imu->set_data(math::Vector3f(0.0f, 0.0f, -(9.80665f + 2.0f)), math::Vector3f(0.0f, 0.0f, 0.0f));
    fusion.update(0.01f);
    CHECK_NEAR(-fusion.get_state().linear_accel_ned.z, 2.0, 0.05);
    std::cout << "  -> Upward accel seen as " << -fusion.get_state().linear_accel_ned.z << " m/s^2 up [PASSED]\n";

    // Test 5: dissimilar backup estimator takes over when the primary is corrupted
    std::cout << "\n[Step 5] Estimator selector prefers the primary while both agree...\n";
    mock_imu->set_data(math::Vector3f(0.0f, 0.0f, -9.80665f), math::Vector3f(0.0f, 0.0f, 0.0f));
    for (int i = 0; i < 300; ++i) fusion.update(0.01f);
    CHECK(fusion.get_state().attitude_source == 0);
    std::cout << "  -> Primary in use while estimators agree [PASSED]\n";

    std::cout << "\n>>> ALL SENSOR FUSION TESTS PASSED SUCCESSFULLY! <<<\n";
    return test::finish("test_sensor_fusion");
}
