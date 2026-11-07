#pragma once

#include "drivers/sensor_interface.hpp"
#include "hal/hal.hpp"
#include "math/vector3.hpp"
#include "math/low_pass_filter.hpp"

namespace autopilot {
namespace drivers {

class MPU6050 : public IMUSensor {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x68;
    static constexpr uint8_t ALT_I2C_ADDR = 0x69;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    static constexpr uint8_t EXPECTED_WHO_AM_I = 0x68;

    MPU6050(hal::I2CDevice* i2c, hal::Time* time, uint8_t i2c_addr = DEFAULT_I2C_ADDR);

    const char* name() const override { return "MPU6050"; }
    bool init() override;
    bool update() override;

    const math::Vector3f& get_accel() const override { return _accel_m_s2; }
    const math::Vector3f& get_gyro() const override { return _gyro_rad_s; }
    float get_temperature() const override { return _temperature_c; }

    void calibrate_gyro_bias(uint16_t sample_count = 500) override;
    const math::Vector3f& get_gyro_bias() const override { return _gyro_bias; }

    uint8_t get_address() const { return _i2c_addr; }

private:
    hal::I2CDevice* _i2c{nullptr};
    hal::Time* _time{nullptr};
    uint8_t _i2c_addr{DEFAULT_I2C_ADDR};

    math::Vector3f _accel_m_s2{0.0f, 0.0f, 9.80665f};
    math::Vector3f _gyro_rad_s{0.0f, 0.0f, 0.0f};
    math::Vector3f _gyro_bias{0.0f, 0.0f, 0.0f};
    float _temperature_c{25.0f};

    math::LowPassFilter _accel_filter_x{30.0f, 400.0f};
    math::LowPassFilter _accel_filter_y{30.0f, 400.0f};
    math::LowPassFilter _accel_filter_z{30.0f, 400.0f};

    static constexpr float ACCEL_SCALE_8G = 9.80665f / 4096.0f;
    static constexpr float GYRO_SCALE_2000DPS = (static_cast<float>(M_PI) / 180.0f) / 16.4f;

    bool verify_device_id();
};

} // namespace drivers
} // namespace autopilot
