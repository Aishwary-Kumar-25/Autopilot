#pragma once

#include "drivers/sensor_interface.hpp"
#include "hal/hal.hpp"
#include "math/low_pass_filter.hpp"

namespace autopilot {
namespace drivers {

class BMP280 : public BarometerSensor {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x76;
    static constexpr uint8_t ALT_I2C_ADDR = 0x77;
    static constexpr uint8_t REG_CHIP_ID = 0xD0;
    static constexpr uint8_t EXPECTED_CHIP_ID = 0x58; // BMP280

    BMP280(hal::I2CDevice* i2c, hal::Time* time, uint8_t i2c_addr = DEFAULT_I2C_ADDR);

    const char* name() const override { return "BMP280"; }
    bool init() override;
    bool update() override;

    float get_pressure_pa() const override { return _pressure_pa; }
    float get_altitude_m() const override { return _relative_altitude_m; }
    float get_temperature_c() const override { return _temperature_c; }

    void set_ground_reference(float ground_pressure_pa) override;
    uint8_t get_address() const { return _i2c_addr; }

private:
    hal::I2CDevice* _i2c{nullptr};
    hal::Time* _time{nullptr};
    uint8_t _i2c_addr{DEFAULT_I2C_ADDR};

    float _pressure_pa{101325.0f};
    float _ground_pressure_pa{101325.0f};
    float _relative_altitude_m{0.0f};
    float _temperature_c{20.0f};

    math::LowPassFilter _altitude_filter{10.0f, 50.0f};

    // Calibration coefficients from sensor OTP
    uint16_t _dig_T1{0};
    int16_t  _dig_T2{0};
    int16_t  _dig_T3{0};
    uint16_t _dig_P1{0};
    int16_t  _dig_P2{0};
    int16_t  _dig_P3{0};
    int16_t  _dig_P4{0};
    int16_t  _dig_P5{0};
    int16_t  _dig_P6{0};
    int16_t  _dig_P7{0};
    int16_t  _dig_P8{0};
    int16_t  _dig_P9{0};

    int32_t _t_fine{0};

    bool read_calibration_data();
    float calculate_altitude(float pressure_pa, float ground_pa);
};

} // namespace drivers
} // namespace autopilot
