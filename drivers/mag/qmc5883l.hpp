#pragma once

#include "drivers/sensor_interface.hpp"
#include "hal/hal.hpp"
#include "math/vector3.hpp"

namespace autopilot {
namespace drivers {

class QMC5883L : public CompassSensor {
public:
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x0D;
    static constexpr uint8_t REG_CHIP_ID = 0x0D;
    static constexpr uint8_t EXPECTED_CHIP_ID = 0xFF; // Common QMC5883L ID

    QMC5883L(hal::I2CDevice* i2c, hal::Time* time, uint8_t i2c_addr = DEFAULT_I2C_ADDR);

    const char* name() const override { return "QMC5883L"; }
    bool init() override;
    bool update() override;

    const math::Vector3f& get_field_gauss() const override { return _field_gauss; }
    float get_heading_rad() const override { return _heading_rad; }

    uint8_t get_address() const { return _i2c_addr; }

private:
    hal::I2CDevice* _i2c{nullptr};
    hal::Time* _time{nullptr};
    uint8_t _i2c_addr{DEFAULT_I2C_ADDR};

    math::Vector3f _field_gauss{0.0f, 0.0f, 0.0f};
    float _heading_rad{0.0f};

    // init() selects the 8 G range (CONTROL_1 = 0x1D): 3000 LSB/G. (The old 2 G
    // scale, 12000 LSB/G, made every reading 4x too small.)
    static constexpr float GAUSS_SCALE_8G = 1.0f / 3000.0f;
};

} // namespace drivers
} // namespace autopilot
