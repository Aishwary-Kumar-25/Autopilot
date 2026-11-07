#include "qmc5883l.hpp"
#include <cmath>

namespace autopilot {
namespace drivers {

static constexpr uint8_t REG_DATA_OUT_X_LSB = 0x00;
static constexpr uint8_t REG_STATUS         = 0x06;
static constexpr uint8_t REG_CONTROL_1      = 0x09;
static constexpr uint8_t REG_CONTROL_2      = 0x0A;
static constexpr uint8_t REG_SET_RESET      = 0x0B;

QMC5883L::QMC5883L(hal::I2CDevice* i2c, hal::Time* time, uint8_t i2c_addr)
    : _i2c(i2c), _time(time), _i2c_addr(i2c_addr) {}

bool QMC5883L::init() {
    if (!_i2c || !_time) {
        _health = SensorHealth::FAILED;
        return false;
    }

    if (!_i2c->probe(_i2c_addr)) {
        _health = SensorHealth::DISCONNECTED;
        return false;
    }

    // Define SET/RESET period (recommended 0x01)
    _i2c->write_byte(_i2c_addr, REG_SET_RESET, 0x01);

    // Continuous mode, 100Hz ODR, 2G range, 512 oversampling ratio
    // Mode = Continuous (0b01), ODR = 100Hz (0b01 << 2), RNG = 2G (0b00 << 4), OSR = 512 (0b00 << 6)
    // 0b00000101 = 0x0D (continuous 50Hz) or 0b00011101 = 0x1D (continuous 200Hz)
    if (!_i2c->write_byte(_i2c_addr, REG_CONTROL_1, 0x1D)) {
        _health = SensorHealth::FAILED;
        return false;
    }

    record_success(_time->millis());
    return true;
}

bool QMC5883L::update() {
    if (!_i2c || !_time) {
        record_failure();
        return false;
    }

    uint8_t raw[6];
    if (!_i2c->read_registers(_i2c_addr, REG_DATA_OUT_X_LSB, raw, 6)) {
        record_failure();
        return false;
    }

    // LSB first for QMC5883L
    int16_t x = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    int16_t y = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    int16_t z = static_cast<int16_t>((raw[5] << 8) | raw[4]);

    _field_gauss.x = x * GAUSS_SCALE_2G;
    _field_gauss.y = y * GAUSS_SCALE_2G;
    _field_gauss.z = z * GAUSS_SCALE_2G;

    // Calculate planar heading
    _heading_rad = std::atan2(_field_gauss.y, _field_gauss.x);
    if (_heading_rad < 0.0f) {
        _heading_rad += 2.0f * static_cast<float>(M_PI);
    }

    record_success(_time->millis());
    return true;
}

} // namespace drivers
} // namespace autopilot
