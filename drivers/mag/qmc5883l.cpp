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
    : _i2c(i2c), _time(time), _i2c_addr(i2c_addr) {
    // Breakout boards are usually mounted component side up: chip z points up.
    _rotation = SensorRotation::ROLL_180;}

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

    // CONTROL_1 = 0x1D: MODE=01 continuous, ODR=11 200 Hz, RNG=01 8 G, OSR=00 512.
    // 8 G (not 2 G) so motor/battery fields near the frame can't saturate the chip.
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

    // Status first: DRDY says a new sample exists (reading the data clears it), so
    // the 6-byte data read only happens at the chip's 200 Hz rate, not every loop.
    uint8_t status = 0;
    if (!_i2c->read_byte(_i2c_addr, REG_STATUS, status)) {
        record_failure();
        return false;
    }
    if (status & 0x02) {
        // OVL: field beyond the 8 G range, the sample is garbage.
        record_failure();
        return false;
    }
    if (!(status & 0x01)) {
        return true; // no new sample yet (bus OK); timeouts are judged by SensorManager
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

    _field_gauss.x = x * GAUSS_SCALE_8G;
    _field_gauss.y = y * GAUSS_SCALE_8G;
    _field_gauss.z = z * GAUSS_SCALE_8G;

    // chip axes -> FRD body frame, then planar heading
    _field_gauss = apply_calibration(apply_rotation(_rotation, _field_gauss));
    _heading_rad = heading_from_field(_field_gauss);

    record_success(_time->millis());
    return true;
}

} // namespace drivers
} // namespace autopilot
