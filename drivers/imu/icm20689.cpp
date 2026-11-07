#include "icm20689.hpp"

namespace autopilot {
namespace drivers {

static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;

ICM20689::ICM20689(hal::I2CDevice* i2c, hal::Time* time, uint8_t i2c_addr)
    : _i2c(i2c), _time(time), _i2c_addr(i2c_addr) {}

bool ICM20689::verify_device_id() {
    if (!_i2c) return false;
    uint8_t who = 0;
    if (!_i2c->read_byte(_i2c_addr, REG_WHO_AM_I, who)) {
        return false;
    }
    return (who == EXPECTED_WHO_AM_I);
}

bool ICM20689::init() {
    if (!_i2c || !_time) {
        _health = SensorHealth::FAILED;
        return false;
    }

    if (!verify_device_id()) {
        _health = SensorHealth::DISCONNECTED;
        return false;
    }

    // Reset device
    if (!_i2c->write_byte(_i2c_addr, REG_PWR_MGMT_1, 0x80)) {
        _health = SensorHealth::FAILED;
        return false;
    }
    _time->delay_ms(10);

    // Auto select clock source
    if (!_i2c->write_byte(_i2c_addr, REG_PWR_MGMT_1, 0x01)) {
        _health = SensorHealth::FAILED;
        return false;
    }

    _i2c->write_byte(_i2c_addr, REG_SMPLRT_DIV, 0x01);
    _i2c->write_byte(_i2c_addr, REG_CONFIG, 0x03);       // 42Hz DLPF
    _i2c->write_byte(_i2c_addr, REG_GYRO_CONFIG, 0x18);  // +/- 2000 dps
    _i2c->write_byte(_i2c_addr, REG_ACCEL_CONFIG, 0x10); // +/- 8g

    calibrate_gyro_bias(50);

    record_success(_time->millis());
    return true;
}

bool ICM20689::update() {
    if (!_i2c || !_time) {
        record_failure();
        return false;
    }

    uint8_t raw[14];
    if (!_i2c->read_registers(_i2c_addr, REG_ACCEL_XOUT_H, raw, 14)) {
        record_failure();
        return false;
    }

    int16_t ax = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    int16_t ay = static_cast<int16_t>((raw[2] << 8) | raw[3]);
    int16_t az = static_cast<int16_t>((raw[4] << 8) | raw[5]);
    int16_t temp = static_cast<int16_t>((raw[6] << 8) | raw[7]);
    int16_t gx = static_cast<int16_t>((raw[8] << 8) | raw[9]);
    int16_t gy = static_cast<int16_t>((raw[10] << 8) | raw[11]);
    int16_t gz = static_cast<int16_t>((raw[12] << 8) | raw[13]);

    float raw_ax = ax * ACCEL_SCALE_8G;
    float raw_ay = ay * ACCEL_SCALE_8G;
    float raw_az = az * ACCEL_SCALE_8G;

    _accel_m_s2.x = _accel_filter_x.apply(raw_ax);
    _accel_m_s2.y = _accel_filter_y.apply(raw_ay);
    _accel_m_s2.z = _accel_filter_z.apply(raw_az);

    _gyro_rad_s.x = (gx * GYRO_SCALE_2000DPS) - _gyro_bias.x;
    _gyro_rad_s.y = (gy * GYRO_SCALE_2000DPS) - _gyro_bias.y;
    _gyro_rad_s.z = (gz * GYRO_SCALE_2000DPS) - _gyro_bias.z;

    _temperature_c = (temp / 326.8f) + 25.0f;

    record_success(_time->millis());
    return true;
}

void ICM20689::calibrate_gyro_bias(uint16_t sample_count) {
    if (sample_count == 0) return;

    math::Vector3f sum_gyro(0.0f, 0.0f, 0.0f);
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        uint8_t raw[6];
        if (_i2c->read_registers(_i2c_addr, 0x43, raw, 6)) {
            int16_t gx = static_cast<int16_t>((raw[0] << 8) | raw[1]);
            int16_t gy = static_cast<int16_t>((raw[2] << 8) | raw[3]);
            int16_t gz = static_cast<int16_t>((raw[4] << 8) | raw[5]);

            sum_gyro.x += gx * GYRO_SCALE_2000DPS;
            sum_gyro.y += gy * GYRO_SCALE_2000DPS;
            sum_gyro.z += gz * GYRO_SCALE_2000DPS;
            valid_samples++;
        }
        _time->delay_ms(2);
    }

    if (valid_samples > 0) {
        _gyro_bias = sum_gyro / static_cast<float>(valid_samples);
    }
}

} // namespace drivers
} // namespace autopilot
