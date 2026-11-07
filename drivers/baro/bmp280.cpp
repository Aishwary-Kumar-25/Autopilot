#include "bmp280.hpp"
#include <cmath>
#include <cstring>

namespace autopilot {
namespace drivers {

static constexpr uint8_t REG_CALIB_START = 0x88;
static constexpr uint8_t REG_CTRL_MEAS   = 0xF4;
static constexpr uint8_t REG_CONFIG      = 0xF5;
static constexpr uint8_t REG_PRESS_MSB   = 0xF7;

BMP280::BMP280(hal::I2CDevice* i2c, hal::Time* time, uint8_t i2c_addr)
    : _i2c(i2c), _time(time), _i2c_addr(i2c_addr) {}

bool BMP280::read_calibration_data() {
    uint8_t calib[24];
    if (!_i2c->read_registers(_i2c_addr, REG_CALIB_START, calib, 24)) {
        return false;
    }

    _dig_T1 = static_cast<uint16_t>(calib[1] << 8 | calib[0]);
    _dig_T2 = static_cast<int16_t>(calib[3] << 8 | calib[2]);
    _dig_T3 = static_cast<int16_t>(calib[5] << 8 | calib[4]);

    _dig_P1 = static_cast<uint16_t>(calib[7] << 8 | calib[6]);
    _dig_P2 = static_cast<int16_t>(calib[9] << 8 | calib[8]);
    _dig_P3 = static_cast<int16_t>(calib[11] << 8 | calib[10]);
    _dig_P4 = static_cast<int16_t>(calib[13] << 8 | calib[12]);
    _dig_P5 = static_cast<int16_t>(calib[15] << 8 | calib[14]);
    _dig_P6 = static_cast<int16_t>(calib[17] << 8 | calib[16]);
    _dig_P7 = static_cast<int16_t>(calib[19] << 8 | calib[18]);
    _dig_P8 = static_cast<int16_t>(calib[21] << 8 | calib[20]);
    _dig_P9 = static_cast<int16_t>(calib[23] << 8 | calib[22]);

    // Blank/corrupt NVM (all 0x00 or 0xFF reads) means a broken chip or bus. Never
    // substitute the datasheet's example coefficients: that yields a plausible but
    // wrong pressure that the health checks can't catch.
    if (_dig_T1 == 0 || _dig_P1 == 0 || _dig_T1 == 0xFFFF || _dig_P1 == 0xFFFF) {
        return false;
    }

    return true;
}

bool BMP280::init() {
    if (!_i2c || !_time) {
        _health = SensorHealth::FAILED;
        return false;
    }

    uint8_t chip_id = 0;
    if (!_i2c->read_byte(_i2c_addr, REG_CHIP_ID, chip_id)) {
        _health = SensorHealth::DISCONNECTED;
        return false;
    }

    if (chip_id != EXPECTED_CHIP_ID && chip_id != 0x60) {
        _health = SensorHealth::FAILED;
        return false;
    }

    if (!read_calibration_data()) {
        _health = SensorHealth::FAILED;
        return false;
    }

    // Normal mode, temp oversampling x2, press oversampling x16
    // osrs_t = 010 (x2), osrs_p = 101 (x16), mode = 11 (normal) => 0b01010111 = 0x57
    if (!_i2c->write_byte(_i2c_addr, REG_CTRL_MEAS, 0x57)) {
        _health = SensorHealth::FAILED;
        return false;
    }

    // Standby time 0.5ms, IIR filter coefficient 16 (0b00010000 = 0x10)
    _i2c->write_byte(_i2c_addr, REG_CONFIG, 0x10);

    // Initial update to seed ground reference
    _time->delay_ms(15);
    _have_read = false;
    if (update()) {
        set_ground_reference(_pressure_pa);
        // The first altitude was computed against the default 101325 Pa reference;
        // restart the filter so it doesn't begin tens of metres off and decay.
        _altitude_filter.reset();
        _relative_altitude_m = 0.0f;
    }

    record_success(_time->millis());
    return true;
}

bool BMP280::update() {
    if (!_i2c || !_time) {
        record_failure();
        return false;
    }

    // The chip produces ~26 samples/s (x16 pressure oversampling). Reading it every
    // 400 Hz loop only repeats values and burns I2C bus time, and the altitude
    // filter is designed for 50 Hz, so read at most every MIN_READ_INTERVAL_MS.
    const uint32_t now = _time->millis();
    if (_have_read && now - _last_read_ms < MIN_READ_INTERVAL_MS) {
        return true;
    }
    _last_read_ms = now;
    _have_read = true;

    uint8_t data[6];
    if (!_i2c->read_registers(_i2c_addr, REG_PRESS_MSB, data, 6)) {
        record_failure();
        return false;
    }

    int32_t adc_P = static_cast<int32_t>((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t adc_T = static_cast<int32_t>((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));

    // Temperature compensation formula from Bosch datasheet
    int32_t var1 = ((((adc_T >> 3) - (static_cast<int32_t>(_dig_T1) << 1))) * static_cast<int32_t>(_dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(_dig_T1)) *
                      ((adc_T >> 4) - static_cast<int32_t>(_dig_T1))) >> 12) *
                    static_cast<int32_t>(_dig_T3)) >> 14;
    _t_fine = var1 + var2;
    _temperature_c = static_cast<float>((_t_fine * 5 + 128) >> 8) / 100.0f;

    // Pressure compensation formula from Bosch datasheet
    int64_t p_var1 = static_cast<int64_t>(_t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * static_cast<int64_t>(_dig_P6);
    // Bosch's reference code left-shifts signed (possibly negative) values, which is
    // undefined behaviour in C++17; multiply by the power of two instead (same result).
    p_var2 = p_var2 + ((p_var1 * static_cast<int64_t>(_dig_P5)) * 131072);
    p_var2 = p_var2 + (static_cast<int64_t>(_dig_P4) * (static_cast<int64_t>(1) << 35));
    p_var1 = ((p_var1 * p_var1 * static_cast<int64_t>(_dig_P3)) >> 8) +
             ((p_var1 * static_cast<int64_t>(_dig_P2)) * 4096);
    p_var1 = (((static_cast<int64_t>(1) << 47) + p_var1)) * static_cast<int64_t>(_dig_P1) >> 33;

    if (p_var1 == 0) {
        record_failure();
        return false;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (static_cast<int64_t>(_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (static_cast<int64_t>(_dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (static_cast<int64_t>(_dig_P7) << 4);

    _pressure_pa = static_cast<float>(p) / 256.0f;

    float raw_alt = calculate_altitude(_pressure_pa, _ground_pressure_pa);
    _relative_altitude_m = _altitude_filter.apply(raw_alt);

    record_success(_time->millis());
    return true;
}

void BMP280::set_ground_reference(float ground_pressure_pa) {
    if (ground_pressure_pa > 50000.0f && ground_pressure_pa < 115000.0f) {
        _ground_pressure_pa = ground_pressure_pa;
    }
}

float BMP280::calculate_altitude(float pressure_pa, float ground_pa) {
    if (ground_pa <= 0.0f || pressure_pa <= 0.0f) return 0.0f;
    // Standard barometric formula: h = 44330 * (1 - (P / P0)^0.190295)
    return 44330.0f * (1.0f - std::pow(pressure_pa / ground_pa, 0.190295f));
}

} // namespace drivers
} // namespace autopilot
