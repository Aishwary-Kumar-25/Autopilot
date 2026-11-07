/**
 * Driver-level fixes: QMC5883L scale/status, BMP280 calibration + read rate,
 * gyro-calibration stillness check, zero-allocation NMEA parser, scheduler timing.
 */
#include "hal/posix/posix_hal.hpp"
#include "drivers/mag/qmc5883l.hpp"
#include "drivers/baro/bmp280.hpp"
#include "drivers/imu/gyro_cal.hpp"
#include "drivers/gps/nmea.hpp"
#include "system/scheduler.hpp"
#include "drivers/sensor_manager.hpp"
#include "tests/fake_sensors.hpp"
#include <memory>
#include "tests/test_common.hpp"
#include <cmath>
#include <cstdio>
#include <string>

using namespace autopilot;

static std::string with_checksum(const std::string& body) {
    uint8_t sum = 0;
    for (char c : body) sum ^= static_cast<uint8_t>(c);
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X", sum);
    return "$" + body + tail;
}

static void test_qmc5883l() {
    std::printf("[QMC5883L]\n");
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);
    hal::PosixI2CDevice::RegisterMap regs;
    // x = +3000 counts, y = -1500, z = 600 (LSB first), status DRDY
    regs[0x00] = 0xB8; regs[0x01] = 0x0B;
    regs[0x02] = 0x24; regs[0x03] = 0xFA;
    regs[0x04] = 0x58; regs[0x05] = 0x02;
    regs[0x06] = 0x01;
    hal.i2c().attach_device(0x0D, regs);
    drivers::QMC5883L mag(&hal.i2c(), &hal.time(), 0x0D);
    CHECK(mag.init());
    CHECK(hal.i2c().get_register(0x0D, 0x09) == 0x1D); // 8 G range configured...
    CHECK(mag.update());
    // ...and 8 G scale used: 3000 counts = 1.0 G (ROLL_180: x kept, y and z negated)
    CHECK_NEAR(mag.get_field_gauss().x, 1.0f, 1e-4);
    CHECK_NEAR(mag.get_field_gauss().y, 0.5f, 1e-4);
    CHECK_NEAR(mag.get_field_gauss().z, -0.2f, 1e-4);

    // No DRDY: no new sample, no bus error
    const uint32_t n = mag.sample_count();
    hal.i2c().set_register(0x0D, 0x06, 0x00);
    CHECK(mag.update());
    CHECK(mag.sample_count() == n);
    CHECK(mag.health() == drivers::SensorHealth::HEALTHY);

    // Overflow: rejected
    hal.i2c().set_register(0x0D, 0x06, 0x03);
    CHECK(!mag.update());
    CHECK(mag.sample_count() == n);

    // Calibration is applied in the body frame
    hal.i2c().set_register(0x0D, 0x06, 0x01);
    mag.set_calibration(math::Vector3f(0.5f, 0.0f, 0.0f), math::Vector3f(2.0f, 1.0f, 1.0f));
    CHECK(mag.update());
    CHECK_NEAR(mag.get_field_gauss().x, 1.0f, 1e-4);              // (1.0 - 0.5) * 2
    CHECK_NEAR(mag.get_field_uncalibrated().x, 1.0f, 1e-4);
}

static hal::PosixI2CDevice::RegisterMap bmp_regs(uint8_t calib_fill) {
    hal::PosixI2CDevice::RegisterMap r;
    r[0xD0] = 0x58;
    for (uint8_t a = 0x88; a <= 0x9F; ++a) r[a] = calib_fill;
    r[0xF7] = 0x80; r[0xFA] = 0x80;
    return r;
}

static void test_bmp280() {
    std::printf("[BMP280]\n");
    {
        hal::PosixHAL hal;
        hal.time().set_manual_time_mode(true);
        hal.i2c().attach_device(0x76, bmp_regs(0x00)); // blank NVM
        drivers::BMP280 baro(&hal.i2c(), &hal.time(), 0x76);
        CHECK(!baro.init()); // was: silently used the datasheet example coefficients
    }
    {
        hal::PosixHAL hal;
        hal.time().set_manual_time_mode(true);
        hal.i2c().attach_device(0x76, bmp_regs(0x01));
        drivers::BMP280 baro(&hal.i2c(), &hal.time(), 0x76);
        CHECK(baro.init());
        CHECK_NEAR(baro.get_altitude_m(), 0.0f, 1e-3); // filter restarted after the ground reference
        hal.time().advance_time_ms(25);
        CHECK(baro.update());
        const uint32_t n = baro.sample_count();
        hal.time().advance_time_ms(5);
        CHECK(baro.update());                 // within 20 ms: no bus read, no new sample
        CHECK(baro.sample_count() == n);
        hal.time().advance_time_ms(20);
        CHECK(baro.update());
        CHECK(baro.sample_count() == n + 1);
    }
    {
        // Bosch datasheet worked example (BMP280 rev 1.14, sec. 8.2):
        // adc_T = 519888, adc_P = 415148 -> T = 25.08 C, P = 100653.27 Pa
        const int32_t cal[12] = {27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};
        hal::PosixHAL hal;
        hal.time().set_manual_time_mode(true);
        hal::PosixI2CDevice::RegisterMap r;
        r[0xD0] = 0x58;
        for (int i = 0; i < 12; ++i) {
            const uint16_t v = static_cast<uint16_t>(cal[i]);
            r[0x88 + 2 * i] = v & 0xFF;
            r[0x89 + 2 * i] = v >> 8;
        }
        const uint32_t adc_p = 415148, adc_t = 519888;
        r[0xF7] = adc_p >> 12; r[0xF8] = (adc_p >> 4) & 0xFF; r[0xF9] = (adc_p & 0xF) << 4;
        r[0xFA] = adc_t >> 12; r[0xFB] = (adc_t >> 4) & 0xFF; r[0xFC] = (adc_t & 0xF) << 4;
        hal.i2c().attach_device(0x76, r);
        drivers::BMP280 baro(&hal.i2c(), &hal.time(), 0x76);
        CHECK(baro.init());
        CHECK_NEAR(baro.get_temperature_c(), 25.08f, 0.01);
        CHECK_NEAR(baro.get_pressure_pa(), 100653.27f, 0.5);
    }
}

static void test_gyro_cal() {
    std::printf("[gyro calibration stillness]\n");
    uint32_t s = 1;
    auto noise = [&]() { s = s * 1664525u + 1013904223u; return (static_cast<float>(s >> 8) / 16777216.0f - 0.5f) * 0.02f; };
    drivers::GyroCalAccumulator still, moving, big_bias;
    for (int i = 0; i < 50; ++i) {
        still.add(math::Vector3f(0.01f + noise(), -0.02f + noise(), 0.005f + noise()));
        moving.add(math::Vector3f(0.3f * std::sin(i * 0.3f), noise(), noise()));
        big_bias.add(math::Vector3f(0.5f + noise(), noise(), noise()));
    }
    CHECK(still.still());
    CHECK_NEAR(still.mean().x, 0.01f, 0.003);
    CHECK_NEAR(still.mean().y, -0.02f, 0.003);
    CHECK(!moving.still());
    CHECK(!big_bias.still());
    drivers::GyroCalAccumulator few;
    for (int i = 0; i < 5; ++i) few.add(math::Vector3f(0, 0, 0));
    CHECK(!few.still());
}

static void test_nmea() {
    std::printf("[NMEA parser]\n");
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);
    drivers::NMEAGPS gps(&hal.uart(), &hal.time());
    CHECK(gps.init());

    CHECK(drivers::NMEAGPS::checksum_ok("$GPGGA,1*57") == ((0 ^ 'G' ^ 'P' ^ 'G' ^ 'G' ^ 'A' ^ ',' ^ '1') == 0x57));
    CHECK(!drivers::NMEAGPS::checksum_ok("$GPGGA,1"));
    CHECK(!drivers::NMEAGPS::checksum_ok("$GPGGA,1*G0"));

    const std::string gga = with_checksum("GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    const std::string rmc = with_checksum("GPRMC,123519,A,4807.038,N,01131.000,E,010.0,090.0,230394,003.1,W");
    // Split across reads, with line noise before and a missing CR/LF between sentences.
    hal.uart().inject_input("xx\x01garbage" + gga.substr(0, 20));
    gps.update();
    hal.uart().inject_input(gga.substr(20) + rmc + "\r\n");
    gps.update();
    const auto& d = gps.get_data();
    CHECK(gps.has_3d_fix());
    CHECK_NEAR(d.latitude_deg, 48.1173, 1e-4);
    CHECK_NEAR(d.longitude_deg, 11.516667, 1e-4);
    CHECK(d.satellites == 8);
    CHECK_NEAR(d.hdop, 0.9f, 1e-4);
    CHECK_NEAR(d.altitude_msl_m, 545.4f, 1e-3);
    CHECK_NEAR(d.speed_m_s, 5.14444f, 1e-3);
    CHECK_NEAR(d.velocity_ned.y, 5.14444f, 1e-3);  // course 090 = east
    CHECK_NEAR(d.velocity_ned.x, 0.0f, 1e-3);

    // Corrupted sentence is ignored
    std::string bad = with_checksum("GPGGA,123520,1000.000,S,01000.000,W,1,09,0.8,10.0,M,0,M,,");
    bad[10] = '9';
    gps.parse_sentence(bad);
    CHECK_NEAR(gps.get_data().latitude_deg, 48.1173, 1e-4);
    CHECK(gps.bad_sentences() >= 1);

    // GSA says 2D: no 3D fix
    gps.parse_sentence(with_checksum("GPGSA,A,2,04,05,,,,,,,,,,,2.5,1.3,2.1"));
    CHECK(!gps.has_3d_fix());
    gps.parse_sentence(with_checksum("GPGSA,A,3,04,05,09,12,,,,,,,,,2.5,1.3,2.1"));
    CHECK(gps.has_3d_fix());
    CHECK_NEAR(gps.get_data().hdop, 1.3f, 1e-4);

    // RMC status V (receiver lost the fix) clears it
    gps.parse_sentence(with_checksum("GPRMC,123521,V,,,,,,,230394,,"));
    CHECK(!gps.has_3d_fix());

    // Over-long line is dropped whole, and the parser recovers on the next '$'
    hal.uart().inject_input("$GPGGA," + std::string(200, '1') + "\r\n" + gga + "\r\n");
    gps.update();
    CHECK(gps.has_3d_fix());
}

static void test_scheduler() {
    std::printf("[scheduler rate]\n");
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);
    system::Scheduler sched(&hal.time());
    int fast = 0, slow = 0;
    sched.add_task("fast", [&] { ++fast; }, 400);
    sched.add_task("slow", [&] { ++slow; }, 50);
    // Poll every 300 us for 1 s (the main loop never lands exactly on a deadline).
    for (int i = 0; i < 3334; ++i) {
        sched.run();
        hal.time().delay_us(300);
    }
    std::printf("  400 Hz task ran %d times, 50 Hz task %d times in 1 s\n", fast, slow);
    CHECK(fast >= 399 && fast <= 401);   // the old "last_run = now" logic gave ~370
    CHECK(slow >= 49 && slow <= 51);
    CHECK(sched.tasks()[0].overruns == 0);
}

static void test_gps_selection() {
    std::printf("[GPS health and selection]\n");
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);
    drivers::SensorManager sm(&hal);
    auto silent = std::make_shared<drivers::NMEAGPS>(&hal.uart(), &hal.time()); // never sends a sentence
    silent->init();
    sm.register_sensor(silent);
    for (int i = 0; i < 20; ++i) { hal.time().advance_time_ms(10); sm.update(); }
    CHECK(!sm.has_healthy_gps());            // alive UART but no fix: not a usable GPS
    auto good = std::make_shared<test::FakeGPS>(&hal.time());
    good->data.has_fix = true; good->data.fix_type = 3;
    good->init();
    sm.register_sensor(good);
    for (int i = 0; i < 20; ++i) { hal.time().advance_time_ms(10); sm.update(); }
    CHECK(sm.get_active_gps() == good.get()); // the GPS with a fix takes over
    CHECK(sm.has_healthy_gps());
}

int main() {
    test_gps_selection();
    test_qmc5883l();
    test_bmp280();
    test_gyro_cal();
    test_nmea();
    test_scheduler();
    return test::finish("test_drivers");
}
