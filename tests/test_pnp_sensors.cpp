#include "hal/posix/posix_hal.hpp"
#include "drivers/sensor_manager.hpp"
#include "drivers/imu/mpu6050.hpp"
#include "drivers/imu/icm20689.hpp"
#include "drivers/baro/bmp280.hpp"
#include "tests/test_common.hpp"
#include <iostream>

using namespace autopilot;

void setup_simulated_mpu6050(hal::PosixI2CDevice& i2c, uint8_t addr = 0x68) {
    hal::PosixI2CDevice::RegisterMap regs;
    regs[0x75] = 0x68; // WHO_AM_I = 0x68
    regs[0x6B] = 0x00; // PWR_MGMT_1
    // Accel Z = 1g (4096 LSB for +/- 8g scale) => 0x1000
    regs[0x3B] = 0x00; // Accel X High
    regs[0x3C] = 0x00; // Accel X Low
    regs[0x3D] = 0x00; // Accel Y High
    regs[0x3E] = 0x00; // Accel Y Low
    regs[0x3F] = 0x10; // Accel Z High
    regs[0x40] = 0x00; // Accel Z Low
    // Gyros = 0
    regs[0x43] = 0x00;
    regs[0x44] = 0x00;
    regs[0x45] = 0x00;
    regs[0x46] = 0x00;
    regs[0x47] = 0x00;
    regs[0x48] = 0x00;
    i2c.attach_device(addr, regs);
}

void setup_simulated_icm20689(hal::PosixI2CDevice& i2c, uint8_t addr = 0x68) {
    hal::PosixI2CDevice::RegisterMap regs;
    regs[0x75] = 0x98; // WHO_AM_I = 0x98 for ICM20689
    regs[0x6B] = 0x00;
    regs[0x3F] = 0x10; // Accel Z = 1g
    i2c.attach_device(addr, regs);
}

void setup_simulated_bmp280(hal::PosixI2CDevice& i2c, uint8_t addr = 0x76) {
    hal::PosixI2CDevice::RegisterMap regs;
    regs[0xD0] = 0x58; // CHIP_ID = BMP280
    // Fill dummy calibration registers
    for (uint8_t r = 0x88; r <= 0xA1; ++r) {
        regs[r] = 0x01;
    }
    // Set pressure / temperature data regs
    regs[0xF7] = 0x80;
    regs[0xF8] = 0x00;
    regs[0xF9] = 0x00;
    regs[0xFA] = 0x80;
    regs[0xFB] = 0x00;
    regs[0xFC] = 0x00;
    i2c.attach_device(addr, regs);
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "TEST 1: Plug-and-Play Sensor Auto-Discovery & Hot-Swap\n";
    std::cout << "========================================================\n";

    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);

    // 1. Initial State: Plug in MPU6050 (0x68) and BMP280 (0x76)
    std::cout << "[Step 1] Attaching MPU6050 and BMP280 to simulated I2C bus...\n";
    setup_simulated_mpu6050(hal.i2c(), 0x68);
    setup_simulated_bmp280(hal.i2c(), 0x76);

    drivers::SensorManager sm(&hal);
    sm.auto_discover_sensors();

    CHECK(sm.has_healthy_imu());
    CHECK(sm.get_active_imu() != nullptr);
    std::cout << "  -> Discovered IMU: " << sm.get_active_imu()->name() << " [PASSED]\n";

    CHECK(sm.has_healthy_baro());
    CHECK(sm.get_active_baro() != nullptr);
    std::cout << "  -> Discovered Barometer: " << sm.get_active_baro()->name() << " [PASSED]\n";

    // Update sensor readings
    sm.update();
    CHECK(sm.get_active_imu()->is_healthy());

    // 2. Simulate Sensor Failure: Sensor goes bad / burns out / disconnected
    std::cout << "\n[Step 2] Simulating sensor hardware burnout / detachment...\n";
    hal.i2c().detach_device(0x68); // MPU6050 is completely unresponsive

    // Run several update cycles to trigger health degradation & failure detection
    for (int i = 0; i < 6; ++i) {
        hal.time().advance_time_ms(10);
        sm.update();
    }

    CHECK(!sm.has_healthy_imu());
    std::cout << "  -> Sensor failure detected! Active IMU is now invalid [PASSED]\n";

    // 3. Simulate Hot-Swap Replacement: Plug in a brand new ICM-20689 sensor
    std::cout << "\n[Step 3] Hot-plugging a replacement ICM-20689 sensor into the I2C port...\n";
    setup_simulated_icm20689(hal.i2c(), 0x68);

    // Advance time and trigger periodic hot-swap recovery probe
    hal.time().advance_time_ms(1200);
    sm.probe_and_recover_sensors();

    CHECK(sm.has_healthy_imu());
    std::cout << "  -> Auto-recovered with replacement sensor: " << sm.get_active_imu()->name() << " [PASSED]\n";

    std::cout << "\n>>> ALL PLUG-AND-PLAY SENSOR TESTS PASSED SUCCESSFULLY! <<<\n";
    return test::finish("test_pnp_sensors");
}
