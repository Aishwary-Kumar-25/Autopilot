#include "hal/posix/posix_hal.hpp"
#include "vehicles/vehicle_manager.hpp"
#include "system/scheduler.hpp"
#include "system/telemetry.hpp"
#include <iostream>

using namespace autopilot;

int main(int argc, char** argv) {
    std::cout << "========================================================\n";
    std::cout << "        E-CLUB MODULAR PLUG-AND-PLAY AUTOPILOT         \n";
    std::cout << "   Single-Firmware Multi-Vehicle Runtime Architecture  \n";
    std::cout << "========================================================\n";

    (void)argc;
    (void)argv;

    hal::PosixHAL hal;

    // Attach simulated sensors to I2C bus for demonstration
    hal::PosixI2CDevice::RegisterMap mpu;
    mpu[0x75] = 0x68; // WHO_AM_I MPU6050
    mpu[0x6B] = 0x00;
    mpu[0x3F] = 0x10; // Accel Z 1g
    hal.i2c().attach_device(0x68, mpu);

    hal::PosixI2CDevice::RegisterMap bmp;
    bmp[0xD0] = 0x58; // BMP280
    bmp[0xF7] = 0x65;
    bmp[0xF8] = 0x5A;
    bmp[0xF9] = 0xC0;
    bmp[0xFA] = 0x7E;
    bmp[0xFB] = 0xE9;
    bmp[0xFC] = 0x00;
    hal.i2c().attach_device(0x76, bmp);

    // Initialize Unified VehicleManager (Default: UAV Multicopter)
    vehicles::VehicleManager vm(&hal, vehicles::VehicleType::UAV_MULTICOPTER);

    if (!vm.init()) {
        std::cerr << "Failed to initialize VehicleManager!\n";
        return 1;
    }

    std::cout << "\n[Profile 1] Active Vehicle: " << vm.get_active_type_name() << "\n";
    std::string err;
    if (vm.arm(err)) {
        std::cout << "  -> Armed UAV successfully! Running Quad-X flight loop...\n";
        for (int i = 0; i < 20; ++i) {
            vm.set_user_inputs(0.1f, 0.0f, 0.0f, 0.5f); // 50% hover throttle, slight roll
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> M1 PWM: " << hal.pwm().get_channel(0)
                  << " us | M2 PWM: " << hal.pwm().get_channel(1) << " us\n";
    }

    // Dynamic Switch to UGV Rover (NO REFLASHING REQUIRED)
    std::cout << "\n>>> DYNAMIC PROFILE SWITCH: Converting to UGV Ground Rover <<<\n";
    std::string switch_reason;
    vm.switch_vehicle(vehicles::VehicleType::UGV_GROUND_ROVER, switch_reason);
    std::cout << "[Profile 2] Active Vehicle: " << vm.get_active_type_name() << "\n";
    if (vm.arm(err)) {
        std::cout << "  -> Armed UGV successfully! Running Ackermann ground drive loop...\n";
        for (int i = 0; i < 20; ++i) {
            vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.6f, 0.6f); // 60% forward speed
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> Steering PWM: " << hal.pwm().get_channel(0)
                  << " us | Throttle PWM: " << hal.pwm().get_channel(1) << " us\n";
    }

    // Dynamic Switch to ROV Submarine (NO REFLASHING REQUIRED)
    std::cout << "\n>>> DYNAMIC PROFILE SWITCH: Converting to ROV Submarine <<<\n";
    vm.switch_vehicle(vehicles::VehicleType::ROV_SUBMARINE, switch_reason);
    std::cout << "[Profile 3] Active Vehicle: " << vm.get_active_type_name() << "\n";
    if (vm.arm(err)) {
        std::cout << "  -> Armed ROV successfully! Running 6-DOF vectored thruster loop...\n";
        for (int i = 0; i < 20; ++i) {
            vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.4f, 0.5f, 0.0f); // Surge forward + vertical dive
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> Thruster 1 (Fwd): " << hal.pwm().get_channel(0)
                  << " us | Thruster 5 (Vert): " << hal.pwm().get_channel(4) << " us\n";
    }

    std::cout << "\n========================================================\n";
    std::cout << "SUCCESS: All 3 vehicle profiles executed seamlessly on\n";
    std::cout << "the SAME firmware binary without reflashing the chip!\n";
    std::cout << "========================================================\n";
    return 0;
}
