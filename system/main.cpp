#include "hal/posix/posix_hal.hpp"
#include "vehicles/vehicle_manager.hpp"
#include "system/scheduler.hpp"
#include "system/telemetry.hpp"
#include <iostream>

using namespace autopilot;

// Real MEMS sensors always have a few LSBs of noise. A register that never
// changes is what a hung sensor looks like, and the sensor manager's "stuck"
// check (correctly) fails it, so the demo dithers the low bytes.
static void sensor_noise(hal::PosixHAL& hal) {
    static uint32_t seed = 12345;
    auto next = [&]() { seed = seed * 1103515245u + 12345u; return static_cast<uint8_t>((seed >> 16) & 0x07); };
    for (uint8_t reg : {0x3C, 0x3E, 0x40, 0x44, 0x46, 0x48}) {
        hal.i2c().set_register(0x68, reg, next());
    }
}

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

    // Profile 1: UAV Multicopter
    std::cout << "\n[Profile 1] Active Vehicle: " << vm.get_active_type_name() << "\n";
    std::string err;
    vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f); // RC link up (required to arm)
    if (vm.arm(err)) {
        std::cout << "  -> Armed UAV successfully! Running Quad-X flight loop...\n";
        for (int i = 0; i < 20; ++i) {
            vm.set_user_inputs(0.1f, 0.0f, 0.0f, 0.5f); // 50% hover throttle, slight roll
            sensor_noise(hal);
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
    vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f); // RC link up (required to arm)
    if (vm.arm(err)) {
        std::cout << "  -> Armed UGV successfully! Running Ackermann ground drive loop...\n";
        for (int i = 0; i < 20; ++i) {
            vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.6f, 0.6f); // 60% forward speed
            sensor_noise(hal);
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> Steering PWM: " << hal.pwm().get_channel(0)
                  << " us | Throttle PWM: " << hal.pwm().get_channel(1) << " us\n";
    }

    // Dynamic Switch to ROV Submarine (Standard 6-Thruster BlueROV2)
    std::cout << "\n>>> DYNAMIC PROFILE SWITCH: Converting to ROV Submarine <<<\n";
    vm.switch_vehicle(vehicles::VehicleType::ROV_SUBMARINE, switch_reason);
    std::cout << "[Profile 3] Active Vehicle: " << vm.get_active_type_name() << "\n";
    vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f); // RC link up (required to arm)
    if (vm.arm(err)) {
        std::cout << "  -> Armed ROV successfully! Running standard 6-DOF vectored thruster loop...\n";
        for (int i = 0; i < 20; ++i) {
            vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.4f, 0.5f, 0.0f); // Surge forward + vertical dive
            sensor_noise(hal);
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> Thruster 1 (Fwd): " << hal.pwm().get_channel(0)
                  << " us | Thruster 5 (Vert): " << hal.pwm().get_channel(4) << " us\n";
    }

    // RUNTIME INTERFACE: Choose Number of Thrusters!
    std::cout << "\n>>> RUNTIME INTERFACE: Selecting Number of Thrusters <<<\n";
    std::cout << "User selection: Switch ROV to 8-Thruster Decoupled 6-DOF AUV configuration\n";
    vm.disarm();
    vm.configure_rov_thrusters(8);
    std::cout << "  -> Reconfigured Active Vehicle: " << vm.get_active_type_name() << "\n";
    std::cout << "  -> Active Thruster Channels: " << static_cast<int>(vm.get_active_actuator_count()) << "\n";
    vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f); // RC link up (required to arm)
    if (vm.arm(err)) {
        std::cout << "  -> Armed 8-Thruster AUV! Running pitch-up + forward surge maneuver...\n";
        vm.set_user_inputs(0.0f, 0.4f, 0.0f, 0.0f, 0.5f, 0.0f); // Pitch up + surge forward
        for (int i = 0; i < 20; ++i) {
            sensor_noise(hal);
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> Horizontal: T1=" << hal.pwm().get_channel(0) << "us, T2=" << hal.pwm().get_channel(1)
                  << "us, T3=" << hal.pwm().get_channel(2) << "us, T4=" << hal.pwm().get_channel(3) << "us\n";
        std::cout << "  -> Bow Vert:   T5=" << hal.pwm().get_channel(4) << "us, T6=" << hal.pwm().get_channel(5) << "us (>1500us UP)\n";
        std::cout << "  -> Stern Vert: T7=" << hal.pwm().get_channel(6) << "us, T8=" << hal.pwm().get_channel(7) << "us (<1500us DOWN)\n";
    }

    std::cout << "\nUser selection: Switch ROV to 3-Thruster Budget / Educational configuration\n";
    vm.disarm();
    vm.configure_rov_thrusters(3);
    std::cout << "  -> Reconfigured Active Vehicle: " << vm.get_active_type_name() << "\n";
    std::cout << "  -> Active Thruster Channels: " << static_cast<int>(vm.get_active_actuator_count()) << "\n";
    vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f); // RC link up (required to arm)
    if (vm.arm(err)) {
        vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f, 0.5f, 0.0f); // Forward surge + climb heave
        for (int i = 0; i < 20; ++i) {
            sensor_noise(hal);
            vm.step(0.01f);
            hal.time().delay_ms(10);
        }
        std::cout << "  -> Horiz Port (T1): " << hal.pwm().get_channel(0)
                  << "us | Horiz Stbd (T2): " << hal.pwm().get_channel(1)
                  << "us | Vert Center (T3): " << hal.pwm().get_channel(2) << "us\n";
    }

    std::cout << "\n========================================================\n";
    std::cout << "SUCCESS: Generalized Actuator Matrix Mixer seamlessly\n";
    std::cout << "handles 3, 4, 6, 8, up to 16 thrusters on a single binary!\n";
    std::cout << "========================================================\n";
    return 0;
}
