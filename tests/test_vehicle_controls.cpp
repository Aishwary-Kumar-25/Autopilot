#include "hal/posix/posix_hal.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/ugv/ugv_vehicle.hpp"
#include "vehicles/rov/rov_vehicle.hpp"
#include <cassert>
#include <iostream>

using namespace autopilot;

void mock_healthy_sensors(hal::PosixHAL& hal) {
    // Provide simulated MPU6050 and BMP280 registers
    hal::PosixI2CDevice::RegisterMap mpu;
    mpu[0x75] = 0x68;
    mpu[0x6B] = 0x00;
    mpu[0x3F] = 0x10; // 1g Z
    hal.i2c().attach_device(0x68, mpu);

    hal::PosixI2CDevice::RegisterMap bmp;
    bmp[0xD0] = 0x58;
    for (uint8_t r = 0x88; r <= 0xA1; ++r) bmp[r] = 0x01;
    bmp[0xF7] = 0x80;
    bmp[0xFA] = 0x80;
    hal.i2c().attach_device(0x76, bmp);
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "TEST 3: Vehicle Controllers & Actuator PWM Outputs\n";
    std::cout << "========================================================\n";

    // ----------------- TEST 3A: UAV Multicopter -----------------
    std::cout << "\n[3A] Testing UAV (Quad-X Multicopter)...\n";
    hal::PosixHAL hal_uav;
    mock_healthy_sensors(hal_uav);

    vehicles::UAVVehicle uav(&hal_uav);
    assert(uav.init());

    // Before arming: PWM must be safe (1000us)
    uav.step(0.01f);
    for (uint8_t i = 0; i < 4; ++i) {
        assert(hal_uav.pwm().get_channel(i) == 1000);
    }
    std::cout << "  -> Disarmed PWM safety check [PASSED]\n";

    // Arming
    std::string err;
    assert(uav.arm(err));
    assert(uav.is_armed());
    std::cout << "  -> Pre-arm checks and Arming [PASSED]\n";

    // Commanded hover throttle = 0.5
    uav.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f);
    uav.step(0.01f);
    uint16_t m1 = hal_uav.pwm().get_channel(0);
    uint16_t m2 = hal_uav.pwm().get_channel(1);
    std::cout << "  -> Armed hover PWMs: M1=" << m1 << "us, M2=" << m2 << "us\n";
    assert(m1 >= 1400 && m1 <= 1600);

    // Roll right command: port motors (M2, M3) should increase, starboard (M1, M4) decrease
    uav.set_user_inputs(0.5f, 0.0f, 0.0f, 0.5f);
    uav.step(0.01f);
    uint16_t m1_roll = hal_uav.pwm().get_channel(0); // Front-Right
    uint16_t m2_roll = hal_uav.pwm().get_channel(1); // Rear-Left
    std::cout << "  -> Roll response: Right=" << m1_roll << "us, Left=" << m2_roll << "us\n";
    assert(m2_roll > m1_roll);
    std::cout << "  -> UAV Quad-X mixing dynamics [PASSED]\n";

    // ----------------- TEST 3B: UGV Ground Rover -----------------
    std::cout << "\n[3B] Testing UGV (Ground Rover)...\n";
    hal::PosixHAL hal_ugv;
    mock_healthy_sensors(hal_ugv);

    vehicles::UGVVehicle ugv(&hal_ugv, vehicles::UGVSteeringType::ACKERMANN);
    assert(ugv.init());
    assert(ugv.arm(err));

    // Forward command 0.5 -> Throttle ESC (CH1) should be > 1500us
    ugv.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f, 0.5f);
    ugv.step(0.01f);
    uint16_t steering = hal_ugv.pwm().get_channel(0);
    uint16_t throttle = hal_ugv.pwm().get_channel(1);
    std::cout << "  -> UGV Output: Steering=" << steering << "us, Throttle=" << throttle << "us\n";
    assert(steering == 1500); // Centered
    assert(throttle > 1650);  // Forward drive
    std::cout << "  -> UGV Ackermann steering & throttle [PASSED]\n";

    // ----------------- TEST 3C: ROV Submarine -----------------
    std::cout << "\n[3C] Testing ROV Submarine (ArduSub 6-DOF)...\n";
    hal::PosixHAL hal_rov;
    mock_healthy_sensors(hal_rov);

    vehicles::ROVVehicle rov(&hal_rov);
    assert(rov.init());
    assert(rov.arm(err));

    // Upward heave thrust (throttle = 0.5) -> Vertical thrusters (CH4, CH5) active
    rov.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f);
    rov.step(0.01f);
    uint16_t t_vert1 = hal_rov.pwm().get_channel(4);
    uint16_t t_vert2 = hal_rov.pwm().get_channel(5);
    std::cout << "  -> ROV Vertical Thrusters: T5=" << t_vert1 << "us, T6=" << t_vert2 << "us\n";
    assert(t_vert1 > 1650);
    assert(t_vert2 > 1650);

    // Forward surge thrust (forward = 0.5) -> Horizontal thrusters (CH0..3) active
    rov.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f);
    rov.step(0.01f);
    uint16_t t_horiz1 = hal_rov.pwm().get_channel(0);
    uint16_t t_horiz2 = hal_rov.pwm().get_channel(1);
    std::cout << "  -> ROV Forward Surge Thrusters: T1=" << t_horiz1 << "us, T2=" << t_horiz2 << "us\n";
    assert(t_horiz1 > 1650);
    assert(t_horiz2 > 1650);
    std::cout << "  -> ROV 6-DOF Thruster Matrix [PASSED]\n";

    std::cout << "\n>>> ALL VEHICLE CONTROLLER & PWM TESTS PASSED SUCCESSFULLY! <<<\n";
    return 0;
}
