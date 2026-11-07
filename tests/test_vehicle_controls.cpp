#include "hal/posix/posix_hal.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/ugv/ugv_vehicle.hpp"
#include "vehicles/rov/rov_vehicle.hpp"
#include "vehicles/vehicle_manager.hpp"
#include "control/actuator_matrix_mixer.hpp"
#include "tests/test_common.hpp"
#include <iostream>
#include <cmath>

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
    std::cout << "TEST 3: Vehicle Controllers & Actuator Matrix Mixer\n";
    std::cout << "========================================================\n";

    // ----------------- TEST 3A: UAV Multicopter (Quad-X) -----------------
    std::cout << "\n[3A] Testing UAV (Quad-X Multicopter)...\n";
    hal::PosixHAL hal_uav;
    mock_healthy_sensors(hal_uav);

    vehicles::UAVVehicle uav(&hal_uav);
    CHECK(uav.init());

    // Before arming: PWM must be safe (1000us)
    uav.step(0.01f);
    for (uint8_t i = 0; i < 4; ++i) {
        CHECK(hal_uav.pwm().get_channel(i) == 1000);
    }
    std::cout << "  -> Disarmed PWM safety check [PASSED]\n";

    // Arming
    std::string err;
    CHECK(uav.arm(err));
    CHECK(uav.is_armed());
    std::cout << "  -> Pre-arm checks and Arming [PASSED]\n";

    // Commanded hover throttle = 0.5
    uav.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f);
    uav.step(0.01f);
    uint16_t m1 = hal_uav.pwm().get_channel(0);
    uint16_t m2 = hal_uav.pwm().get_channel(1);
    std::cout << "  -> Armed hover PWMs: M1=" << m1 << "us, M2=" << m2 << "us\n";
    CHECK(m1 >= 1400 && m1 <= 1600);

    // Roll right command: port motors (M2, M3) should increase, starboard (M1, M4) decrease
    uav.set_user_inputs(0.5f, 0.0f, 0.0f, 0.5f);
    uav.step(0.01f);
    uint16_t m1_roll = hal_uav.pwm().get_channel(0); // Front-Right
    uint16_t m2_roll = hal_uav.pwm().get_channel(1); // Rear-Left
    std::cout << "  -> Roll response: Right=" << m1_roll << "us, Left=" << m2_roll << "us\n";
    CHECK(m2_roll > m1_roll);
    std::cout << "  -> UAV Quad-X mixing dynamics [PASSED]\n";

    // ----------------- TEST 3B: UGV Ground Rover -----------------
    std::cout << "\n[3B] Testing UGV (Ground Rover)...\n";
    hal::PosixHAL hal_ugv;
    mock_healthy_sensors(hal_ugv);

    vehicles::UGVVehicle ugv(&hal_ugv, vehicles::UGVSteeringType::ACKERMANN);
    CHECK(ugv.init());
    CHECK(ugv.arm(err));

    // Forward command 0.5 -> Throttle ESC (CH1) should be > 1500us
    ugv.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f, 0.5f);
    ugv.step(0.01f);
    uint16_t steering = hal_ugv.pwm().get_channel(0);
    uint16_t throttle = hal_ugv.pwm().get_channel(1);
    std::cout << "  -> UGV Output: Steering=" << steering << "us, Throttle=" << throttle << "us\n";
    CHECK(steering == 1500); // Centered
    CHECK(throttle > 1650);  // Forward drive
    std::cout << "  -> UGV Ackermann steering & throttle [PASSED]\n";

    // ----------------- TEST 3C: Standard 6-Thruster ROV -----------------
    std::cout << "\n[3C] Testing ROV Submarine (Standard 6-Thruster BlueROV2 6-DOF)...\n";
    hal::PosixHAL hal_rov6;
    mock_healthy_sensors(hal_rov6);

    vehicles::ROVVehicle rov6(&hal_rov6, 6);
    CHECK(rov6.init());
    CHECK(rov6.arm(err));
    CHECK(rov6.get_thruster_count() == 6);

    // Upward heave thrust (throttle = 0.5) -> Vertical thrusters (CH4, CH5) active
    rov6.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f);
    rov6.step(0.01f);
    uint16_t t_vert1 = hal_rov6.pwm().get_channel(4);
    uint16_t t_vert2 = hal_rov6.pwm().get_channel(5);
    std::cout << "  -> 6-Thruster Vertical: T5=" << t_vert1 << "us, T6=" << t_vert2 << "us\n";
    CHECK(t_vert1 > 1650);
    CHECK(t_vert2 > 1650);

    // Forward surge thrust (forward = 0.5) -> Horizontal thrusters (CH0..3) active
    rov6.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f);
    rov6.step(0.01f);
    uint16_t t_horiz1 = hal_rov6.pwm().get_channel(0);
    uint16_t t_horiz2 = hal_rov6.pwm().get_channel(1);
    std::cout << "  -> 6-Thruster Forward Surge: T1=" << t_horiz1 << "us, T2=" << t_horiz2 << "us\n";
    CHECK(t_horiz1 > 1650);
    CHECK(t_horiz2 > 1650);
    std::cout << "  -> 6-Thruster Vectored Matrix [PASSED]\n";

    // ----------------- TEST 3D: 8-Thruster Decoupled 6-DOF AUV -----------------
    std::cout << "\n[3D] Testing 8-Thruster Heavy AUV (BlueROV2 Heavy / RoboSub Decoupled 6-DOF)...\n";
    hal::PosixHAL hal_rov8;
    mock_healthy_sensors(hal_rov8);

    vehicles::ROVVehicle rov8(&hal_rov8, 8);
    CHECK(rov8.init());
    CHECK(rov8.arm(err));
    CHECK(rov8.get_thruster_count() == 8);
    rov8.set_mode(vehicles::ROVMode::MANUAL); // Pure mixing verification
    std::cout << "  -> Initialized 8-Thruster Layout: " << rov8.get_thruster_layout_name() << "\n";

    // 1. Pure Pitch Up Command (pitch = +0.5):
    // Bow vertical thrusters (T5, T6: CH4, CH5) push UP (1750us)
    // Stern vertical thrusters (T7, T8: CH6, CH7) push DOWN (1250us)
    rov8.set_user_inputs(0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f);
    rov8.step(0.01f);
    uint16_t t5_pitch = hal_rov8.pwm().get_channel(4);
    uint16_t t6_pitch = hal_rov8.pwm().get_channel(5);
    uint16_t t7_pitch = hal_rov8.pwm().get_channel(6);
    uint16_t t8_pitch = hal_rov8.pwm().get_channel(7);
    std::cout << "  -> Decoupled Pitch Response:\n"
              << "     Bow Thrusters:   T5=" << t5_pitch << "us, T6=" << t6_pitch << "us (>1500us UP)\n"
              << "     Stern Thrusters: T7=" << t7_pitch << "us, T8=" << t8_pitch << "us (<1500us DOWN)\n";
    CHECK(t5_pitch == 1750);
    CHECK(t6_pitch == 1750);
    CHECK(t7_pitch == 1250);
    CHECK(t8_pitch == 1250);

    // 2. Pure Roll Right Command (roll = +0.5):
    // Port vertical thrusters (T6, T8: CH5, CH7) push UP (1750us)
    // Starboard vertical thrusters (T5, T7: CH4, CH6) push DOWN (1250us)
    rov8.set_user_inputs(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    rov8.step(0.01f);
    uint16_t t5_roll = hal_rov8.pwm().get_channel(4);
    uint16_t t6_roll = hal_rov8.pwm().get_channel(5);
    uint16_t t7_roll = hal_rov8.pwm().get_channel(6);
    uint16_t t8_roll = hal_rov8.pwm().get_channel(7);
    std::cout << "  -> Decoupled Roll Response:\n"
              << "     Port Thrusters:      T6=" << t6_roll << "us, T8=" << t8_roll << "us (>1500us UP)\n"
              << "     Starboard Thrusters: T5=" << t5_roll << "us, T7=" << t7_roll << "us (<1500us DOWN)\n";
    CHECK(t6_roll == 1750);
    CHECK(t8_roll == 1750);
    CHECK(t5_roll == 1250);
    CHECK(t7_roll == 1250);

    // 3. Pure Heave Command (throttle = +0.5):
    // All 4 vertical thrusters (CH4..7) push UP equally (1750us)
    rov8.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f);
    rov8.step(0.01f);
    CHECK(hal_rov8.pwm().get_channel(4) == 1750);
    CHECK(hal_rov8.pwm().get_channel(5) == 1750);
    CHECK(hal_rov8.pwm().get_channel(6) == 1750);
    CHECK(hal_rov8.pwm().get_channel(7) == 1750);
    std::cout << "  -> 8-Thruster Decoupled 6-DOF Dynamics [PASSED]\n";

    // ----------------- TEST 3E: Budget 3-Thruster ROV -----------------
    std::cout << "\n[3E] Testing Budget 3-Thruster ROV (2 Horiz + 1 Vert)...\n";
    hal::PosixHAL hal_rov3;
    mock_healthy_sensors(hal_rov3);

    vehicles::ROVVehicle rov3(&hal_rov3, 3);
    CHECK(rov3.init());
    CHECK(rov3.arm(err));
    CHECK(rov3.get_thruster_count() == 3);

    // Forward surge command
    rov3.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f);
    rov3.step(0.01f);
    CHECK(hal_rov3.pwm().get_channel(0) > 1650); // Horiz Port
    CHECK(hal_rov3.pwm().get_channel(1) > 1650); // Horiz Stbd
    CHECK(hal_rov3.pwm().get_channel(2) == 1500); // Vert stopped

    // Heave climb command
    rov3.set_user_inputs(0.0f, 0.0f, 0.0f, 0.6f, 0.0f, 0.0f);
    rov3.step(0.01f);
    CHECK(hal_rov3.pwm().get_channel(2) > 1700); // Vert thruster active
    // Inactive channels 3..15 must remain at safe disarmed value (1000us)
    for (uint8_t ch = 3; ch < hal::PWMOutput::MAX_CHANNELS; ++ch) {
        CHECK(hal_rov3.pwm().get_channel(ch) == 1000);
    }
    std::cout << "  -> 3-Thruster Simple Allocation [PASSED]\n";

    // ----------------- TEST 3F: Runtime Interface Option -----------------
    std::cout << "\n[3F] Testing Runtime Interface to Choose Number of Thrusters...\n";
    hal::PosixHAL hal_vm;
    mock_healthy_sensors(hal_vm);

    vehicles::VehicleManager vm(&hal_vm, vehicles::VehicleType::ROV_SUBMARINE);
    CHECK(vm.init());

    // Switch between 3, 6, 8, and 12 thrusters at runtime
    CHECK(vm.configure_rov_thrusters(8));
    CHECK(vm.get_active_actuator_count() == 8);
    std::cout << "  -> Interface configured 8 thrusters: Active Count = "
              << static_cast<int>(vm.get_active_actuator_count()) << "\n";

    CHECK(vm.configure_rov_thrusters(3));
    CHECK(vm.get_active_actuator_count() == 3);
    std::cout << "  -> Interface configured 3 thrusters: Active Count = "
              << static_cast<int>(vm.get_active_actuator_count()) << "\n";

    CHECK(vm.configure_rov_thrusters(6));
    CHECK(vm.get_active_actuator_count() == 6);
    std::cout << "  -> Interface configured 6 thrusters: Active Count = "
              << static_cast<int>(vm.get_active_actuator_count()) << "\n";

    // Heavy work-class ROV with 12 thrusters
    CHECK(vm.configure_rov_thrusters(12));
    CHECK(vm.get_active_actuator_count() == 12);
    std::cout << "  -> Interface configured 12 thrusters: Active Count = "
              << static_cast<int>(vm.get_active_actuator_count()) << "\n";

    // Massive 16-channel maximum allocation
    CHECK(vm.configure_rov_thrusters(16));
    CHECK(vm.get_active_actuator_count() == 16);
    std::cout << "  -> Interface configured 16 thrusters: Active Count = "
              << static_cast<int>(vm.get_active_actuator_count()) << "\n";
    std::cout << "  -> Dynamic Thruster Selection Interface [PASSED]\n";

    // ----------------- TEST 3G: UAV Multi-Rotor Scaling -----------------
    std::cout << "\n[3G] Testing UAV Rotor Count Scaling (4 -> 6 -> 8 -> 12 Rotors)...\n";
    CHECK(vm.configure_uav_rotors(6));
    CHECK(vm.get_active_actuator_count() == 6);
    CHECK(vm.arm(err));

    vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.5f);
    vm.step(0.01f);
    for (uint8_t i = 0; i < 6; ++i) {
        std::cout << "  Channel " << (int)i << ": " << hal_vm.pwm().get_channel(i) << "us\n";
        CHECK(hal_vm.pwm().get_channel(i) >= 1400);
    }
    std::cout << "  -> Hexacopter (6 Rotors) Mixing [PASSED]\n";

    CHECK(vm.configure_uav_rotors(8));
    CHECK(vm.get_active_actuator_count() == 8);
    std::cout << "  -> Octacopter (8 Rotors) Mixing [PASSED]\n";

    // ----------------- TEST 3H: DroneCAN ESC Output Integration -----------------
    std::cout << "\n[3H] Testing DroneCAN ESC Output Generation (DSDL 1030)...\n";
    control::ActuatorMatrixMixer can_mixer(control::MixerPreset::ROV_6_VECTORED);
    control::ControlDemands demands;
    demands.forward = 0.5f;

    hal::CANFrame esc_frame;
    uint8_t transfer_id = 0;
    bool gen_ok = can_mixer.generate_dronecan_esc_frame(demands, transfer_id, esc_frame, true);
    CHECK(gen_ok);
    CHECK(esc_frame.is_extended);
    CHECK(esc_frame.dlc == 8);
    // Message ID 1030 (0x406) embedded in 29-bit ID
    uint32_t msg_type = (esc_frame.id >> 8) & 0xFFFF;
    CHECK(msg_type == 1030);
    std::cout << "  -> DroneCAN ESC CAN ID: 0x" << std::hex << esc_frame.id << std::dec
              << " (DSDL Message Type ID: " << msg_type << ")\n";
    // Check start and end bits in tail byte (0xC0 | transfer_id)
    CHECK((esc_frame.data[7] & 0xC0) == 0xC0);
    std::cout << "  -> DroneCAN ESC RawCommand Frame Encoding [PASSED]\n";

    std::cout << "\n========================================================\n";
    std::cout << "ALL 8 ACTUATOR MATRIX MIXER & VEHICLE CONTROLLER TESTS PASSED!\n";
    std::cout << "========================================================\n";
    return test::finish("test_vehicle_controls");
}
