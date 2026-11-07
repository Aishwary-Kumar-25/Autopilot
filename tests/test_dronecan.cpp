#include "hal/posix/posix_hal.hpp"
#include "drivers/can/dronecan_types.hpp"
#include "drivers/can/dronecan_node.hpp"
#include "drivers/sensor_manager.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace autopilot;
using namespace autopilot::drivers::dronecan;

int main() {
    std::cout << "========================================================\n";
    std::cout << "TEST 5: DroneCAN Protocol & High-Reliability CAN Sensors\n";
    std::cout << "========================================================\n";

    // 1. Test 29-bit CAN ID construction and parsing
    std::cout << "[Step 1] Testing DroneCAN 29-bit Identifier bitfields...\n";
    uint32_t can_id = CANIDFields::construct_message_id(24, MSG_ID_STATIC_PRESSURE, 42);
    auto parsed_id = CANIDFields::parse(can_id);
    assert(parsed_id.priority == 24);
    assert(parsed_id.data_type_id == MSG_ID_STATIC_PRESSURE);
    assert(parsed_id.source_node_id == 42);
    assert(!parsed_id.is_service);
    std::cout << "  -> 29-bit CAN ID packing & parsing [PASSED]\n";

    // 2. Test IEEE 754 float16 conversion accuracy
    std::cout << "\n[Step 2] Testing IEEE 754 Float16 conversions...\n";
    float test_vals[] = {0.0f, 1.0f, -1.0f, 9.80665f, 0.5f, 25.4f};
    for (float v : test_vals) {
        uint16_t h = float32_to_float16(v);
        float recovered = float16_to_float32(h);
        assert(std::abs(v - recovered) < 0.05f);
    }
    std::cout << "  -> Float16 compression accuracy [PASSED]\n";

    // 3. Test DroneCAN Node reception and auto-discovery
    std::cout << "\n[Step 3] Initializing POSIX CAN device & DroneCAN Node...\n";
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);

    DroneCANNode dronecan_node(&hal.can(), hal.get_time(), 1);
    assert(dronecan_node.init(1000000));

    // Simulate Node 10 broadcasting NodeStatus (Uptime 50 sec, Health OK, Mode Operational)
    std::cout << "  -> Injecting NodeStatus broadcast from remote Node 10...\n";
    hal::CANFrame status_frame;
    status_frame.is_extended = true;
    status_frame.dlc = 8;
    status_frame.id = CANIDFields::construct_message_id(30, MSG_ID_NODE_STATUS, 10);
    uint32_t uptime = 50;
    std::memcpy(status_frame.data, &uptime, 4);
    status_frame.data[4] = 0x00; // Health OK (0), Mode Operational (0)
    status_frame.data[5] = 0x00;
    status_frame.data[6] = 0x00;
    status_frame.data[7] = TailByte::construct(true, true, false, 0); // SOT=1, EOT=1
    hal.can().inject_frame(status_frame);

    dronecan_node.process_rx();
    assert(dronecan_node.get_active_node_count() == 1);
    std::cout << "  -> Discovered active DroneCAN Node 10 on CAN bus [PASSED]\n";

    // 4. Inject StaticPressure CAN message (Data Type 1020) from Node 10
    std::cout << "\n[Step 4] Injecting DroneCAN StaticPressure message (101325 Pa)...\n";
    hal::CANFrame baro_frame;
    baro_frame.is_extended = true;
    baro_frame.dlc = 8;
    baro_frame.id = CANIDFields::construct_message_id(20, MSG_ID_STATIC_PRESSURE, 10);
    float pressure_val = 101325.0f;
    std::memcpy(baro_frame.data, &pressure_val, 4);
    uint16_t variance = float32_to_float16(0.1f);
    std::memcpy(baro_frame.data + 4, &variance, 2);
    baro_frame.data[6] = 0x00;
    baro_frame.data[7] = TailByte::construct(true, true, false, 1);
    hal.can().inject_frame(baro_frame);

    dronecan_node.process_rx();
    auto can_baro = dronecan_node.get_baro(10);
    assert(can_baro != nullptr);
    assert(can_baro->is_healthy());
    assert(std::abs(can_baro->get_pressure_pa() - 101325.0f) < 0.1f);
    std::cout << "  -> DroneCAN Barometer received: " << can_baro->get_pressure_pa() << " Pa [PASSED]\n";

    // 5. Inject MagneticField CAN message (Data Type 1001) from Node 10
    std::cout << "\n[Step 5] Injecting DroneCAN MagneticFieldStrength message...\n";
    hal::CANFrame mag_frame;
    mag_frame.is_extended = true;
    mag_frame.dlc = 8;
    mag_frame.id = CANIDFields::construct_message_id(20, MSG_ID_MAGNETIC_FIELD, 10);
    mag_frame.data[0] = 0; // Sensor ID 0
    uint16_t mx = float32_to_float16(0.25f);
    uint16_t my = float32_to_float16(0.00f);
    uint16_t mz = float32_to_float16(0.40f);
    std::memcpy(mag_frame.data + 1, &mx, 2);
    std::memcpy(mag_frame.data + 3, &my, 2);
    std::memcpy(mag_frame.data + 5, &mz, 2);
    mag_frame.data[7] = TailByte::construct(true, true, false, 2);
    hal.can().inject_frame(mag_frame);

    dronecan_node.process_rx();
    auto can_compass = dronecan_node.get_compass(10);
    assert(can_compass != nullptr);
    assert(can_compass->is_healthy());
    assert(std::abs(can_compass->get_field_gauss().x - 0.25f) < 0.05f);
    std::cout << "  -> DroneCAN Compass received field: X=" << can_compass->get_field_gauss().x << " Gauss [PASSED]\n";

    // 6. Test SensorManager Integration with DroneCAN
    std::cout << "\n[Step 6] Testing SensorManager auto-discovery of DroneCAN sensors...\n";
    drivers::SensorManager sm(&hal);
    sm.auto_discover_sensors();

    // Inject another pressure frame to trigger dynamic registration
    hal.can().inject_frame(baro_frame);
    sm.update();

    assert(sm.has_healthy_baro());
    assert(sm.get_active_baro() != nullptr);
    std::cout << "  -> SensorManager active barometer is: " << sm.get_active_baro()->name() << " [PASSED]\n";

    std::cout << "\n>>> ALL DRONECAN PROTOCOL & SENSOR TESTS PASSED SUCCESSFULLY! <<<\n";
    return 0;
}

