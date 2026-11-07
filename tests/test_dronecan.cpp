/**
 * DroneCAN protocol tests. Golden values come from outside this codebase so the
 * tests can't just agree with our own mistakes:
 *  - scalar bit-order vectors: libcanard's test_scalar_encoding.cpp
 *  - data type signatures: ArduPilot's dsdlc-generated headers
 *  - message IDs: file names in github.com/DroneCAN/DSDL
 */
#include "hal/posix/posix_hal.hpp"
#include "drivers/can/dronecan_codec.hpp"
#include "drivers/can/dronecan_node.hpp"
#include "drivers/sensor_manager.hpp"
#include "control/actuator_matrix_mixer.hpp"
#include "tests/test_common.hpp"
#include <cstring>
#include <iostream>
#include <vector>

using namespace autopilot;
using namespace autopilot::drivers::dronecan;

static std::vector<hal::CANFrame> frames_for(uint16_t dtid, uint8_t node, uint8_t tid, const uint8_t* p, size_t len) {
    std::vector<hal::CANFrame> f(32);
    size_t n = make_broadcast_frames(20, dtid, node, tid, p, len, f.data(), f.size());
    f.resize(n);
    return f;
}

int main() {
    std::cout << "TEST: DroneCAN protocol\n";

    // 1. IDs and signatures
    CHECK(MSG_ID_RAW_IMU == 1003);
    CHECK(MSG_ID_STATIC_PRESSURE == 1028);
    CHECK(MSG_ID_STATIC_TEMPERATURE == 1029);
    CHECK(MSG_ID_GNSS_FIX2 == 1063);
    CHECK(SIG_ESC_RAW_COMMAND == 0x217F5C87D7EC951DULL);
    CHECK(SIG_RAW_IMU == 0x8280632C40E574B5ULL);
    CHECK(SIG_NODE_STATUS == 0x0F0868D0C1A7C6F1ULL);

    // 2. 29-bit CAN ID
    uint32_t can_id = CANIDFields::construct_message_id(24, MSG_ID_STATIC_PRESSURE, 42);
    auto f = CANIDFields::parse(can_id);
    CHECK(f.priority == 24 && f.data_type_id == MSG_ID_STATIC_PRESSURE && f.source_node_id == 42 && !f.is_service);

    // 3. Bit-level decoding, golden vectors from libcanard's test suite
    {
        const uint8_t buf[7] = {0b10100101, 0b11000011, 0b11100111, 0b01111110, 0b01010101, 0b10101010, 0b11101000};
        auto rd = [&](size_t off, uint8_t len, bool sgn) {
            BitReader r(buf, sizeof(buf));
            r.skip(off);
            if (sgn) { int64_t v = 0; r.read_i(len, v); return v; }
            uint64_t v = 0; r.read_u(len, v); return static_cast<int64_t>(v);
        };
        CHECK(rd(0, 8, false) == 0b10100101);
        CHECK(rd(4, 8, false) == 0b01011100);
        CHECK(rd(4, 4, false) == 0b00000101);
        CHECK(rd(9, 1, false) == 1);
        CHECK(rd(10, 1, false) == 0);
        CHECK(rd(24, 32, false) == 0b11101000101010100101010101111110LL);
        CHECK(rd(21, 32, false) == 0b01011101101101011100101011101111LL);
        CHECK(rd(21, 30, false) == 0b010111101101011100101011101111LL);
        CHECK(rd(16, 3, true) == -1);
        CHECK(rd(2, 3, true) == -4);
        CHECK(rd(0, 8, true) == -91);
        CHECK(rd(0, 16, true) == -15451);
        CHECK(rd(0, 15, true) == -7771);
        // encode must be the exact inverse
        uint8_t out[7] = {0};
        BitWriter w(out, sizeof(out));
        w.write_u(0b10100101, 8); w.write_u(0b1100001111100111LL, 16);
        CHECK(out[0] == 0b10100101);
        BitReader r(out, sizeof(out)); uint64_t v; r.skip(8); r.read_u(16, v);
        CHECK(v == 0b1100001111100111LL);
    }

    // 4. float16 round trip
    for (float v : {0.0f, 1.0f, -1.0f, 9.80665f, 0.5f, 25.4f, 0.00012f, -3000.0f}) {
        uint8_t b[2] = {0};
        BitWriter w(b, 2); w.write_f16(v);
        BitReader r(b, 2); float back; r.read_f16(back);
        CHECK(std::fabs(v - back) <= std::fabs(v) * 1e-3f + 1e-6f);
    }

    // 5. ESC RawCommand: 14-bit values in DroneCAN bit order
    {
        int16_t cmd[4] = {8191, 0, -8192, 1234};
        uint8_t p[8] = {0};
        size_t len = encode_esc_raw_command(cmd, 4, p, sizeof(p));
        CHECK(len == 7);
        // first value 8191 = 0x1FFF: low byte 0xFF streamed first, then the top 6 bits (011111)
        CHECK(p[0] == 0xFF);
        CHECK((p[1] & 0xFC) == 0x7C);
        int16_t back[20]; uint8_t n = 0;
        decode_esc_raw_command(p, len, back, n);
        CHECK(n == 4 && back[0] == 8191 && back[1] == 0 && back[2] == -8192 && back[3] == 1234);

        control::ActuatorMatrixMixer mixer(control::MixerPreset::UAV_QUAD_X);
        control::ControlDemands d; d.throttle = 0.5f;
        hal::CANFrame esc; uint8_t tid = 0;
        CHECK(mixer.generate_dronecan_esc_frame(d, tid, esc, true));
        CHECK(CANIDFields::parse(esc.id).data_type_id == 1030);
        CHECK(esc.id == 0x14040601u);
        decode_esc_raw_command(esc.data, esc.dlc - 1, back, n);
        CHECK(n == 4);
        CHECK(std::abs(back[0] - 4096) < 10);
    }

    // 6. Multi-frame RawIMU through the transport, CRC and toggles checked
    hal::PosixHAL hal;
    hal.time().set_manual_time_mode(true);
    {
        RawIMUMsg imu;
        imu.gyro[0] = 0.1f; imu.gyro[1] = -0.2f; imu.gyro[2] = 0.3f;
        imu.accel[0] = 0.5f; imu.accel[1] = -0.25f; imu.accel[2] = -9.80665f;
        uint8_t p[64];
        size_t len = encode_raw_imu(imu, p, sizeof(p));
        CHECK(len == 47);
        auto frames = frames_for(MSG_ID_RAW_IMU, 11, 5, p, len);
        CHECK(frames.size() == 7); // 5 + 6x7 = 47
        CHECK((frames[0].data[frames[0].dlc - 1] & 0xA0) == 0x80);  // SOT, toggle 0
        CHECK((frames.back().data[frames.back().dlc - 1] & 0x40) == 0x40); // EOT

        Reassembler rx;
        Transfer t; bool got = false;
        for (auto& fr : frames) got = rx.accept(fr, 0, t) || got;
        CHECK(got && t.len == 47 && t.source_node == 11);
        RawIMUMsg dec;
        CHECK(decode_raw_imu(t.payload, t.len, dec));
        CHECK_NEAR(dec.gyro[1], -0.2, 1e-3);
        CHECK_NEAR(dec.accel[2], -9.80665, 1e-2);

        // corrupt one byte -> CRC rejects the whole transfer
        auto bad = frames;
        bad[3].data[2] ^= 0x10;
        Reassembler rx2; got = false;
        for (auto& fr : bad) got = rx2.accept(fr, 0, t) || got;
        CHECK(!got && rx2.crc_errors() == 1);

        // dropped middle frame -> toggle mismatch, rejected
        auto dropped = frames;
        dropped.erase(dropped.begin() + 2);
        Reassembler rx3; got = false;
        for (auto& fr : dropped) got = rx3.accept(fr, 0, t) || got;
        CHECK(!got && rx3.sequence_errors() >= 1);

        // full node path
        DroneCANNode node(&hal.can(), hal.get_time(), 1);
        CHECK(node.init(1000000));
        for (auto& fr : frames) hal.can().inject_frame(fr);
        node.process_rx();
        auto can_imu = node.get_imu(11);
        CHECK(can_imu != nullptr);
        if (can_imu) {
            CHECK_NEAR(can_imu->get_gyro().z, 0.3, 1e-3);
            CHECK_NEAR(can_imu->get_accel().z, -9.80665, 1e-2);
        }
    }

    // 7. GNSS Fix2 round trip (multi-frame, 37-bit signed lat/lon)
    {
        GnssFix2Msg g;
        g.lat_deg = 26.5123456; g.lon_deg = -80.2329871; g.height_msl_m = 125.4f;
        g.ned_velocity[0] = 1.5f; g.ned_velocity[1] = -2.0f; g.ned_velocity[2] = 0.1f;
        g.sats_used = 14; g.status = 3;
        uint8_t p[64];
        size_t len = encode_gnss_fix2(g, p, sizeof(p));
        CHECK(len == 50);
        GnssFix2Msg d;
        CHECK(decode_gnss_fix2(p, len, d));
        CHECK_NEAR(d.lat_deg, 26.5123456, 1e-7);
        CHECK_NEAR(d.lon_deg, -80.2329871, 1e-7);
        CHECK_NEAR(d.height_msl_m, 125.4, 1e-3);
        CHECK(d.sats_used == 14 && d.status == 3);
        CHECK_NEAR(d.ned_velocity[1], -2.0, 1e-6);
    }

    // 8. Single-frame messages through the node + SensorManager, then unplug -> timeout
    {
        drivers::SensorManager sm(&hal);
        sm.auto_discover_sensors();

        auto send = [&](uint16_t dtid, const uint8_t* p, size_t len, uint8_t tid) {
            for (auto& fr : frames_for(dtid, 10, tid, p, len)) hal.can().inject_frame(fr);
        };
        NodeStatusMsg ns; ns.uptime_sec = 50;
        uint8_t p[16];
        send(MSG_ID_NODE_STATUS, p, encode_node_status(ns, p, sizeof(p)), 0);
        StaticPressureMsg sp; sp.pressure_pa = 101325.0f;
        send(MSG_ID_STATIC_PRESSURE, p, encode_static_pressure(sp, p, sizeof(p)), 1);
        MagFieldMsg mf; mf.field_ga[0] = 0.25f; mf.field_ga[2] = 0.40f;
        send(MSG_ID_MAGNETIC_FIELD, p, encode_mag_field(mf, false, p, sizeof(p)), 2);
        sm.update();

        auto* dn = sm.get_dronecan();
        CHECK(dn && dn->get_active_node_count() == 1);
        CHECK(sm.has_healthy_baro());
        CHECK(sm.get_active_baro() && std::fabs(sm.get_active_baro()->get_pressure_pa() - 101325.0f) < 0.1f);
        CHECK(sm.has_healthy_compass());
        CHECK(sm.get_active_compass() && std::fabs(sm.get_active_compass()->get_field_gauss().x - 0.25f) < 1e-3f);

        // repeated updates must not keep re-registering the same CAN sensors
        size_t n_before = sm.get_all_sensors().size();
        for (int i = 0; i < 100; ++i) { send(MSG_ID_STATIC_PRESSURE, p, encode_static_pressure(sp, p, sizeof(p)), 3); sm.update(); }
        CHECK(sm.get_all_sensors().size() == n_before);

        // unplug the CAN node: no more frames -> barometer times out
        hal.time().advance_time_ms(drivers::SensorManager::BARO_TIMEOUT_MS + 100);
        sm.update();
        CHECK(!sm.has_healthy_baro());
    }

    std::cout << ">>> DroneCAN tests done\n";
    return test::finish("test_dronecan");
}
