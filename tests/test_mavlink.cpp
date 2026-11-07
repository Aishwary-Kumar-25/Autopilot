/**
 * MAVLink v2 tests.
 *   1. Codec: every encoder matches pymavlink byte for byte (tests/mavlink_golden.hpp,
 *      regenerate with tools/gen_mavlink_golden.py), the parser decodes pymavlink
 *      frames (v2 and v1), resyncs on garbage and rejects corrupted frames.
 *   2. GcsLink against the real flight stack, over an in-memory "radio":
 *      GCS discovery, telemetry rates, parameter protocol, arm/disarm rules,
 *      modes, joystick input, vehicle profile switching.
 */
#include "comms/gcs_link.hpp"
#include "comms/mavlink/mavlink_messages.hpp"
#include "comms/vehicle_params.hpp"
#include "hal/posix/posix_hal.hpp"
#include "sim/quad_sim.hpp"
#include "tests/mavlink_golden.hpp"
#include "tests/test_common.hpp"
#include "vehicles/vehicle_manager.hpp"

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace autopilot;
using namespace autopilot::comms;
using namespace autopilot::comms::mavlink;

namespace {

template <typename Msg>
bool matches_golden(const char* name, const Msg& m, uint8_t seq, const uint8_t* gold, size_t gold_len) {
    uint8_t f[MAX_FRAME];
    const size_t n = pack(f, seq, 1, 1, m);
    const bool ok = n == gold_len && std::memcmp(f, gold, n) == 0;
    if (!ok) {
        std::printf("  %s mismatch (len %zu vs %zu)\n   ours:", name, n, gold_len);
        for (size_t i = 0; i < n; ++i) std::printf(" %02X", f[i]);
        std::printf("\n   gold:");
        for (size_t i = 0; i < gold_len; ++i) std::printf(" %02X", gold[i]);
        std::printf("\n");
    }
    return ok;
}
#define GOLDEN(msg, seq, arr) CHECK(matches_golden(#arr, msg, seq, golden::arr, sizeof(golden::arr)))

bool parse_one(const uint8_t* data, size_t len, Message& out, Parser* shared = nullptr) {
    Parser local;
    Parser& p = shared ? *shared : local;
    bool got = false;
    for (size_t i = 0; i < len; ++i) {
        if (p.parse(data[i])) {
            out = p.msg();
            got = true;
        }
    }
    return got;
}

void test_codec() {
    std::printf("-- codec vs pymavlink --\n");
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(crc_calculate(check, 9) == 0x6F91); // CRC-16/MCRF4XX check value

    Heartbeat hb;
    hb.type = MAV_TYPE_QUADROTOR; hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.base_mode = 1 | 64 | 16 | 128; hb.custom_mode = 2; hb.system_status = MAV_STATE_ACTIVE;
    GOLDEN(hb, 0, HEARTBEAT);

    SysStatus ss;
    ss.present = ss.enabled = 0x1020002B; ss.health = 0x10200023; ss.load = 125; ss.voltage_battery = 15800;
    ss.current_battery = 1234; ss.battery_remaining = 87; ss.drop_rate_comm = 50; ss.errors_comm = 7;
    GOLDEN(ss, 1, SYS_STATUS);

    Attitude at;
    at.time_boot_ms = 123456; at.roll = 0.1f; at.pitch = -0.2f; at.yaw = 3.0f;
    at.rollspeed = 0.01f; at.pitchspeed = -0.02f; at.yawspeed = 0.5f;
    GOLDEN(at, 2, ATTITUDE);

    GlobalPositionInt gp;
    gp.time_boot_ms = 5000; gp.lat = 265123000; gp.lon = 802329000; gp.alt = 128500; gp.relative_alt = 2500;
    gp.vx = 120; gp.vy = -35; gp.vz = -10; gp.hdg = 9000;
    GOLDEN(gp, 3, GLOBAL_POSITION_INT);

    GpsRawInt gr;
    gr.time_usec = 7000000; gr.fix_type = 3; gr.lat = 265123000; gr.lon = 802329000; gr.alt = 128500;
    gr.vel = 126; gr.cog = 34000; gr.satellites_visible = 12;
    GOLDEN(gr, 4, GPS_RAW_INT);

    VfrHud vh;
    vh.airspeed = vh.groundspeed = 1.25f; vh.heading = 271; vh.throttle = 48; vh.alt = 128.5f; vh.climb = -0.4f;
    GOLDEN(vh, 5, VFR_HUD);

    ServoOutputRaw so;
    so.time_usec = 4000; so.servo[0] = 1500; so.servo[1] = 1510; so.servo[2] = 1490; so.servo[3] = 1505;
    GOLDEN(so, 6, SERVO_OUTPUT_RAW);

    ParamValue pv;
    pv.param_value = 0.15f; pv.param_count = 22; pv.param_index = 5; std::strcpy(pv.param_id, "MC_ROLLRATE_P");
    GOLDEN(pv, 7, PARAM_VALUE_F);
    ParamValue pv16;  // a 16-char name fills the field with no terminator
    pv16.param_value = 2.5f; pv16.param_count = 22; pv16.param_index = 21;
    std::strcpy(pv16.param_id, "ABCDEFGHIJKLMNOP");
    GOLDEN(pv16, 8, PARAM_VALUE_16);

    CommandAck ack;
    ack.command = 400; ack.result = MAV_RESULT_FAILED; ack.target_system = 255; ack.target_component = 190;
    GOLDEN(ack, 9, COMMAND_ACK);

    StatusText st;
    st.severity = MAV_SEVERITY_CRITICAL; std::strcpy(st.text, "PreArm: no RC input");
    GOLDEN(st, 10, STATUSTEXT);

    AutopilotVersion av;
    av.capabilities = CAP_MAVLINK2 | CAP_PARAM_FLOAT | CAP_PARAM_ENCODE_BYTEWISE | CAP_COMMAND_INT | CAP_MISSION_INT;
    av.flight_sw_version = 0x01000000; av.uid = 0x45434C5542415001ull;
    GOLDEN(av, 11, AUTOPILOT_VERSION);

    MissionCount mc;
    mc.count = 0; mc.target_system = 255; mc.target_component = 190;
    GOLDEN(mc, 12, MISSION_COUNT);

    std::printf("-- decoder on pymavlink frames --\n");
    Message m;
    CHECK(parse_one(golden::RX_HEARTBEAT_GCS, sizeof(golden::RX_HEARTBEAT_GCS), m));
    CHECK(m.msgid == Heartbeat::ID && m.sysid == 255 && m.compid == 190 && m.seq == 40 && m.v2);
    CHECK(Heartbeat::decode(m.payload).type == MAV_TYPE_GCS);

    CHECK(parse_one(golden::RX_ARM, sizeof(golden::RX_ARM), m));
    const auto cl = CommandLong::decode(m.payload);
    CHECK(m.msgid == CommandLong::ID && cl.command == 400 && cl.param[0] == 1.0f && cl.target_system == 1);

    CHECK(parse_one(golden::RX_PARAM_SET, sizeof(golden::RX_PARAM_SET), m));
    const auto ps = ParamSet::decode(m.payload);
    CHECK(std::strcmp(ps.param_id, "MC_ROLLRATE_P") == 0);
    CHECK_NEAR(ps.param_value, 0.2, 1e-7);
    CHECK(ps.param_type == MAV_PARAM_TYPE_REAL32);

    CHECK(parse_one(golden::RX_MANUAL_CONTROL, sizeof(golden::RX_MANUAL_CONTROL), m));
    const auto man = ManualControl::decode(m.payload);
    CHECK(man.target == 1 && man.x == 250 && man.y == -500 && man.z == 700 && man.r == 100);

    CHECK(parse_one(golden::RX_PARAM_REQUEST_READ, sizeof(golden::RX_PARAM_REQUEST_READ), m));
    const auto prr = ParamRequestRead::decode(m.payload);
    CHECK(prr.param_index == -1 && std::strcmp(prr.param_id, "FS_RC_TIMEOUT") == 0);

    CHECK(parse_one(golden::RX_V1_HEARTBEAT, sizeof(golden::RX_V1_HEARTBEAT), m));
    CHECK(!m.v2 && m.msgid == Heartbeat::ID && m.seq == 45 && Heartbeat::decode(m.payload).type == MAV_TYPE_GCS);

    // Our own frames round-trip through our parser (also checks truncated payloads:
    // ATTITUDE with yawspeed 0 loses its trailing zero bytes on the wire).
    Attitude a0;
    a0.time_boot_ms = 77; a0.roll = 0.5f;
    uint8_t f[MAX_FRAME];
    size_t n = pack(f, 3, 1, 1, a0);
    CHECK(n < 12 + 28);
    CHECK(parse_one(f, n, m) && m.msgid == Attitude::ID);
    const auto a1 = Attitude::decode(m.payload);
    CHECK(a1.time_boot_ms == 77 && a1.roll == 0.5f && a1.pitch == 0.0f && a1.yawspeed == 0.0f);

    // Skip line noise, reject a corrupted frame, then accept the next good one.
    Parser p;
    std::vector<uint8_t> stream = {0x00, 0x13, 0x55, 0x42, 0x99};
    std::vector<uint8_t> bad(golden::HEARTBEAT, golden::HEARTBEAT + sizeof(golden::HEARTBEAT));
    bad[12] ^= 0x40; // flip a payload bit
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), golden::ATTITUDE, golden::ATTITUDE + sizeof(golden::ATTITUDE));
    int got = 0;
    uint32_t last_id = 999;
    for (uint8_t b : stream) {
        if (p.parse(b)) { ++got; last_id = p.msg().msgid; }
    }
    CHECK(got == 1 && last_id == Attitude::ID);
    CHECK(p.crc_errors() >= 1);
}

// ------------------------------------------------------------ GcsLink tests

/** In-memory telemetry radio: the test plays the GCS on the other end. */
class FakeRadio : public hal::UARTDevice {
public:
    std::vector<uint8_t> to_vehicle, from_vehicle;
    bool init(uint32_t) override { return true; }
    size_t write(const uint8_t* d, size_t n) override { from_vehicle.insert(from_vehicle.end(), d, d + n); return n; }
    size_t read(uint8_t* buf, size_t max) override {
        const size_t n = std::min(max, to_vehicle.size());
        std::memcpy(buf, to_vehicle.data(), n);
        to_vehicle.erase(to_vehicle.begin(), to_vehicle.begin() + static_cast<long>(n));
        return n;
    }
    size_t available() override { return to_vehicle.size(); }
};

struct Gcs {
    FakeRadio radio;
    Parser parser;
    uint8_t seq{0};
    std::vector<Message> inbox;

    template <typename Msg> void send(const Msg& m) {
        uint8_t f[MAX_FRAME];
        const size_t n = pack(f, seq++, 255, 190, m);
        radio.to_vehicle.insert(radio.to_vehicle.end(), f, f + n);
    }
    void drain() {
        for (uint8_t b : radio.from_vehicle) if (parser.parse(b)) inbox.push_back(parser.msg());
        radio.from_vehicle.clear();
    }
    size_t count(uint32_t id) const {
        size_t c = 0;
        for (const auto& m : inbox) c += (m.msgid == id);
        return c;
    }
    const Message* last(uint32_t id) const {
        for (auto it = inbox.rbegin(); it != inbox.rend(); ++it) if (it->msgid == id) return &*it;
        return nullptr;
    }
    bool has_text(const char* needle) const {
        for (const auto& m : inbox) {
            if (m.msgid == StatusText::ID && std::strstr(StatusText::decode(m.payload).text, needle)) return true;
        }
        return false;
    }
    void command(uint16_t cmd, float p1 = 0, float p2 = 0) {
        CommandLong c;
        c.command = cmd; c.target_system = 1; c.target_component = 1; c.param[0] = p1; c.param[1] = p2;
        send(c);
    }
    int ack_result(uint16_t cmd) const {
        for (auto it = inbox.rbegin(); it != inbox.rend(); ++it) {
            if (it->msgid == CommandAck::ID) {
                const auto a = CommandAck::decode(it->payload);
                if (a.command == cmd) return a.result;
            }
        }
        return -1;
    }
    void param_set(const char* name, float v, uint8_t type = MAV_PARAM_TYPE_REAL32) {
        ParamSet s;
        s.target_system = 1; s.target_component = 1; std::strcpy(s.param_id, name); s.param_type = type;
        if (type == MAV_PARAM_TYPE_INT32) { const int32_t i = static_cast<int32_t>(v); std::memcpy(&s.param_value, &i, 4); }
        else s.param_value = v;
        send(s);
    }
    bool param_reply(const char* name, ParamValue& out) const {
        for (auto it = inbox.rbegin(); it != inbox.rend(); ++it) {
            if (it->msgid != ParamValue::ID) continue;
            const auto pv = ParamValue::decode(it->payload);
            if (std::strcmp(pv.param_id, name) == 0) { out = pv; return true; }
        }
        return false;
    }
    void manual(int16_t x, int16_t y, int16_t z, int16_t r) {
        ManualControl mc; mc.target = 1; mc.x = x; mc.y = y; mc.z = z; mc.r = r; send(mc);
    }
};

float as_int(const ParamValue& pv) { int32_t i; std::memcpy(&i, &pv.param_value, 4); return static_cast<float>(i); }

struct Rig {
    hal::PosixHAL hal;
    sim::QuadModel quad;   // sits on the ground: gives the sensors realistic data
    sim::SimContext ctx;
    vehicles::VehicleManager vm{&hal, vehicles::VehicleType::UAV_MULTICOPTER};
    ParamStore params;
    Gcs gcs;
    std::unique_ptr<GcsLink> link;

    Rig() {
        hal.time().set_manual_time_mode(true);
        hal.time().advance_time_ms(1000);
        ctx.model = &quad;
        ctx.time = &hal.time();
        vm.init();
        vm.get_sensors()->register_sensor(std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU0"));
        vm.get_sensors()->register_sensor(std::make_shared<sim::SimBaro>(&ctx));
        vm.get_sensors()->register_sensor(std::make_shared<sim::SimCompass>(&ctx));
        register_vehicle_params(params, vm);
        link = std::make_unique<GcsLink>(&gcs.radio, &vm, &params);
    }
    void run_ms(uint32_t ms) {
        for (uint32_t t = 0; t < ms; t += 4) {
            vm.step(0.004f);
            link->update();
            hal.time().advance_time_ms(4);
        }
        gcs.drain();
    }
    vehicles::UAVVehicle* uav() { return dynamic_cast<vehicles::UAVVehicle*>(vm.get_active_vehicle()); }
};

void test_link() {
    std::printf("-- GcsLink with the flight stack --\n");
    Rig r;
    r.run_ms(200);
    CHECK(r.gcs.count(Heartbeat::ID) >= 1);
    CHECK(!r.link->gcs_connected());

    // GCS appears
    Heartbeat ghb; ghb.type = MAV_TYPE_GCS; ghb.autopilot = 8;
    r.gcs.send(ghb);
    r.gcs.inbox.clear();
    r.run_ms(1000);
    CHECK(r.link->gcs_connected());
    CHECK(r.gcs.has_text("GCS connected"));
    CHECK(r.gcs.count(AutopilotVersion::ID) == 1);
    // telemetry rates over 1 s
    const size_t hb = r.gcs.count(Heartbeat::ID), att = r.gcs.count(Attitude::ID), ss = r.gcs.count(SysStatus::ID);
    std::printf("  1 s of telemetry: HEARTBEAT %zu, ATTITUDE %zu, SYS_STATUS %zu, VFR_HUD %zu, SERVO %zu\n", hb, att, ss,
                r.gcs.count(VfrHud::ID), r.gcs.count(ServoOutputRaw::ID));
    CHECK(hb >= 1 && hb <= 2);
    CHECK(att >= 9 && att <= 11);
    CHECK(ss >= 1 && ss <= 2);
    CHECK(r.gcs.count(GlobalPositionInt::ID) == 0); // GPS without a fix: never claim a position
    const auto hb0 = Heartbeat::decode(r.gcs.last(Heartbeat::ID)->payload);
    CHECK(hb0.type == MAV_TYPE_QUADROTOR && hb0.autopilot == MAV_AUTOPILOT_GENERIC);
    CHECK(!(hb0.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) && hb0.system_status == MAV_STATE_STANDBY);
    const auto st = [&] { SysStatus s; Reader rd(r.gcs.last(SysStatus::ID)->payload);
                          s.present = rd.u32(); s.enabled = rd.u32(); s.health = rd.u32(); return s; }();
    CHECK((st.present & SENSOR_3D_GYRO) && (st.health & SENSOR_3D_GYRO));
    CHECK((st.present & SENSOR_ABSOLUTE_PRESSURE) && (st.health & SENSOR_ABSOLUTE_PRESSURE));
    CHECK(st.present & SENSOR_GPS);                 // the auto-discovered NMEA GPS on the UART


    // ---- parameter protocol ----
    r.gcs.inbox.clear();
    ParamRequestList prl; prl.target_system = 1; prl.target_component = 1;
    r.gcs.send(prl);
    r.run_ms(100);
    std::map<uint16_t, std::string> seen;
    uint16_t reported_count = 0;
    for (const auto& m : r.gcs.inbox) {
        if (m.msgid != ParamValue::ID) continue;
        const auto pv = ParamValue::decode(m.payload);
        seen[pv.param_index] = pv.param_id;
        reported_count = pv.param_count;
    }
    std::printf("  param list: %zu params streamed (store has %zu)\n", seen.size(), r.params.count());
    CHECK(seen.size() == r.params.count() && reported_count == r.params.count());
    CHECK(r.params.count() >= 20);

    r.gcs.param_set("MC_ROLLRATE_P", 0.2f);
    r.run_ms(20);
    ParamValue pv;
    CHECK(r.gcs.param_reply("MC_ROLLRATE_P", pv));
    CHECK_NEAR(pv.param_value, 0.2, 1e-6);
    CHECK_NEAR(r.uav()->pid(vehicles::UAVPid::ROLL_RATE).config().kp, 0.2, 1e-6);

    r.gcs.param_set("FS_RC_TIMEOUT", 800, MAV_PARAM_TYPE_INT32);   // INT32: byte-wise encoding
    r.run_ms(20);
    CHECK(r.gcs.param_reply("FS_RC_TIMEOUT", pv) && pv.param_type == MAV_PARAM_TYPE_INT32 && as_int(pv) == 800.0f);
    CHECK(r.vm.get_active_vehicle()->get_failsafe().config().rc_timeout_ms == 800);

    r.gcs.inbox.clear();
    r.gcs.param_set("MC_ROLLRATE_P", 99.0f);                         // out of range
    r.run_ms(20);
    CHECK(r.gcs.param_reply("MC_ROLLRATE_P", pv));
    CHECK_NEAR(pv.param_value, 0.2, 1e-6);                           // unchanged, GCS sees the refusal
    CHECK(r.gcs.has_text("out of range"));

    ParamRequestRead prr; prr.target_system = 1; prr.target_component = 1; prr.param_index = -1;
    std::strcpy(prr.param_id, "MC_HOVER_THR");
    r.gcs.send(prr);
    r.run_ms(20);
    CHECK(r.gcs.param_reply("MC_HOVER_THR", pv));
    CHECK_NEAR(pv.param_value, r.uav()->get_hover_throttle(), 1e-6);

    // ---- arming ----
    r.gcs.inbox.clear();
    r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);                // no RC yet
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_FAILED);
    CHECK(r.gcs.has_text("RC"));
    CHECK(!r.vm.is_armed());

    r.gcs.inbox.clear();
    for (int i = 0; i < 10; ++i) { r.gcs.manual(0, 0, 0, 0); r.run_ms(20); } // joystick, throttle down
    r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    for (int i = 0; i < 60; ++i) { r.gcs.manual(0, 0, 0, 0); r.run_ms(20); }
    CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED);
    CHECK(r.vm.is_armed());
    CHECK(r.gcs.has_text("Armed"));
    const auto hb1 = Heartbeat::decode(r.gcs.last(Heartbeat::ID)->payload);
    CHECK((hb1.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) && hb1.system_status == MAV_STATE_ACTIVE);
    r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 0);                 // on the ground: plain disarm is fine
    r.gcs.manual(0, 0, 0, 0);
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED);
    CHECK(!r.vm.is_armed());
    CHECK(r.gcs.has_text("Disarmed"));

    // "Flying": arm in STABILIZE and push the throttle up (the land detector clears).
    auto take_off = [&] {
        CHECK(set_vehicle_mode(r.vm, 1));
        for (int i = 0; i < 5; ++i) { r.gcs.manual(0, 0, 0, 0); r.run_ms(20); }
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
        for (int i = 0; i < 10; ++i) { r.gcs.manual(0, 0, 800, 0); r.run_ms(20); }
        CHECK(r.vm.is_armed());
        CHECK(r.uav() && !r.uav()->is_landed());
    };
    r.gcs.inbox.clear();
    take_off();

    // Joystick mapping: forward stick = nose down = negative pitch demand
    r.gcs.manual(500, 0, 800, 0);
    r.run_ms(8);
    CHECK(r.vm.get_active_vehicle()->get_demands().pitch < 0.0f);

    r.gcs.param_set("VEHICLE_TYPE", 1, MAV_PARAM_TYPE_INT32);       // refused while armed
    r.gcs.manual(0, 0, 800, 0);
    r.run_ms(20);
    CHECK(r.vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER);
    CHECK(r.gcs.param_reply("VEHICLE_TYPE", pv) && as_int(pv) == 0.0f);

    r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 0);                 // not landed, not forced
    r.gcs.manual(0, 0, 800, 0);
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_DENIED);
    CHECK(r.vm.is_armed());

    // Joystick goes quiet -> RC-loss failsafe (FS_RC_TIMEOUT = 800 ms) is reported
    r.run_ms(1000);
    CHECK(r.gcs.has_text("Failsafe LAND: RC"));
    CHECK(Heartbeat::decode(r.gcs.last(Heartbeat::ID)->payload).system_status == MAV_STATE_CRITICAL);
    r.run_ms(5000);                                                  // failsafe land finishes on the ground
    CHECK(!r.vm.is_armed());

    r.gcs.inbox.clear();
    take_off();
    r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 0, ARM_DISARM_FORCE_MAGIC); // forced: allowed
    r.gcs.manual(0, 0, 800, 0);
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED);
    CHECK(!r.vm.is_armed());

    // ---- modes ----
    r.gcs.inbox.clear();
    r.gcs.command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2);
    r.run_ms(1000);
    CHECK(r.gcs.ack_result(MAV_CMD_DO_SET_MODE) == MAV_RESULT_ACCEPTED);
    CHECK(r.uav()->get_mode() == vehicles::UAVFlightMode::ALT_HOLD);
    CHECK(Heartbeat::decode(r.gcs.last(Heartbeat::ID)->payload).custom_mode == 2);
    CHECK(r.gcs.has_text("Mode ALT_HOLD"));
    SetMode sm; sm.target_system = 1; sm.base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;   // base-mode-only GCS
    r.gcs.send(sm);
    r.run_ms(20);
    CHECK(r.uav()->get_mode() == vehicles::UAVFlightMode::STABILIZE);
    r.gcs.command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 7);
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_DO_SET_MODE) == MAV_RESULT_DENIED);

    // ---- vehicle profile switch from the GCS; tuning survives it ----
    r.gcs.inbox.clear();
    r.gcs.param_set("VEHICLE_TYPE", 1, MAV_PARAM_TYPE_INT32);
    r.run_ms(1000);
    CHECK(r.vm.get_active_type() == vehicles::VehicleType::UGV_GROUND_ROVER);
    CHECK(Heartbeat::decode(r.gcs.last(Heartbeat::ID)->payload).type == MAV_TYPE_GROUND_ROVER);
    CHECK(r.gcs.has_text("Vehicle: UGV"));
    CHECK(r.vm.get_active_vehicle()->get_failsafe().config().rc_timeout_ms == 800);
    r.gcs.param_set("VEHICLE_TYPE", 0, MAV_PARAM_TYPE_INT32);
    r.run_ms(20);
    CHECK(r.uav() != nullptr);
    CHECK_NEAR(r.uav()->pid(vehicles::UAVPid::ROLL_RATE).config().kp, 0.2, 1e-6);
    r.gcs.param_set("UAV_ROTORS", 6, MAV_PARAM_TYPE_INT32);
    r.run_ms(1000);
    CHECK(r.uav()->get_rotor_count() == 6);
    CHECK(Heartbeat::decode(r.gcs.last(Heartbeat::ID)->payload).type == MAV_TYPE_HEXAROTOR);

    // ---- misc protocol ----
    r.gcs.inbox.clear();
    MissionRequestList mrl; mrl.target_system = 1; mrl.target_component = 1;
    r.gcs.send(mrl);
    r.gcs.command(MAV_CMD_SET_MESSAGE_INTERVAL, static_cast<float>(Attitude::ID), 50000.0f); // 20 Hz
    r.gcs.command(31000);                                            // something we don't do
    r.run_ms(1000);
    CHECK(r.gcs.count(MissionCount::ID) == 1 && MissionCount::decode(r.gcs.last(MissionCount::ID)->payload).count == 0);
    CHECK(r.gcs.ack_result(MAV_CMD_SET_MESSAGE_INTERVAL) == MAV_RESULT_ACCEPTED);
    CHECK(r.gcs.count(Attitude::ID) >= 19 && r.gcs.count(Attitude::ID) <= 21);
    CHECK(r.gcs.ack_result(31000) == MAV_RESULT_UNSUPPORTED);
    r.gcs.command(MAV_CMD_REQUEST_MESSAGE, static_cast<float>(AutopilotVersion::ID));
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_REQUEST_MESSAGE) == MAV_RESULT_ACCEPTED && r.gcs.count(AutopilotVersion::ID) == 1);

    // Commands addressed to another vehicle are ignored (no ACK)
    r.gcs.inbox.clear();
    CommandLong other; other.command = MAV_CMD_COMPONENT_ARM_DISARM; other.target_system = 2; other.param[0] = 1;
    r.gcs.send(other);
    r.run_ms(20);
    CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == -1);

    // GCS silence -> link reports it lost
    r.run_ms(3500);
    CHECK(!r.link->gcs_connected());
    CHECK(r.link->stats().rx_crc_errors == 0);
}

} // namespace

int main() {
    std::printf("== MAVLink ==\n");
    test_codec();
    test_link();
    return test::finish("test_mavlink");
}
