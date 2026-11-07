#pragma once
/**
 * The MAVLink common.xml messages this autopilot speaks, hand-written from the
 * spec. Fields are serialised in WIRE order (base fields sorted by type size,
 * largest first; extension fields appended in declaration order), which is not
 * the order the XML lists them in.
 *
 * Each struct has:
 *   ID, CRC_EXTRA      message id and layout hash (must match common.xml)
 *   encode(buf)        writes the full payload, returns its length (v2 trims zeros)
 *   decode(payload)    reads from a zero-filled MAX_PAYLOAD buffer (Message::payload)
 *
 * tests/test_mavlink.cpp checks every encoder byte for byte against frames
 * produced by pymavlink.
 */
#include "comms/mavlink/mavlink_protocol.hpp"

namespace autopilot {
namespace comms {
namespace mavlink {

// ---- enums used by this stack (subset of common.xml / minimal.xml) ----
enum MavType : uint8_t {
    MAV_TYPE_GENERIC = 0, MAV_TYPE_QUADROTOR = 2, MAV_TYPE_GCS = 6, MAV_TYPE_GROUND_ROVER = 10,
    MAV_TYPE_SUBMARINE = 12, MAV_TYPE_HEXAROTOR = 13, MAV_TYPE_OCTOROTOR = 14, MAV_TYPE_TRICOPTER = 15,
};
enum MavAutopilot : uint8_t { MAV_AUTOPILOT_GENERIC = 0 };
enum MavModeFlag : uint8_t {
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
    MAV_MODE_FLAG_SAFETY_ARMED = 128,
};
enum MavState : uint8_t {
    MAV_STATE_UNINIT = 0, MAV_STATE_BOOT = 1, MAV_STATE_CALIBRATING = 2, MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4, MAV_STATE_CRITICAL = 5, MAV_STATE_EMERGENCY = 6,
};
enum MavResult : uint8_t {
    MAV_RESULT_ACCEPTED = 0, MAV_RESULT_TEMPORARILY_REJECTED = 1, MAV_RESULT_DENIED = 2,
    MAV_RESULT_UNSUPPORTED = 3, MAV_RESULT_FAILED = 4,
};
enum MavSeverity : uint8_t {
    MAV_SEVERITY_EMERGENCY = 0, MAV_SEVERITY_ALERT = 1, MAV_SEVERITY_CRITICAL = 2, MAV_SEVERITY_ERROR = 3,
    MAV_SEVERITY_WARNING = 4, MAV_SEVERITY_NOTICE = 5, MAV_SEVERITY_INFO = 6, MAV_SEVERITY_DEBUG = 7,
};
enum MavParamType : uint8_t { MAV_PARAM_TYPE_INT32 = 6, MAV_PARAM_TYPE_REAL32 = 9 };
enum MavCmd : uint16_t {
    MAV_CMD_DO_SET_MODE = 176,
    MAV_CMD_PREFLIGHT_STORAGE = 245,
    MAV_CMD_COMPONENT_ARM_DISARM = 400,
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,
    MAV_CMD_REQUEST_MESSAGE = 512,
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
};
enum MavSysStatusSensor : uint32_t {
    SENSOR_3D_GYRO = 1u << 0, SENSOR_3D_ACCEL = 1u << 1, SENSOR_3D_MAG = 1u << 2,
    SENSOR_ABSOLUTE_PRESSURE = 1u << 3, SENSOR_GPS = 1u << 5, SENSOR_MOTOR_OUTPUTS = 1u << 15,
    SENSOR_RC_RECEIVER = 1u << 16, SENSOR_AHRS = 1u << 21, SENSOR_BATTERY = 1u << 25,
    SENSOR_PREARM_CHECK = 1u << 28,
};
enum MavProtocolCapability : uint64_t {
    CAP_MISSION_INT = 1ull << 2, CAP_PARAM_FLOAT = 1ull << 1, CAP_COMMAND_INT = 1ull << 3,
    CAP_PARAM_ENCODE_BYTEWISE = 1ull << 4, CAP_MAVLINK2 = 1ull << 13,
};
constexpr uint8_t MAV_COMP_ID_AUTOPILOT1 = 1;
constexpr uint16_t ARM_DISARM_FORCE_MAGIC = 21196;

struct Heartbeat {
    static constexpr uint32_t ID = 0;
    static constexpr uint8_t CRC_EXTRA = 50;
    uint32_t custom_mode{0};
    uint8_t type{0}, autopilot{0}, base_mode{0}, system_status{0}, mavlink_version{3};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u32(custom_mode); w.u8(type); w.u8(autopilot); w.u8(base_mode); w.u8(system_status);
        w.u8(mavlink_version);
        return w.size();
    }
    static Heartbeat decode(const uint8_t* p) {
        Reader r(p); Heartbeat m;
        m.custom_mode = r.u32(); m.type = r.u8(); m.autopilot = r.u8(); m.base_mode = r.u8();
        m.system_status = r.u8(); m.mavlink_version = r.u8();
        return m;
    }
};

struct SysStatus {
    static constexpr uint32_t ID = 1;
    static constexpr uint8_t CRC_EXTRA = 124;
    uint32_t present{0}, enabled{0}, health{0};
    uint16_t load{0};                 // 0.1 %
    uint16_t voltage_battery{UINT16_MAX}; // mV, UINT16_MAX = unknown
    int16_t current_battery{-1};      // 10 mA, -1 = unknown
    uint16_t drop_rate_comm{0};       // 0.01 %
    uint16_t errors_comm{0};
    uint16_t errors_count[4]{};
    int8_t battery_remaining{-1};     // %, -1 = unknown
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u32(present); w.u32(enabled); w.u32(health); w.u16(load); w.u16(voltage_battery);
        w.i16(current_battery); w.u16(drop_rate_comm); w.u16(errors_comm);
        for (uint16_t e : errors_count) w.u16(e);
        w.i8(battery_remaining);
        w.u32(0); w.u32(0); w.u32(0); // *_extended (ext)
        return w.size();
    }
};

struct Ping {
    static constexpr uint32_t ID = 4;
    static constexpr uint8_t CRC_EXTRA = 237;
    uint64_t time_usec{0};
    uint32_t seq{0};
    uint8_t target_system{0}, target_component{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u64(time_usec); w.u32(seq); w.u8(target_system); w.u8(target_component);
        return w.size();
    }
    static Ping decode(const uint8_t* p) {
        Reader r(p); Ping m;
        m.time_usec = r.u64(); m.seq = r.u32(); m.target_system = r.u8(); m.target_component = r.u8();
        return m;
    }
};

struct SetMode {
    static constexpr uint32_t ID = 11;
    static constexpr uint8_t CRC_EXTRA = 89;
    uint32_t custom_mode{0};
    uint8_t target_system{0}, base_mode{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u32(custom_mode); w.u8(target_system); w.u8(base_mode);
        return w.size();
    }
    static SetMode decode(const uint8_t* p) {
        Reader r(p); SetMode m;
        m.custom_mode = r.u32(); m.target_system = r.u8(); m.base_mode = r.u8();
        return m;
    }
};

struct ParamRequestRead {
    static constexpr uint32_t ID = 20;
    static constexpr uint8_t CRC_EXTRA = 214;
    int16_t param_index{-1};
    uint8_t target_system{0}, target_component{0};
    char param_id[17]{};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.i16(param_index); w.u8(target_system); w.u8(target_component); w.chars(param_id, 16);
        return w.size();
    }
    static ParamRequestRead decode(const uint8_t* p) {
        Reader r(p); ParamRequestRead m;
        m.param_index = r.i16(); m.target_system = r.u8(); m.target_component = r.u8();
        r.chars(m.param_id, 16);
        return m;
    }
};

struct ParamRequestList {
    static constexpr uint32_t ID = 21;
    static constexpr uint8_t CRC_EXTRA = 159;
    uint8_t target_system{0}, target_component{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u8(target_system); w.u8(target_component);
        return w.size();
    }
    static ParamRequestList decode(const uint8_t* p) {
        Reader r(p); ParamRequestList m;
        m.target_system = r.u8(); m.target_component = r.u8();
        return m;
    }
};

struct ParamValue {
    static constexpr uint32_t ID = 22;
    static constexpr uint8_t CRC_EXTRA = 220;
    float param_value{0};
    uint16_t param_count{0}, param_index{0};
    char param_id[17]{};
    uint8_t param_type{MAV_PARAM_TYPE_REAL32};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.f32(param_value); w.u16(param_count); w.u16(param_index); w.chars(param_id, 16);
        w.u8(param_type);
        return w.size();
    }
    static ParamValue decode(const uint8_t* p) {
        Reader r(p); ParamValue m;
        m.param_value = r.f32(); m.param_count = r.u16(); m.param_index = r.u16();
        r.chars(m.param_id, 16); m.param_type = r.u8();
        return m;
    }
};

struct ParamSet {
    static constexpr uint32_t ID = 23;
    static constexpr uint8_t CRC_EXTRA = 168;
    float param_value{0};
    uint8_t target_system{0}, target_component{0};
    char param_id[17]{};
    uint8_t param_type{MAV_PARAM_TYPE_REAL32};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.f32(param_value); w.u8(target_system); w.u8(target_component); w.chars(param_id, 16);
        w.u8(param_type);
        return w.size();
    }
    static ParamSet decode(const uint8_t* p) {
        Reader r(p); ParamSet m;
        m.param_value = r.f32(); m.target_system = r.u8(); m.target_component = r.u8();
        r.chars(m.param_id, 16); m.param_type = r.u8();
        return m;
    }
};

struct GpsRawInt {
    static constexpr uint32_t ID = 24;
    static constexpr uint8_t CRC_EXTRA = 24;
    uint64_t time_usec{0};
    int32_t lat{0}, lon{0}, alt{0};     // degE7, degE7, mm MSL
    uint16_t eph{UINT16_MAX}, epv{UINT16_MAX};
    uint16_t vel{UINT16_MAX};           // cm/s
    uint16_t cog{UINT16_MAX};           // cdeg
    uint8_t fix_type{0}, satellites_visible{UINT8_MAX};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u64(time_usec); w.i32(lat); w.i32(lon); w.i32(alt); w.u16(eph); w.u16(epv); w.u16(vel);
        w.u16(cog); w.u8(fix_type); w.u8(satellites_visible);
        // extensions: alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw (all unknown = 0)
        w.i32(0); w.u32(0); w.u32(0); w.u32(0); w.u32(0); w.u16(0);
        return w.size();
    }
};

struct Attitude {
    static constexpr uint32_t ID = 30;
    static constexpr uint8_t CRC_EXTRA = 39;
    uint32_t time_boot_ms{0};
    float roll{0}, pitch{0}, yaw{0}, rollspeed{0}, pitchspeed{0}, yawspeed{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u32(time_boot_ms); w.f32(roll); w.f32(pitch); w.f32(yaw); w.f32(rollspeed); w.f32(pitchspeed);
        w.f32(yawspeed);
        return w.size();
    }
    static Attitude decode(const uint8_t* p) {
        Reader r(p); Attitude m;
        m.time_boot_ms = r.u32(); m.roll = r.f32(); m.pitch = r.f32(); m.yaw = r.f32();
        m.rollspeed = r.f32(); m.pitchspeed = r.f32(); m.yawspeed = r.f32();
        return m;
    }
};

struct GlobalPositionInt {
    static constexpr uint32_t ID = 33;
    static constexpr uint8_t CRC_EXTRA = 104;
    uint32_t time_boot_ms{0};
    int32_t lat{0}, lon{0}, alt{0}, relative_alt{0}; // degE7, degE7, mm MSL, mm above home
    int16_t vx{0}, vy{0}, vz{0};                       // cm/s NED
    uint16_t hdg{UINT16_MAX};                          // cdeg
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u32(time_boot_ms); w.i32(lat); w.i32(lon); w.i32(alt); w.i32(relative_alt); w.i16(vx);
        w.i16(vy); w.i16(vz); w.u16(hdg);
        return w.size();
    }
};

struct ServoOutputRaw {
    static constexpr uint32_t ID = 36;
    static constexpr uint8_t CRC_EXTRA = 222;
    uint32_t time_usec{0};
    uint16_t servo[16]{};
    uint8_t port{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u32(time_usec);
        for (int i = 0; i < 8; ++i) w.u16(servo[i]);
        w.u8(port);
        for (int i = 8; i < 16; ++i) w.u16(servo[i]); // extensions
        return w.size();
    }
};

struct MissionRequestList {
    static constexpr uint32_t ID = 43;
    static constexpr uint8_t CRC_EXTRA = 132;
    uint8_t target_system{0}, target_component{0}, mission_type{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u8(target_system); w.u8(target_component); w.u8(mission_type);
        return w.size();
    }
    static MissionRequestList decode(const uint8_t* p) {
        Reader r(p); MissionRequestList m;
        m.target_system = r.u8(); m.target_component = r.u8(); m.mission_type = r.u8();
        return m;
    }
};

struct MissionCount {
    static constexpr uint32_t ID = 44;
    static constexpr uint8_t CRC_EXTRA = 221;
    uint16_t count{0};
    uint8_t target_system{0}, target_component{0}, mission_type{0};
    static MissionCount decode(const uint8_t* p) {
        Reader r(p); MissionCount m;
        m.count = r.u16(); m.target_system = r.u8(); m.target_component = r.u8(); m.mission_type = r.u8();
        return m;
    }
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u16(count); w.u8(target_system); w.u8(target_component); w.u8(mission_type);
        w.u32(0); // opaque_id (ext, newer common.xml): 0 = not supported
        return w.size();
    }
};

struct MissionAck {
    static constexpr uint32_t ID = 47;
    static constexpr uint8_t CRC_EXTRA = 153;
    uint8_t target_system{0}, target_component{0}, type{0}, mission_type{0};
};

struct ManualControl {
    static constexpr uint32_t ID = 69;
    static constexpr uint8_t CRC_EXTRA = 243;
    int16_t x{0}, y{0}, z{0}, r{0}; // pitch fwd, roll right, thrust, yaw right (-1000..1000, z 0..1000)
    uint16_t buttons{0};
    uint8_t target{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.i16(x); w.i16(y); w.i16(z); w.i16(r); w.u16(buttons); w.u8(target);
        return w.size();
    }
    static ManualControl decode(const uint8_t* p) {
        Reader rd(p); ManualControl m;
        m.x = rd.i16(); m.y = rd.i16(); m.z = rd.i16(); m.r = rd.i16(); m.buttons = rd.u16();
        m.target = rd.u8();
        return m;
    }
};

struct VfrHud {
    static constexpr uint32_t ID = 74;
    static constexpr uint8_t CRC_EXTRA = 20;
    float airspeed{0}, groundspeed{0}, alt{0}, climb{0};
    int16_t heading{0};    // deg 0..359
    uint16_t throttle{0};  // %
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.f32(airspeed); w.f32(groundspeed); w.f32(alt); w.f32(climb); w.i16(heading); w.u16(throttle);
        return w.size();
    }
};

struct CommandLong {
    static constexpr uint32_t ID = 76;
    static constexpr uint8_t CRC_EXTRA = 152;
    float param[7]{};
    uint16_t command{0};
    uint8_t target_system{0}, target_component{0}, confirmation{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        for (float f : param) w.f32(f);
        w.u16(command); w.u8(target_system); w.u8(target_component); w.u8(confirmation);
        return w.size();
    }
    static CommandLong decode(const uint8_t* p) {
        Reader r(p); CommandLong m;
        for (float& f : m.param) f = r.f32();
        m.command = r.u16(); m.target_system = r.u8(); m.target_component = r.u8(); m.confirmation = r.u8();
        return m;
    }
};

struct CommandInt {
    static constexpr uint32_t ID = 75;
    static constexpr uint8_t CRC_EXTRA = 158;
    float param1{0}, param2{0}, param3{0}, param4{0};
    int32_t x{0}, y{0};
    float z{0};
    uint16_t command{0};
    uint8_t target_system{0}, target_component{0}, frame{0}, current{0}, autocontinue{0};
    static CommandInt decode(const uint8_t* p) {
        Reader r(p); CommandInt m;
        m.param1 = r.f32(); m.param2 = r.f32(); m.param3 = r.f32(); m.param4 = r.f32();
        m.x = r.i32(); m.y = r.i32(); m.z = r.f32(); m.command = r.u16(); m.target_system = r.u8();
        m.target_component = r.u8(); m.frame = r.u8(); m.current = r.u8(); m.autocontinue = r.u8();
        return m;
    }
};

struct CommandAck {
    static constexpr uint32_t ID = 77;
    static constexpr uint8_t CRC_EXTRA = 143;
    uint16_t command{0};
    uint8_t result{0};
    uint8_t progress{0};
    int32_t result_param2{0};
    uint8_t target_system{0}, target_component{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u16(command); w.u8(result);
        w.u8(progress); w.i32(result_param2); w.u8(target_system); w.u8(target_component); // extensions
        return w.size();
    }
    static CommandAck decode(const uint8_t* p) {
        Reader r(p); CommandAck m;
        m.command = r.u16(); m.result = r.u8(); m.progress = r.u8(); m.result_param2 = r.i32();
        m.target_system = r.u8(); m.target_component = r.u8();
        return m;
    }
};

struct Timesync {
    static constexpr uint32_t ID = 111;
    static constexpr uint8_t CRC_EXTRA = 34;
    int64_t tc1{0}, ts1{0};
    uint8_t target_system{0}, target_component{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.i64(tc1); w.i64(ts1); w.u8(target_system); w.u8(target_component); // targets are extensions
        return w.size();
    }
    static Timesync decode(const uint8_t* p) {
        Reader r(p); Timesync m;
        m.tc1 = r.i64(); m.ts1 = r.i64(); m.target_system = r.u8(); m.target_component = r.u8();
        return m;
    }
};

struct AutopilotVersion {
    static constexpr uint32_t ID = 148;
    static constexpr uint8_t CRC_EXTRA = 178;
    uint64_t capabilities{0}, uid{0};
    uint32_t flight_sw_version{0}, middleware_sw_version{0}, os_sw_version{0}, board_version{0};
    uint16_t vendor_id{0}, product_id{0};
    uint8_t flight_custom_version[8]{}, middleware_custom_version[8]{}, os_custom_version[8]{};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u64(capabilities); w.u64(uid); w.u32(flight_sw_version); w.u32(middleware_sw_version);
        w.u32(os_sw_version); w.u32(board_version); w.u16(vendor_id); w.u16(product_id);
        w.bytes(flight_custom_version, 8); w.bytes(middleware_custom_version, 8); w.bytes(os_custom_version, 8);
        w.bytes(nullptr, 18); // uid2 (ext)
        return w.size();
    }
};

struct StatusText {
    static constexpr uint32_t ID = 253;
    static constexpr uint8_t CRC_EXTRA = 83;
    uint8_t severity{MAV_SEVERITY_INFO};
    char text[51]{};
    uint16_t id{0};
    uint8_t chunk_seq{0};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u8(severity); w.chars(text, 50); w.u16(id); w.u8(chunk_seq);
        return w.size();
    }
    static StatusText decode(const uint8_t* p) {
        Reader r(p); StatusText m;
        m.severity = r.u8(); r.chars(m.text, 50); m.id = r.u16(); m.chunk_seq = r.u8();
        return m;
    }
};

struct SetupSigning {
    static constexpr uint32_t ID = 256;
    static constexpr uint8_t CRC_EXTRA = 71;
    uint64_t initial_timestamp{0};
    uint8_t target_system{0}, target_component{0};
    uint8_t secret_key[32]{};
    size_t encode(uint8_t* b) const {
        Writer w(b);
        w.u64(initial_timestamp); w.u8(target_system); w.u8(target_component); w.bytes(secret_key, 32);
        return w.size();
    }
    static SetupSigning decode(const uint8_t* p) {
        Reader r(p); SetupSigning m;
        m.initial_timestamp = r.u64(); m.target_system = r.u8(); m.target_component = r.u8();
        for (auto& k : m.secret_key) k = r.u8();
        return m;
    }
};

/** Encode any message struct above into a ready-to-send v2 frame. */
template <typename Msg>
size_t pack(uint8_t* frame, uint8_t seq, uint8_t sysid, uint8_t compid, const Msg& m, uint8_t incompat_flags = 0) {
    uint8_t payload[MAX_PAYLOAD] = {};
    const size_t n = m.encode(payload);
    return encode_v2(frame, seq, sysid, compid, Msg::ID, payload, n, Msg::CRC_EXTRA, incompat_flags);
}

} // namespace mavlink
} // namespace comms
} // namespace autopilot
