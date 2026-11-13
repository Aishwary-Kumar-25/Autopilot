#pragma once
/**
 * External physics over the ArduPilot "JSON" SITL protocol.
 *
 * Test/SITL only. This is the same wire protocol ArduPilot's SITL uses for
 * external simulators (libraries/SITL/SIM_JSON.cpp), so anything written for it
 * drives our firmware unchanged, most usefully Gazebo Harmonic with the
 * ardupilot_gazebo plugin (iris quad, rovers, boats in a 3D world, ROS 2 via
 * ros_gz). Lock-step, one exchange per physics step:
 *
 *   SITL -> simulator (binary, little endian, to UDP 9002 by default)
 *       uint16 magic = 18458; uint16 frame_rate; uint32 frame_count; uint16 pwm[16]
 *   simulator -> SITL (one JSON object per datagram)
 *       {"timestamp": s,
 *        "imu": {"gyro": [p,q,r], "accel_body": [x,y,z]},    FRD, rad/s and m/s^2 (specific force)
 *        "position": [n,e,d], "velocity": [vn,ve,vd],         NED, m and m/s
 *        "quaternion": [w,x,y,z]  or  "attitude": [roll,pitch,yaw]}
 *
 * The simulator steps its physics only when a servo packet arrives, so the
 * simulation runs exactly as fast as both sides allow and never drifts.
 */
#include "sim/external_body.hpp"
#include "sim/udp_socket.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

namespace autopilot {
namespace sim {

struct JsonFrame {
    double timestamp{0};
    double gyro[3]{}, accel_body[3]{0, 0, -9.80665}, pos[3]{}, vel[3]{}, quat[4]{1, 0, 0, 0};
};

/** Parse one physics frame. Returns false if a required field is missing. */
inline bool parse_json_frame(const char* text, JsonFrame& f) {
    auto find_key = [&](const char* key) -> const char* {
        const std::string k = std::string("\"") + key + "\"";
        const char* p = std::strstr(text, k.c_str());
        if (!p) return nullptr;
        p = std::strchr(p + k.size(), ':');
        return p ? p + 1 : nullptr;
    };
    auto read_array = [&](const char* key, double* out, int n) -> bool {
        const char* p = find_key(key);
        if (!p) return false;
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
        if (*p != '[') return false;
        ++p;
        for (int i = 0; i < n; ++i) {
            char* end = nullptr;
            out[i] = std::strtod(p, &end);
            if (end == p || !std::isfinite(out[i])) return false;
            p = end;
            while (*p == ' ' || *p == ',' || *p == '\n' || *p == '\t' || *p == '\r') ++p;
        }
        return true;
    };
    const char* ts = find_key("timestamp");
    if (!ts) return false;
    f.timestamp = std::strtod(ts, nullptr);
    if (!read_array("gyro", f.gyro, 3) || !read_array("accel_body", f.accel_body, 3) ||
        !read_array("position", f.pos, 3) || !read_array("velocity", f.vel, 3)) {
        return false;
    }
    if (!read_array("quaternion", f.quat, 4)) {
        double e[3];
        if (!read_array("attitude", e, 3)) return false;
        const double cr = std::cos(e[0] / 2), sr = std::sin(e[0] / 2), cp = std::cos(e[1] / 2),
                     sp = std::sin(e[1] / 2), cy = std::cos(e[2] / 2), sy = std::sin(e[2] / 2);
        f.quat[0] = cr * cp * cy + sr * sp * sy;
        f.quat[1] = sr * cp * cy - cr * sp * sy;
        f.quat[2] = cr * sp * cy + sr * cp * sy;
        f.quat[3] = cr * cp * sy - sr * sp * cy;
    }
    return true;
}

class JsonPhysics {
public:
    struct Config {
        std::string host{"127.0.0.1"};
        uint16_t port{9002};
        // The firmware outputs 1000..2000 us. Many external models expect another
        // ESC range (ardupilot_gazebo's iris: 1100..1900), so pulses are rescaled
        // linearly onto [pwm_min, pwm_max]. 0 (output disabled) stays 0.
        uint16_t pwm_min{1000}, pwm_max{2000};
        uint16_t frame_rate{1000};   // advertised physics rate (informational)
    };

    explicit JsonPhysics(const Config& c) : _cfg(c) {}

    bool open() {
        _sock.follow_sender = false;
        if (!_sock.open(0)) return false;
        return _sock.set_peer(_cfg.host, _cfg.port);
    }
    const std::string& error() const { return _sock.error(); }

    /**
     * Send the outputs and wait for the resulting physics frame (lock-step).
     * pwm == nullptr / n == 0: outputs disabled (all channels 0).
     * Returns false on timeout (the caller resends; the same frame_count makes a
     * lock-step simulator repeat its last state instead of stepping twice).
     */
    bool exchange(const uint16_t* pwm, int n, JsonFrame& out, int timeout_ms, bool new_frame = true) {
        if (new_frame) ++_frame_count;
        Packet pkt{};
        pkt.magic = 18458;
        pkt.frame_rate = _cfg.frame_rate;
        pkt.frame_count = _frame_count;
        for (int i = 0; i < 16; ++i) {
            uint16_t us = (pwm && i < n) ? pwm[i] : 0;
            if (us != 0) {
                const double x = std::fmax(0.0, std::fmin(1.0, (us - 1000.0) / 1000.0));
                us = static_cast<uint16_t>(std::lround(_cfg.pwm_min + x * (_cfg.pwm_max - _cfg.pwm_min)));
            }
            pkt.pwm[i] = us;
        }
        _sock.send(&pkt, sizeof(pkt));
        // Take the newest frame that arrives (drop stale duplicates).
        char buf[8192];
        bool got = false;
        int wait = timeout_ms;
        for (;;) {
            const int len = _sock.recv(buf, sizeof(buf) - 1, wait);
            if (len < 0) break;
            buf[len] = '\0';
            JsonFrame f;
            if (parse_json_frame(buf, f)) {
                out = f;
                got = true;
                ++_frames;
            } else {
                ++_bad;
            }
            wait = 0;   // drain whatever else is queued, then stop
        }
        return got;
    }

    uint32_t frames() const { return _frames; }
    uint32_t bad_frames() const { return _bad; }

private:
#pragma pack(push, 1)
    struct Packet {
        uint16_t magic;
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[16];
    };
#pragma pack(pop)
    static_assert(sizeof(Packet) == 40, "servo packet layout");

    Config _cfg;
    UdpSocket _sock;
    uint32_t _frame_count{0};
    uint32_t _frames{0}, _bad{0};
};

} // namespace sim
} // namespace autopilot
