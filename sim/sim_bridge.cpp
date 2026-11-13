#include "sim/sim_bridge.hpp"

#include "comms/vehicle_params.hpp"
#include "control/failsafe.hpp"
#include "vehicles/uav/uav_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace autopilot {
namespace sim {

namespace {

constexpr double RAD2DEG = 57.29577951308232;

/** Tiny JSON writer: just enough for flat numbers, arrays and strings. */
class Json {
public:
    std::string s;
    void raw(const char* t) { s += t; }
    void key(const char* k) {
        comma();
        s += '"';
        s += k;
        s += "\":";
        _first = true;   // the value follows directly, without a comma
    }
    void num(double v) {
        comma();
        if (!std::isfinite(v)) { s += "null"; return; }
        char b[32];
        std::snprintf(b, sizeof(b), "%.7g", v);
        s += b;
    }
    void boolean(bool v) { comma(); s += v ? "true" : "false"; }
    void str(const std::string& v) {
        comma();
        s += '"';
        for (char c : v) {
            if (c == '"' || c == '\\') { s += '\\'; s += c; }
            else if (static_cast<unsigned char>(c) < 0x20) s += ' ';
            else s += c;
        }
        s += '"';
    }
    void arr(const double* v, int n) {
        comma();
        s += '[';
        _first = true;
        for (int i = 0; i < n; ++i) num(v[i]);
        s += ']';
        _first = false;
    }
    void v3(const V3& v) { const double a[3] = {v.x, v.y, v.z}; arr(a, 3); }
    void v3f(const math::Vector3f& v) { const double a[3] = {v.x, v.y, v.z}; arr(a, 3); }
    void begin_obj() { comma(); s += '{'; _first = true; }
    void end_obj() { s += '}'; _first = false; }
    void begin_arr() { comma(); s += '['; _first = true; }
    void end_arr() { s += ']'; _first = false; }

private:
    bool _first{true};
    void comma() {
        if (!_first) s += ',';
        _first = false;
    }
};

const char* health_name(drivers::SensorHealth h) {
    switch (h) {
        case drivers::SensorHealth::HEALTHY: return "HEALTHY";
        case drivers::SensorHealth::DEGRADED: return "DEGRADED";
        case drivers::SensorHealth::FAILED: return "FAILED";
        case drivers::SensorHealth::DISCONNECTED: return "DISCONNECTED";
        default: return "UNINITIALIZED";
    }
}

const char* fault_name(Fault f) {
    switch (f) {
        case Fault::STUCK: return "stuck";
        case Fault::DROPOUT: return "dropout";
        case Fault::GYRO_OFFSET: return "gyro_offset";
        case Fault::NOT_A_NUMBER: return "nan";
        default: return "none";
    }
}

bool parse_fault(const std::string& s, Fault& f) {
    if (s == "none" || s == "ok" || s == "clear") f = Fault::NONE;
    else if (s == "dropout" || s == "dead" || s == "loss") f = Fault::DROPOUT;
    else if (s == "stuck" || s == "frozen") f = Fault::STUCK;
    else if (s == "gyro_offset") f = Fault::GYRO_OFFSET;
    else if (s == "nan") f = Fault::NOT_A_NUMBER;
    else return false;
    return true;
}

std::string failsafe_reason_names(uint8_t r) {
    std::string out;
    auto add = [&](uint8_t bit, const char* n) {
        if (r & bit) { if (!out.empty()) out += '+'; out += n; }
    };
    add(control::FS_RC_LOSS, "rc_loss");
    add(control::FS_BATTERY, "battery");
    add(control::FS_ESTIMATOR, "estimator");
    add(control::FS_NO_IMU, "no_imu");
    add(control::FS_MOTOR, "motor");
    return out;
}

bool vehicle_from_name(const std::string& v, vehicles::VehicleType& t) {
    if (v == "quad" || v == "uav" || v == "copter" || v == "0") t = vehicles::VehicleType::UAV_MULTICOPTER;
    else if (v == "rover" || v == "ugv" || v == "car" || v == "1") t = vehicles::VehicleType::UGV_GROUND_ROVER;
    else if (v == "sub" || v == "rov" || v == "2") t = vehicles::VehicleType::ROV_SUBMARINE;
    else if (v == "boat" || v == "usv" || v == "3") t = vehicles::VehicleType::USV_SURFACE_BOAT;
    else return false;
    return true;
}

} // namespace

bool SimBridge::open(uint16_t port, double rate_hz, bool any_interface) {
    _period = rate_hz > 0 ? 1.0 / rate_hz : 0.02;
    _sock.follow_sender = true;   // reply to whoever talks to us
    return _sock.open(port, any_interface);
}

void SimBridge::apply_env() {
    if (auto* quad = _world.as<QuadModel>()) {
        quad->wind = _env.wind;
        if (_env.quad_mass > 0) quad->mass = _env.quad_mass;
        if (_env.quad_max_thrust > 0) quad->max_thrust = _env.quad_max_thrust;
        for (int i = 0; i < QuadModel::MAX_MOTORS; ++i) quad->motor_health[i] = _env.motor_health[i];
    }
    if (auto* boat = _world.as<BoatModel>()) boat->current = _env.current;
}

void SimBridge::update(double t) {
    _t = t;
    // 1) everything that arrived
    char buf[2048];
    for (;;) {
        const int n = _sock.recv(buf, sizeof(buf) - 1, 0);
        if (n < 0) break;
        buf[n] = '\0';
        std::istringstream lines{std::string(buf)};
        std::string line;
        while (std::getline(lines, line)) {
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
            if (!line.empty()) handle_line(line);
        }
    }
    // 2) scheduled commands that are due, in time then arrival order
    if (!_queue.empty()) {
        std::stable_sort(_queue.begin(), _queue.end(), [](const Scheduled& a, const Scheduled& b) {
            return a.t < b.t || (a.t == b.t && a.seq < b.seq);
        });
        size_t done = 0;
        while (done < _queue.size() && _queue[done].t <= t + 1e-9) {
            std::string msg;
            const bool ok = execute(_queue[done].cmd, msg);
            char when[32];
            std::snprintf(when, sizeof(when), "at %g ", _queue[done].t);
            send_ack(when + _queue[done].cmd, ok, msg);
            ++done;
        }
        _queue.erase(_queue.begin(), _queue.begin() + static_cast<long>(done));
    }
    // 3) telemetry
    if (_sock.has_peer() && t + 1e-9 >= _next_t) {
        send_telemetry();
        _next_t = std::max(_next_t + _period, t);
    }
}

void SimBridge::handle_line(const std::string& line) {
    if (line.rfind("at ", 0) == 0) {
        std::istringstream in(line.substr(3));
        double when = 0;
        if (!(in >> when)) { send_ack(line, false, "usage: at <seconds> <command>"); return; }
        std::string rest;
        std::getline(in, rest);
        rest.erase(0, rest.find_first_not_of(' '));
        if (rest.empty()) { send_ack(line, false, "usage: at <seconds> <command>"); return; }
        _queue.push_back({when, rest, _seq++});
        send_ack(line, true, "scheduled");
        return;
    }
    // A command that acts now (anything but hello / ping / at, incl. "start") lets
    // simulated time run. Scheduled commands don't: "at 0 ..." runs while waiting.
    if (line != "hello" && line != "ping") _started = true;
    std::string msg;
    const bool ok = execute(line, msg);
    send_ack(line, ok, msg);
}

bool SimBridge::execute(const std::string& line, std::string& msg) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    auto read_v3 = [&](V3& v) -> bool { return static_cast<bool>(in >> v.x >> v.y >> v.z); };
    auto imu_targets = [&](const std::string& which) {
        std::vector<SimIMU*> out;
        if (which == "imu" || which == "imu0" || which == "all") out.push_back(_s.imu[0].get());
        if (which == "imu" || which == "imu1" || which == "all") out.push_back(_s.imu[1].get());
        out.erase(std::remove(out.begin(), out.end(), nullptr), out.end());
        return out;
    };

    if (cmd == "hello" || cmd == "ping") {
        msg = "autopilot_sitl bridge";
        return true;
    }
    if (cmd == "start") {
        msg = "running";
        return true;
    }
    if (cmd == "arm") {
        std::string why;
        if (_vm.arm(why)) { msg = "armed"; return true; }
        msg = why;
        return false;
    }
    if (cmd == "disarm") {
        _vm.disarm();
        msg = "disarmed";
        return true;
    }
    if (cmd == "mode") {
        std::string m;
        in >> m;
        uint32_t mode = 99;
        if (m == "0" || m == "manual") mode = 0;
        else if (m == "1" || m == "stabilize" || m == "heading_hold") mode = 1;
        else if (m == "2" || m == "alt_hold" || m == "depth_hold" || m == "speed_hold") mode = 2;
        if (mode == 99 || !comms::set_vehicle_mode(_vm, mode)) { msg = "unknown mode " + m; return false; }
        msg = comms::vehicle_mode_name(_vm.get_active_type(), mode);
        return true;
    }
    if (cmd == "vehicle") {
        std::string v;
        in >> v;
        vehicles::VehicleType type;
        if (!vehicle_from_name(v, type)) { msg = "unknown vehicle " + v; return false; }
        std::string why;
        if (_vm.get_active_type() != type && !_vm.switch_vehicle(type, why)) { msg = why; return false; }
        if (_world.sync()) apply_env();
        msg = _vm.get_active_type_name();
        return true;
    }
    if (cmd == "frame") {
        int n = 0;
        in >> n;
        if (_vm.is_armed()) { msg = "disarm first"; return false; }
        bool ok = false;
        if (_vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER) ok = _vm.configure_uav_rotors(static_cast<uint8_t>(n));
        else if (_vm.get_active_type() == vehicles::VehicleType::ROV_SUBMARINE) ok = _vm.configure_rov_thrusters(static_cast<uint8_t>(n));
        if (!ok) { msg = "frame not supported"; return false; }
        if (_world.sync()) apply_env();
        msg = "frame " + std::to_string(n);
        return true;
    }
    if (cmd == "sticks") {
        float v[6]{0, 0, 0, 0, 0, 0};
        int k = 0;
        while (k < 6 && (in >> v[k])) ++k;
        if (k < 4) { msg = "usage: sticks roll pitch yaw throttle [forward lateral]"; return false; }
        for (int i = 0; i < 6; ++i) _env.sticks[i] = std::fmax(-1.0f, std::fmin(1.0f, v[i]));
        _env.sticks_active = true;
        msg = "ok";
        return true;
    }
    if (cmd == "rc") {
        std::string s;
        in >> s;
        if (s == "off" || s == "lost" || s == "loss") _env.rc_lost = true;
        else if (s == "on") _env.rc_lost = false;
        else if (s == "release") _env.sticks_active = false;   // back to joystick / --rc-neutral
        else { msg = "usage: rc off|on|release"; return false; }
        msg = s;
        return true;
    }
    if (cmd == "fault") {
        std::string which, kind;
        in >> which >> kind;
        Fault f;
        if (!parse_fault(kind, f)) { msg = "unknown fault " + kind; return false; }
        if (which.rfind("imu", 0) == 0 || which == "all") {
            auto t = imu_targets(which);
            if (t.empty()) { msg = "unknown sensor " + which; return false; }
            for (auto* i : t) i->set_fault(f);
        } else if (which == "baro") {
            _s.baro->set_fault(f);
        } else if (which == "mag" || which == "compass") {
            _s.mag->set_fault(f);
        } else if (which == "gps") {
            _s.gps->set_fault(f);
        } else {
            msg = "unknown sensor " + which;
            return false;
        }
        msg = which + " " + fault_name(f);
        return true;
    }
    if (cmd == "gps_offset") { if (!read_v3(_s.gps->pos_offset)) { msg = "usage: gps_offset n e d"; return false; } return true; }
    if (cmd == "gps_vel_offset") { if (!read_v3(_s.gps->vel_offset)) { msg = "usage: gps_vel_offset n e d"; return false; } return true; }
    if (cmd == "mag_interference") { if (!read_v3(_s.mag->interference)) { msg = "usage: mag_interference x y z (gauss)"; return false; } return true; }
    if (cmd == "baro_offset") { if (!(in >> _s.baro->offset_m)) { msg = "usage: baro_offset metres"; return false; } return true; }
    if (cmd == "baro_noise") { if (!(in >> _s.baro->noise_m)) { msg = "usage: baro_noise metres"; return false; } return true; }
    if (cmd == "gps_noise") {
        if (!(in >> _s.gps->pos_noise >> _s.gps->vel_noise)) { msg = "usage: gps_noise pos_m vel_m_s"; return false; }
        return true;
    }
    if (cmd == "gyro_bias" || cmd == "accel_noise") {
        std::string which;
        in >> which;
        auto t = imu_targets(which);
        if (t.empty()) { msg = "unknown imu " + which; return false; }
        if (cmd == "gyro_bias") {
            V3 b;
            if (!read_v3(b)) { msg = "usage: gyro_bias imu0|imu1|imu x y z (rad/s)"; return false; }
            for (auto* i : t) i->gyro_bias = math::Vector3f(float(b.x), float(b.y), float(b.z));
        } else {
            double sd;
            if (!(in >> sd)) { msg = "usage: accel_noise imu0|imu1|imu sd"; return false; }
            for (auto* i : t) i->accel_noise = sd;
        }
        return true;
    }
    if (cmd == "motor") {
        int m = 0;
        double h = 1.0;
        if (!(in >> m >> h) || m < 1 || m > 16) { msg = "usage: motor <1..16> <health 0..1>"; return false; }
        _env.motor_health[m - 1] = std::fmax(0.0, std::fmin(1.0, h));
        apply_env();
        msg = "motor " + std::to_string(m);
        return true;
    }
    if (cmd == "wind") { if (!read_v3(_env.wind)) { msg = "usage: wind n e d (m/s)"; return false; } apply_env(); return true; }
    if (cmd == "current") {
        if (!(in >> _env.current.x >> _env.current.y)) { msg = "usage: current n e (m/s)"; return false; }
        apply_env();
        return true;
    }
    if (cmd == "mass") { if (!(in >> _env.quad_mass)) { msg = "usage: mass kg"; return false; } apply_env(); return true; }
    if (cmd == "max_thrust") { if (!(in >> _env.quad_max_thrust)) { msg = "usage: max_thrust N"; return false; } apply_env(); return true; }
    if (cmd == "battery_used") {
        if (!(in >> _bat.used_mah)) { msg = "usage: battery_used mAh"; return false; }
        return true;
    }
    if (cmd == "quit" || cmd == "exit") {
        _env.quit = true;
        msg = "bye";
        return true;
    }
    msg = "unknown command '" + cmd + "'";
    return false;
}

std::string SimBridge::active_faults() const {
    std::string out;
    auto add = [&](const std::string& s) { if (!out.empty()) out += ','; out += s; };
    for (int i = 0; i < 2; ++i) {
        if (!_s.imu[i]) continue;
        if (_s.imu[i]->fault() != Fault::NONE) add("imu" + std::to_string(i) + ":" + fault_name(_s.imu[i]->fault()));
        const auto& b = _s.imu[i]->gyro_bias;
        if (b.x != 0 || b.y != 0 || b.z != 0) add("imu" + std::to_string(i) + ":gyro_bias");
    }
    if (_s.baro->fault() != Fault::NONE) add(std::string("baro:") + fault_name(_s.baro->fault()));
    if (_s.baro->offset_m != 0) add("baro:offset");
    if (_s.mag->fault() != Fault::NONE) add(std::string("mag:") + fault_name(_s.mag->fault()));
    if (_s.mag->interference.norm() > 0) add("mag:interference");
    if (_s.gps->fault() != Fault::NONE) add(std::string("gps:") + fault_name(_s.gps->fault()));
    if (_s.gps->pos_offset.norm() > 0 || _s.gps->vel_offset.norm() > 0) add("gps:spoof");
    for (int i = 0; i < 16; ++i) {
        if (_env.motor_health[i] < 1.0) {
            char b[32];
            std::snprintf(b, sizeof(b), "motor%d:%.2f", i + 1, _env.motor_health[i]);
            add(b);
        }
    }
    if (_env.rc_lost) add("rc:lost");
    return out;
}

void SimBridge::send_ack(const std::string& cmd, bool ok, const std::string& msg) {
    if (!_sock.has_peer()) return;
    Json j;
    j.begin_obj();
    j.key("type"); j.str("ack");
    j.key("t"); j.num(_t);
    j.key("cmd"); j.str(cmd);
    j.key("ok"); j.boolean(ok);
    j.key("msg"); j.str(msg);
    j.end_obj();
    _sock.send(j.s);
}

void SimBridge::send_telemetry() {
    const SimBody& b = _world.body();
    const auto& st = _vm.get_latest_state();
    auto* veh = _vm.get_active_vehicle();
    const uint32_t mode = comms::get_vehicle_mode(_vm);
    Json j;
    j.begin_obj();
    j.key("type"); j.str("telem");
    j.key("t"); j.num(_t);
    j.key("seq"); j.num(_sent);
    j.key("vehicle"); j.str(_vm.get_active_type_name());
    j.key("body"); j.str(b.name());
    j.key("armed"); j.boolean(_vm.is_armed());
    j.key("mode"); j.num(mode);
    j.key("mode_name"); j.str(comms::vehicle_mode_name(_vm.get_active_type(), mode));

    j.key("truth");
    j.begin_obj();
    j.key("pos"); j.v3(b.pos);
    j.key("vel"); j.v3(b.vel);
    j.key("q"); j.arr(b.q, 4);
    const double rpy[3] = {b.roll() * RAD2DEG, b.pitch() * RAD2DEG, b.yaw() * RAD2DEG};
    j.key("rpy"); j.arr(rpy, 3);
    j.key("omega"); j.v3(b.omega);
    j.key("accel"); j.v3(b.accel_ned);
    j.end_obj();

    j.key("est");
    j.begin_obj();
    const double eq[4] = {st.attitude_q.q0, st.attitude_q.q1, st.attitude_q.q2, st.attitude_q.q3};
    j.key("q"); j.arr(eq, 4);
    const double erpy[3] = {st.euler_rad.x * RAD2DEG, st.euler_rad.y * RAD2DEG, st.euler_rad.z * RAD2DEG};
    j.key("rpy"); j.arr(erpy, 3);
    j.key("omega"); j.v3f(st.angular_velocity);
    j.key("alt"); j.num(st.altitude_m);
    j.key("vz_up"); j.num(st.vertical_velocity_m_s);
    j.key("pos"); j.v3f(st.position_ned);
    j.key("vel"); j.v3f(st.velocity_ned);
    j.key("gyro_bias"); j.v3f(st.gyro_bias);
    j.key("att_valid"); j.boolean(st.attitude_valid);
    j.key("alt_valid"); j.boolean(st.altitude_valid);
    j.key("pos_valid"); j.boolean(st.position_valid);
    j.key("att_source"); j.num(st.attitude_source);
    j.key("tilt_aiding"); j.num(st.tilt_aiding);
    j.end_obj();

    // Raw sensor readings, exactly what the firmware's drivers handed to the estimator.
    j.key("imu");
    j.begin_arr();
    for (int i = 0; i < 2; ++i) {
        if (!_s.imu[i]) continue;
        j.begin_obj();
        j.key("name"); j.str(_s.imu[i]->name());
        j.key("accel"); j.v3f(_s.imu[i]->get_accel());
        j.key("gyro"); j.v3f(_s.imu[i]->get_gyro());
        j.end_obj();
    }
    j.end_arr();
    j.key("baro");
    j.begin_obj();
    j.key("alt"); j.num(_s.baro->get_altitude_m());
    j.key("pressure"); j.num(_s.baro->get_pressure_pa());
    j.end_obj();
    j.key("mag"); j.v3f(_s.mag->get_field_gauss());
    const auto& g = _s.gps->get_data();
    j.key("gps");
    j.begin_obj();
    j.key("fix"); j.num(g.fix_type);
    j.key("sats"); j.num(g.satellites);
    j.key("lat"); j.num(g.latitude_deg);
    j.key("lon"); j.num(g.longitude_deg);
    j.key("alt"); j.num(g.altitude_msl_m);
    j.key("vel"); j.v3f(g.velocity_ned);
    j.end_obj();

    auto* sm = _vm.get_sensors();
    j.key("active");
    j.begin_obj();
    j.key("imu"); j.str(sm->get_active_imu() ? sm->get_active_imu()->name() : "");
    j.key("baro"); j.str(sm->get_active_baro() ? sm->get_active_baro()->name() : "");
    j.key("mag"); j.str(sm->get_active_compass() ? sm->get_active_compass()->name() : "");
    j.key("gps"); j.str(sm->get_active_gps() ? sm->get_active_gps()->name() : "");
    j.end_obj();
    j.key("health");
    j.begin_obj();
    for (const auto& s : sm->get_all_sensors()) {
        j.key(s->name());
        j.str(health_name(s->health()));
    }
    j.end_obj();

    // Outputs
    j.key("pwm");
    j.begin_arr();
    hal::PWMOutput* pwm = _vm.get_hal() ? _vm.get_hal()->get_pwm() : nullptr;
    const int n = _vm.get_active_actuator_count();
    for (int i = 0; i < n; ++i) j.num(pwm && pwm->is_enabled() ? pwm->get_channel(static_cast<uint8_t>(i)) : 0);
    j.end_arr();
    double rc[6];
    for (int i = 0; i < 6; ++i) rc[i] = _env.sticks[i];
    j.key("sticks"); j.arr(rc, 6);

    auto* uav = dynamic_cast<vehicles::UAVVehicle*>(veh);
    j.key("motor_failed"); j.num(uav ? uav->last_failed_motor() + 1 : 0);   // 1-based, 0 = none
    if (veh) {
        const auto& fs = veh->get_failsafe();
        static const char* names[] = {"NONE", "LAND", "TERMINATE"};
        j.key("failsafe");
        j.begin_obj();
        j.key("action"); j.str(names[std::min<int>(2, static_cast<int>(fs.action()))]);
        j.key("reasons"); j.str(failsafe_reason_names(fs.reasons()));
        j.end_obj();
    }
    j.key("battery");
    j.begin_obj();
    j.key("v"); j.num(_bat.voltage);
    j.key("a"); j.num(_bat.current_a);
    j.key("pct"); j.num(_bat.remaining_pct());
    j.end_obj();
    j.key("env");
    j.begin_obj();
    j.key("wind"); j.v3(_env.wind);
    j.key("current"); j.v3(_env.current);
    j.end_obj();
    j.key("faults"); j.str(active_faults());
    j.end_obj();
    _sock.send(j.s);
    ++_sent;
}

} // namespace sim
} // namespace autopilot
