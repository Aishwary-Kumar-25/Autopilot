#include "comms/gcs_link.hpp"
#include "comms/vehicle_params.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace autopilot {
namespace comms {

using namespace mavlink;

namespace {
constexpr float RAD2DEG = 57.2957795f;

int16_t clamp_i16(float v) { return static_cast<int16_t>(std::clamp(v, -32767.0f, 32767.0f)); }

const char* health_name(drivers::SensorHealth h) {
    switch (h) {
        case drivers::SensorHealth::HEALTHY: return "healthy";
        case drivers::SensorHealth::DEGRADED: return "degraded";
        case drivers::SensorHealth::FAILED: return "FAILED";
        case drivers::SensorHealth::DISCONNECTED: return "disconnected";
        default: return "uninitialized";
    }
}

/** INT32 parameters travel bit-for-bit inside the float field (MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE). */
float encode_param(float v, ParamStore::Type t) {
    if (t != ParamStore::Type::INT32) return v;
    const int32_t i = static_cast<int32_t>(std::lround(v));
    float f;
    std::memcpy(&f, &i, 4);
    return f;
}
float decode_param(float wire, uint8_t mav_type, ParamStore::Type t) {
    if (mav_type == MAV_PARAM_TYPE_INT32 || (mav_type != MAV_PARAM_TYPE_REAL32 && t == ParamStore::Type::INT32)) {
        int32_t i;
        std::memcpy(&i, &wire, 4);
        return static_cast<float>(i);
    }
    return wire;
}
} // namespace

GcsLink::GcsLink(hal::UARTDevice* port, vehicles::VehicleManager* vm, ParamStore* params, const Config& cfg)
    : _port(port), _vm(vm), _params(params), _cfg(cfg),
      _streams{{Heartbeat::ID, 1000, 1000, 0},        {SysStatus::ID, 1000, 1000, 0},
               {Attitude::ID, 100, 100, 0},           {VfrHud::ID, 250, 250, 0},
               {GlobalPositionInt::ID, 200, 200, 0},  {GpsRawInt::ID, 500, 500, 0},
               {ServoOutputRaw::ID, 500, 500, 0}} {}

GcsLink::~GcsLink() {
    // The sensor manager outlives us: don't leave it calling into a dead object.
    if (_sensor_callback_installed && _vm && _vm->get_sensors()) _vm->get_sensors()->set_event_callback(nullptr);
}

uint32_t GcsLink::now() const {
    hal::HAL* h = _vm ? _vm->get_hal() : nullptr;
    return (h && h->get_time()) ? h->get_time()->millis() : 0;
}

bool GcsLink::for_us(uint8_t target_sys, uint8_t target_comp) const {
    return (target_sys == 0 || target_sys == _cfg.sysid) && (target_comp == 0 || target_comp == _cfg.compid);
}

void GcsLink::update() {
    if (!_vm) return;
    if (!_sensor_callback_installed && _vm->get_sensors()) {
        _vm->get_sensors()->set_event_callback([this](const char* name, drivers::SensorHealth h) {
            char t[51];
            std::snprintf(t, sizeof(t), "Sensor %s %s", name ? name : "?", health_name(h));
            send_statustext(h == drivers::SensorHealth::HEALTHY ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, t);
        });
        _sensor_callback_installed = true;
    }
    const uint32_t t = now();
    if (_signing) _signing->tick(t);
    receive();
    detect_events();
    if (_settings) _settings->update(_vm->is_armed(), t);
    stream_telemetry(t);
    stream_params();
    flush_texts();
}

// ---------------------------------------------------------------- receive

void GcsLink::receive() {
    if (!_port) return;
    uint8_t buf[256];
    size_t n;
    // Bounded per call so a flood can't starve the flight loop.
    for (int chunk = 0; chunk < 16 && (n = _port->read(buf, sizeof(buf))) > 0; ++chunk) {
        for (size_t i = 0; i < n; ++i) {
            if (_parser.parse(buf[i])) {
                bool signature_ok = false;
                if (!accept_frame(_parser.msg(), signature_ok)) continue;
                ++_stats.rx_messages;
                handle(_parser.msg(), signature_ok);
            }
        }
    }
    _stats.rx_crc_errors = _parser.crc_errors();
    _stats.rx_lost = _parser.seq_lost();
}

bool GcsLink::accept_frame(const Message& msg, bool& signature_ok) {
    signature_ok = false;
    if (!_signing || !_signing->enabled()) return true;
    const auto r = _signing->verify(_parser.frame(), _parser.frame_len(), msg.sysid, msg.compid);
    if (r == mavlink::Signing::Result::OK) {
        signature_ok = true;
        return true;
    }
    // The one unsigned message we take: SETUP_SIGNING over a trusted (wired) link,
    // so a GCS that lost the key can be re-keyed over USB.
    if (r == mavlink::Signing::Result::UNSIGNED && msg.msgid == SetupSigning::ID && _cfg.trusted_link) return true;

    if (r == mavlink::Signing::Result::UNSIGNED) ++_stats.rx_rejected_unsigned;
    else ++_stats.rx_rejected_signature;
    const uint32_t t = now();
    if (!_reject_text_sent || t - _last_reject_text_ms >= 10000) {
        _reject_text_sent = true;
        _last_reject_text_ms = t;
        send_statustext(MAV_SEVERITY_WARNING, r == mavlink::Signing::Result::UNSIGNED
                                                  ? "Signing on: unsigned MAVLink ignored"
                                                  : "Signing: bad/replayed signature ignored");
    }
    return false;
}

void GcsLink::save_signing() {
    if (_signing && _settings && _settings->storage()) _signing->save(_settings->storage());
}

void GcsLink::handle_setup_signing(const Message& msg, bool signature_ok) {
    const auto m = SetupSigning::decode(msg.payload);
    if (!for_us(m.target_system, m.target_component)) return;
    if (!_signing) {
        send_statustext(MAV_SEVERITY_WARNING, "Signing not supported on this link");
        return;
    }
    if (_vm->is_armed()) {
        send_statustext(MAV_SEVERITY_WARNING, "Signing setup refused: disarm first");
        return;
    }
    // Installing the first key over an untrusted link would let anyone in radio
    // range lock the owner out; changing an existing key needs the old key.
    if (!_cfg.trusted_link && !signature_ok) {
        send_statustext(MAV_SEVERITY_WARNING, "Signing setup refused: needs USB or current key");
        return;
    }
    _signing->set_key(m.secret_key, m.initial_timestamp);
    save_signing();
    send_statustext(MAV_SEVERITY_NOTICE, _signing->enabled() ? "Signing enabled" : "Signing disabled");
}

uint8_t GcsLink::handle_preflight_storage(int action) {
    if (!_settings || !_settings->available()) return MAV_RESULT_UNSUPPORTED;
    if (_vm->is_armed()) {
        send_statustext(MAV_SEVERITY_WARNING, "Param storage: disarm first");
        return MAV_RESULT_DENIED;
    }
    switch (action) {
        case 0: {   // PARAM_READ_PERSISTENT: back to the saved values
            _params->reset_to_defaults();
            const auto r = _settings->load();
            char t[51];
            std::snprintf(t, sizeof(t), "Params loaded: %u", static_cast<unsigned>(r.loaded));
            send_statustext(MAV_SEVERITY_INFO, t);
            return MAV_RESULT_ACCEPTED;
        }
        case 1:     // PARAM_WRITE_PERSISTENT
            if (!_settings->save_now()) return MAV_RESULT_FAILED;
            send_statustext(MAV_SEVERITY_INFO, "Params saved");
            return MAV_RESULT_ACCEPTED;
        case 2:     // PARAM_RESET_FACTORY_DEFAULT (QGC "Reset all to firmware's defaults")
        case 4: {   // PARAM_RESET_ALL_DEFAULT
            const size_t refused = _params->reset_to_defaults();
            if (!_settings->save_now() || refused) return MAV_RESULT_FAILED;
            send_statustext(MAV_SEVERITY_NOTICE, "Params reset to defaults");
            return MAV_RESULT_ACCEPTED;
        }
        default:
            return MAV_RESULT_UNSUPPORTED;
    }
}

void GcsLink::handle(const Message& msg, bool signature_ok) {
    if (msg.sysid == _cfg.sysid && msg.compid == _cfg.compid) return; // our own echo

    switch (msg.msgid) {
        case Heartbeat::ID: {
            const Heartbeat hb = Heartbeat::decode(msg.payload);
            if (hb.type == MAV_TYPE_GCS) {
                if (!gcs_connected()) {
                    _gcs_sysid = msg.sysid;
                    _gcs_compid = msg.compid;
                    _gcs_seen = true;
                    _last_gcs_heartbeat_ms = now();
                    send_statustext(MAV_SEVERITY_INFO, "GCS connected");
                    send_autopilot_version();
                }
                _last_gcs_heartbeat_ms = now();
            }
            break;
        }
        case ParamRequestList::ID: {
            const auto m = ParamRequestList::decode(msg.payload);
            if (!for_us(m.target_system, m.target_component) || !_params) break;
            _param_streaming = true;
            _param_next = 0;
            break;
        }
        case ParamRequestRead::ID: {
            const auto m = ParamRequestRead::decode(msg.payload);
            if (!for_us(m.target_system, m.target_component) || !_params) break;
            const int idx = (m.param_index >= 0) ? m.param_index : _params->find(m.param_id);
            if (idx >= 0 && static_cast<size_t>(idx) < _params->count()) send_param(static_cast<size_t>(idx));
            break;
        }
        case ParamSet::ID: {
            const auto m = ParamSet::decode(msg.payload);
            if (!for_us(m.target_system, m.target_component) || !_params) break;
            const int idx = _params->find(m.param_id);
            if (idx < 0) {
                char t[51];
                std::snprintf(t, sizeof(t), "Unknown param %s", m.param_id);
                send_statustext(MAV_SEVERITY_WARNING, t);
                break;
            }
            const auto* p = _params->at(static_cast<size_t>(idx));
            const float v = decode_param(m.param_value, m.param_type, p->type);
            const auto r = _params->set(static_cast<size_t>(idx), v);
            if (r != ParamStore::SetResult::OK) {
                char t[51];
                std::snprintf(t, sizeof(t), "%s: %s", p->name,
                              r == ParamStore::SetResult::OUT_OF_RANGE ? "out of range" : "refused (disarm first?)");
                send_statustext(MAV_SEVERITY_WARNING, t);
            }
            // Always answer with the value now in effect: that's how the GCS learns success/failure.
            send_param(static_cast<size_t>(idx));
            break;
        }
        case CommandLong::ID: {
            const auto m = CommandLong::decode(msg.payload);
            if (!for_us(m.target_system, m.target_component)) break;
            handle_command(m.command, m.param, msg.sysid, msg.compid);
            break;
        }
        case CommandInt::ID: {
            const auto m = CommandInt::decode(msg.payload);
            if (!for_us(m.target_system, m.target_component)) break;
            const float p[7] = {m.param1, m.param2, m.param3, m.param4, static_cast<float>(m.x),
                                static_cast<float>(m.y), m.z};
            handle_command(m.command, p, msg.sysid, msg.compid);
            break;
        }
        case SetMode::ID: {
            const auto m = SetMode::decode(msg.payload);
            if (m.target_system != _cfg.sysid) break;
            set_base_mode(m.base_mode, m.custom_mode);
            break;
        }
        case ManualControl::ID: {
            const auto m = ManualControl::decode(msg.payload);
            if (m.target != _cfg.sysid || !_cfg.accept_manual_control) break;
            ++_stats.manual_control_msgs;
            handle_manual_control(m);
            break;
        }
        case MissionRequestList::ID: {
            const auto m = MissionRequestList::decode(msg.payload);
            if (!for_us(m.target_system, m.target_component)) break;
            MissionCount c;              // no mission storage yet: report an empty plan
            c.count = 0;
            c.target_system = msg.sysid;
            c.target_component = msg.compid;
            c.mission_type = m.mission_type;
            send(c);
            break;
        }
        case Timesync::ID: {
            const auto m = Timesync::decode(msg.payload);
            if (m.tc1 != 0) break;       // a reply to someone else's request
            Timesync r;
            hal::HAL* h = _vm->get_hal();
            r.tc1 = (h && h->get_time()) ? static_cast<int64_t>(h->get_time()->micros()) * 1000 : 0;
            r.ts1 = m.ts1;
            r.target_system = msg.sysid;
            r.target_component = msg.compid;
            send(r);
            break;
        }
        case Ping::ID: {
            const auto m = Ping::decode(msg.payload);
            if (m.target_system != 0) break; // request only
            Ping r = m;
            r.target_system = msg.sysid;
            r.target_component = msg.compid;
            send(r);
            break;
        }
        case SetupSigning::ID:
            handle_setup_signing(msg, signature_ok);
            break;
        default:
            break;
    }
}

void GcsLink::handle_command(uint16_t command, const float p[7], uint8_t src_sys, uint8_t src_comp) {
    uint8_t result = MAV_RESULT_UNSUPPORTED;
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            if (p[0] > 0.5f) {
                std::string err;
                if (_vm->arm(err)) {
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                    send_statustext(MAV_SEVERITY_CRITICAL, err.empty() ? "Arming refused" : err.c_str());
                }
            } else {
                const bool force = std::lround(p[1]) == ARM_DISARM_FORCE_MAGIC;
                auto* uav = dynamic_cast<vehicles::UAVVehicle*>(_vm->get_active_vehicle());
                if (_vm->is_armed() && uav && !uav->is_landed() && !force) {
                    result = MAV_RESULT_DENIED;
                    send_statustext(MAV_SEVERITY_CRITICAL, "Disarm refused: not landed (force to override)");
                } else {
                    _vm->disarm();
                    result = MAV_RESULT_ACCEPTED;
                }
            }
            break;
        }
        case MAV_CMD_DO_SET_MODE:
            result = set_base_mode(static_cast<uint8_t>(p[0]), static_cast<uint32_t>(p[1])) ? MAV_RESULT_ACCEPTED
                                                                                            : MAV_RESULT_DENIED;
            break;
        case MAV_CMD_SET_MESSAGE_INTERVAL: {
            const uint32_t id = static_cast<uint32_t>(p[0]);
            const float us = p[1];
            bool ok = false;
            for (auto& s : _streams) {
                if (s.msgid != id) continue;
                if (us < 0.0f) s.interval_ms = 0;                         // -1: disable
                else if (us == 0.0f) s.interval_ms = s.default_ms;        //  0: default rate
                else s.interval_ms = std::max<uint32_t>(10, static_cast<uint32_t>(us / 1000.0f));
                ok = true;
            }
            result = ok ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED;
            break;
        }
        case MAV_CMD_REQUEST_MESSAGE:
            result = send_message_by_id(static_cast<uint32_t>(p[0])) ? MAV_RESULT_ACCEPTED : MAV_RESULT_UNSUPPORTED;
            break;
        case MAV_CMD_PREFLIGHT_STORAGE:
            result = handle_preflight_storage(static_cast<int>(std::lround(p[0])));
            break;
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            send_autopilot_version();
            result = MAV_RESULT_ACCEPTED;
            break;
        default:
            break;
    }
    send_ack(command, result, src_sys, src_comp);
}

void GcsLink::handle_manual_control(const ManualControl& mc) {
    // MANUAL_CONTROL axes: x = pitch stick (forward +), y = roll (right +),
    // z = thrust (0..1000), r = yaw (right +); +-1000 full scale, INT16_MAX = axis invalid.
    auto axis = [](int16_t v, float def) {
        return v == INT16_MAX ? def : std::clamp(static_cast<float>(v) / 1000.0f, -1.0f, 1.0f);
    };
    const float x = axis(mc.x, 0.0f), y = axis(mc.y, 0.0f), r = axis(mc.r, 0.0f);
    const float z = axis(mc.z, 0.0f);
    switch (_vm->get_active_type()) {
        case vehicles::VehicleType::UAV_MULTICOPTER:
            // forward stick = nose down = negative pitch (FRD)
            _vm->set_user_inputs(y, -x, r, std::clamp(z, 0.0f, 1.0f));
            break;
        case vehicles::VehicleType::UGV_GROUND_ROVER:
        case vehicles::VehicleType::USV_SURFACE_BOAT:
            _vm->set_user_inputs(0.0f, 0.0f, r, x, x, 0.0f);
            break;
        case vehicles::VehicleType::ROV_SUBMARINE:
            // thrust stick centred (500) = neutral heave
            _vm->set_user_inputs(0.0f, 0.0f, r, (std::clamp(z, 0.0f, 1.0f) - 0.5f) * 2.0f, x, y);
            break;
    }
}

bool GcsLink::set_custom_mode(uint32_t mode) {
    if (!set_vehicle_mode(*_vm, mode)) return false;
    char t[51];
    std::snprintf(t, sizeof(t), "Mode %s", vehicle_mode_name(_vm->get_active_type(), mode));
    send_statustext(MAV_SEVERITY_INFO, t);
    return true;
}

bool GcsLink::set_base_mode(uint8_t base_mode, uint32_t custom) {
    if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) return set_custom_mode(custom);
    // GCSs that only speak base-mode flags (e.g. QGC with a generic autopilot):
    // STABILIZE flag -> STABILIZE (mode 1), manual-input only -> MANUAL (mode 0).
    if (base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) return set_custom_mode(1);
    if (base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) return set_custom_mode(0);
    return false; // GUIDED / AUTO: no navigation modes yet
}

// ---------------------------------------------------------------- events

void GcsLink::detect_events() {
    vehicles::VehicleBase* v = _vm->get_active_vehicle();
    if (v != _last_vehicle) {
        if (_last_vehicle != nullptr) {
            // A new vehicle object starts with built-in defaults: re-apply tuning.
            if (_params) _params->apply_all();
            char t[51];
            std::snprintf(t, sizeof(t), "Vehicle: %s", _vm->get_active_type_name());
            send_statustext(MAV_SEVERITY_NOTICE, t);
            send_heartbeat(); // MAV_TYPE changed: tell the GCS now
        }
        _last_vehicle = v;
        _last_armed = _vm->is_armed();
        _last_fs = v ? v->get_failsafe().action() : control::FailsafeAction::NONE;
        return;
    }
    if (!v) return;

    // Vehicle events (motor failure, ...)
    vehicles::VehicleBase::Event ev;
    while (v->pop_event(ev)) send_statustext(static_cast<MavSeverity>(ev.severity), ev.text);

    const bool armed = _vm->is_armed();
    if (armed != _last_armed) {
        send_statustext(MAV_SEVERITY_INFO, armed ? "Armed" : "Disarmed");
        // Once per flight: save the signing clock, so after a reboot our
        // timestamps continue past everything the GCS has already seen.
        if (!armed && signing_enabled()) save_signing();
        _last_armed = armed;
    }
    const auto fs = v->get_failsafe().action();
    if (fs != _last_fs) {
        if (fs != control::FailsafeAction::NONE) {
            const uint8_t r = v->get_failsafe().reasons();
            char t[51];
            std::snprintf(t, sizeof(t), "Failsafe %s:%s%s%s%s%s", fs == control::FailsafeAction::LAND ? "LAND" : "TERMINATE",
                          (r & control::FS_RC_LOSS) ? " RC" : "", (r & control::FS_BATTERY) ? " BATT" : "",
                          (r & control::FS_ESTIMATOR) ? " EKF" : "", (r & control::FS_NO_IMU) ? " NO_IMU" : "",
                          (r & control::FS_MOTOR) ? " MOTOR" : "");
            send_statustext(fs == control::FailsafeAction::TERMINATE ? MAV_SEVERITY_EMERGENCY : MAV_SEVERITY_CRITICAL, t);
        } else {
            send_statustext(MAV_SEVERITY_NOTICE, "Failsafe cleared");
        }
        _last_fs = fs;
    }
}

// ---------------------------------------------------------------- transmit

void GcsLink::stream_telemetry(uint32_t now_ms) {
    for (auto& s : _streams) {
        if (s.interval_ms == 0) continue;
        if (s.last_ms != 0 && now_ms - s.last_ms < s.interval_ms) continue;
        s.last_ms = now_ms == 0 ? 1 : now_ms;
        send_message_by_id(s.msgid);
    }
}

bool GcsLink::send_message_by_id(uint32_t msgid) {
    switch (msgid) {
        case Heartbeat::ID: send_heartbeat(); return true;
        case SysStatus::ID: send_sys_status(); return true;
        case Attitude::ID: send_attitude(); return true;
        case VfrHud::ID: send_vfr_hud(); return true;
        case GlobalPositionInt::ID: send_global_position(); return true;
        case GpsRawInt::ID: send_gps_raw(); return true;
        case ServoOutputRaw::ID: send_servo_output(); return true;
        case AutopilotVersion::ID: send_autopilot_version(); return true;
        default: return false;
    }
}

void GcsLink::stream_params() {
    if (!_param_streaming || !_params) return;
    for (size_t i = 0; i < _cfg.params_per_update && _param_next < _params->count(); ++i) {
        send_param(_param_next++);
    }
    if (_param_next >= _params->count()) _param_streaming = false;
}

void GcsLink::send_statustext(MavSeverity severity, const char* text) {
    Text& t = _texts[(_text_head + _text_count) % TEXT_QUEUE];
    if (_text_count == TEXT_QUEUE) _text_head = (_text_head + 1) % TEXT_QUEUE; // overwrite oldest
    else ++_text_count;
    t.severity = severity;
    std::snprintf(t.text, sizeof(t.text), "%s", text ? text : "");
}

void GcsLink::flush_texts() {
    while (_text_count > 0) {
        const Text& t = _texts[_text_head];
        StatusText m;
        m.severity = t.severity;
        std::memcpy(m.text, t.text, sizeof(m.text));
        send(m);
        _text_head = (_text_head + 1) % TEXT_QUEUE;
        --_text_count;
    }
}

void GcsLink::set_battery(float voltage_v, float current_a, int remaining_pct) {
    _batt_v = voltage_v;
    _batt_a = current_a;
    _batt_pct = remaining_pct;
}

bool GcsLink::set_stream_interval(uint32_t msgid, uint32_t interval_ms) {
    for (auto& s : _streams) {
        if (s.msgid == msgid) {
            s.interval_ms = interval_ms;
            return true;
        }
    }
    return false;
}

uint8_t GcsLink::mav_type() const {
    switch (_vm->get_active_type()) {
        case vehicles::VehicleType::UGV_GROUND_ROVER: return MAV_TYPE_GROUND_ROVER;
        case vehicles::VehicleType::USV_SURFACE_BOAT: return MAV_TYPE_SURFACE_BOAT;
        case vehicles::VehicleType::ROV_SUBMARINE: return MAV_TYPE_SUBMARINE;
        case vehicles::VehicleType::UAV_MULTICOPTER:
        default: break;
    }
    auto* uav = dynamic_cast<const vehicles::UAVVehicle*>(_vm->get_active_vehicle());
    switch (uav ? uav->get_rotor_count() : 4) {
        case 3: return MAV_TYPE_TRICOPTER;
        case 6: return MAV_TYPE_HEXAROTOR;
        case 8: return MAV_TYPE_OCTOROTOR;
        default: return MAV_TYPE_QUADROTOR;
    }
}

uint32_t GcsLink::custom_mode() const { return get_vehicle_mode(*_vm); }

void GcsLink::send_heartbeat() {
    Heartbeat hb;
    hb.type = mav_type();
    hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.custom_mode = custom_mode();
    const bool armed = _vm->is_armed();
    hb.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (hb.custom_mode != 0) hb.base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    if (armed) hb.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    const auto* v = _vm->get_active_vehicle();
    const auto fs = v ? v->get_failsafe().action() : control::FailsafeAction::NONE;
    if (fs == control::FailsafeAction::TERMINATE) hb.system_status = MAV_STATE_EMERGENCY;
    else if (fs == control::FailsafeAction::LAND) hb.system_status = MAV_STATE_CRITICAL;
    else hb.system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    hb.mavlink_version = 3;
    send(hb);
}

void GcsLink::send_sys_status() {
    SysStatus m;
    auto* sm = _vm->get_sensors();
    auto* v = _vm->get_active_vehicle();
    if (sm) {
        bool has_imu = false, has_baro = false, has_mag = false, has_gps = false;
        for (const auto& s : sm->get_all_sensors()) {
            switch (s->type()) {
                case drivers::SensorType::IMU: has_imu = true; break;
                case drivers::SensorType::BAROMETER: has_baro = true; break;
                case drivers::SensorType::MAGNETOMETER: has_mag = true; break;
                case drivers::SensorType::GPS: has_gps = true; break;
                default: break;
            }
        }
        if (has_imu) m.present |= SENSOR_3D_GYRO | SENSOR_3D_ACCEL;
        if (has_baro) m.present |= SENSOR_ABSOLUTE_PRESSURE;
        if (has_mag) m.present |= SENSOR_3D_MAG;
        if (has_gps) m.present |= SENSOR_GPS;
        if (sm->has_healthy_imu()) m.health |= SENSOR_3D_GYRO | SENSOR_3D_ACCEL;
        if (sm->has_healthy_baro()) m.health |= SENSOR_ABSOLUTE_PRESSURE;
        if (sm->has_healthy_compass()) m.health |= SENSOR_3D_MAG;
        if (sm->has_healthy_gps()) m.health |= SENSOR_GPS;
    }
    m.present |= SENSOR_AHRS | SENSOR_RC_RECEIVER | SENSOR_MOTOR_OUTPUTS | SENSOR_PREARM_CHECK;
    if (_vm->get_latest_state().attitude_valid) m.health |= SENSOR_AHRS;
    if (v && !(v->get_failsafe().reasons() & control::FS_RC_LOSS)) m.health |= SENSOR_RC_RECEIVER;
    m.health |= SENSOR_MOTOR_OUTPUTS;
    if (auto* arming = _vm->get_arming()) {
        std::string why;
        if (_vm->is_armed() || arming->check_prearm(why)) m.health |= SENSOR_PREARM_CHECK;
    }
    if (_batt_v >= 0.0f) {
        m.present |= SENSOR_BATTERY;
        const bool low = v && (v->get_failsafe().reasons() & control::FS_BATTERY);
        if (!low) m.health |= SENSOR_BATTERY;
        m.voltage_battery = static_cast<uint16_t>(std::clamp(_batt_v * 1000.0f, 0.0f, 65534.0f));
        m.current_battery = _batt_a < 0.0f ? -1 : clamp_i16(_batt_a * 100.0f);
        m.battery_remaining = static_cast<int8_t>(std::clamp(_batt_pct, -1, 100));
    }
    m.enabled = m.present;
    const uint32_t total = _parser.frames_ok() + _parser.seq_lost();
    m.drop_rate_comm = total ? static_cast<uint16_t>(10000ull * _parser.seq_lost() / total) : 0;
    m.errors_comm = static_cast<uint16_t>(std::min<uint32_t>(_parser.crc_errors(), 65535));
    send(m);
}

void GcsLink::send_attitude() {
    const auto& s = _vm->get_latest_state();
    Attitude m;
    m.time_boot_ms = now();
    m.roll = s.euler_rad.x;
    m.pitch = s.euler_rad.y;
    m.yaw = s.euler_rad.z;
    m.rollspeed = s.angular_velocity.x;
    m.pitchspeed = s.angular_velocity.y;
    m.yawspeed = s.angular_velocity.z;
    send(m);
}

void GcsLink::send_vfr_hud() {
    const auto& s = _vm->get_latest_state();
    VfrHud m;
    m.groundspeed = std::sqrt(s.velocity_ned.x * s.velocity_ned.x + s.velocity_ned.y * s.velocity_ned.y);
    m.airspeed = m.groundspeed; // no airspeed sensor
    m.alt = s.altitude_m;
    const drivers::GPSSensor* gps = telemetry_gps();
    if (gps && gps->get_data().has_fix && gps->get_data().fix_type >= 3) m.alt = gps->get_data().altitude_msl_m;
    m.climb = s.vertical_velocity_m_s;
    float hdg = std::fmod(s.euler_rad.z * RAD2DEG + 360.0f, 360.0f);
    m.heading = static_cast<int16_t>(hdg);
    const auto* v = _vm->get_active_vehicle();
    const float thr = (v && _vm->is_armed()) ? std::fabs(v->get_demands().throttle) : 0.0f;
    m.throttle = static_cast<uint16_t>(std::clamp(thr, 0.0f, 1.0f) * 100.0f + 0.5f);
    send(m);
}

const drivers::GPSSensor* GcsLink::telemetry_gps() const {
    // The primary GPS if it has a fix, otherwise any usable GPS that does (the
    // sensor manager keeps a fix-less but "healthy" receiver as primary), otherwise
    // the primary so the GCS still sees "GPS present, no fix".
    auto* sm = _vm->get_sensors();
    if (!sm) return nullptr;
    const drivers::GPSSensor* active = sm->get_active_gps();
    if (active && active->get_data().has_fix && sm->is_usable(active)) return active;
    for (const auto& s : sm->get_all_sensors()) {
        if (s->type() != drivers::SensorType::GPS) continue;
        const auto* g = static_cast<const drivers::GPSSensor*>(s.get());
        if (g->get_data().has_fix && sm->is_usable(g)) return g;
    }
    return active;
}

void GcsLink::send_global_position() {
    const drivers::GPSSensor* gps = telemetry_gps();
    if (!gps) return;
    const auto& g = gps->get_data();
    if (!g.has_fix) return; // no position: don't put the vehicle at 0,0 on the map
    const auto& s = _vm->get_latest_state();
    GlobalPositionInt m;
    m.time_boot_ms = now();
    m.lat = static_cast<int32_t>(std::llround(g.latitude_deg * 1e7));
    m.lon = static_cast<int32_t>(std::llround(g.longitude_deg * 1e7));
    m.alt = static_cast<int32_t>(std::lround(g.altitude_msl_m * 1000.0f));
    m.relative_alt = static_cast<int32_t>(std::lround(s.altitude_m * 1000.0f));
    const math::Vector3f vel = s.position_valid ? s.velocity_ned : g.velocity_ned;
    m.vx = clamp_i16(vel.x * 100.0f);
    m.vy = clamp_i16(vel.y * 100.0f);
    m.vz = clamp_i16(vel.z * 100.0f);
    m.hdg = static_cast<uint16_t>(std::fmod(s.euler_rad.z * RAD2DEG + 360.0f, 360.0f) * 100.0f) % 36000;
    send(m);
}

void GcsLink::send_gps_raw() {
    const drivers::GPSSensor* gps = telemetry_gps();
    if (!gps) return;
    const auto& g = gps->get_data();
    GpsRawInt m;
    m.time_usec = static_cast<uint64_t>(g.timestamp_ms) * 1000ull;
    m.fix_type = g.has_fix ? std::max<uint8_t>(g.fix_type, 2) : 1; // 1 = GPS present, no fix
    m.lat = static_cast<int32_t>(std::llround(g.latitude_deg * 1e7));
    m.lon = static_cast<int32_t>(std::llround(g.longitude_deg * 1e7));
    m.alt = static_cast<int32_t>(std::lround(g.altitude_msl_m * 1000.0f));
    m.vel = static_cast<uint16_t>(std::clamp(g.speed_m_s * 100.0f, 0.0f, 65534.0f));
    m.cog = static_cast<uint16_t>(std::fmod(g.ground_course_deg + 360.0f, 360.0f) * 100.0f) % 36000;
    m.satellites_visible = g.satellites;
    send(m);
}

void GcsLink::send_servo_output() {
    hal::HAL* h = _vm->get_hal();
    hal::PWMOutput* pwm = h ? h->get_pwm() : nullptr;
    if (!pwm) return;
    ServoOutputRaw m;
    m.time_usec = now() * 1000u;
    const uint8_t n = std::min<uint8_t>(_vm->get_active_actuator_count(), 16);
    for (uint8_t i = 0; i < n; ++i) m.servo[i] = pwm->is_enabled() ? pwm->get_channel(i) : 0;
    send(m);
}

void GcsLink::send_autopilot_version() {
    AutopilotVersion m;
    m.capabilities = CAP_MAVLINK2 | CAP_PARAM_FLOAT | CAP_PARAM_ENCODE_BYTEWISE | CAP_COMMAND_INT | CAP_MISSION_INT;
    m.flight_sw_version = (1u << 24) | (0u << 16) | (0u << 8) | 0u; // 1.0.0, FIRMWARE_VERSION_TYPE_DEV
    m.uid = 0x4543'4C55'4241'5000ull | _cfg.sysid;                   // "ECLUBAP" + sysid
    send(m);
}

void GcsLink::send_param(size_t index) {
    const auto* p = _params ? _params->at(index) : nullptr;
    if (!p) return;
    ParamValue m;
    m.param_value = encode_param(_params->value(index), p->type);
    m.param_count = static_cast<uint16_t>(_params->count());
    m.param_index = static_cast<uint16_t>(index);
    std::memcpy(m.param_id, p->name, sizeof(m.param_id));
    m.param_type = p->type == ParamStore::Type::INT32 ? MAV_PARAM_TYPE_INT32 : MAV_PARAM_TYPE_REAL32;
    send(m);
}

void GcsLink::send_ack(uint16_t command, uint8_t result, uint8_t target_sys, uint8_t target_comp) {
    CommandAck a;
    a.command = command;
    a.result = result;
    a.target_system = target_sys;
    a.target_component = target_comp;
    send(a);
}

} // namespace comms
} // namespace autopilot
