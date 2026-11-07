#include "dronecan_node.hpp"

namespace autopilot {
namespace drivers {
namespace dronecan {

DroneCANNode::DroneCANNode(hal::CANDevice* can, hal::Time* time, uint8_t local_node_id)
    : _can(can), _time(time), _local_node_id(local_node_id) {}

bool DroneCANNode::init(uint32_t bitrate_bps) {
    if (!_can) return false;
    return _can->init(bitrate_bps);
}

void DroneCANNode::broadcast_node_status(NodeHealth health, NodeMode mode) {
    if (!_can || !_time) return;

    uint32_t now_ms = _time->millis();
    if (_last_status_tx_ms != 0 && now_ms - _last_status_tx_ms < 1000) {
        return;
    }
    _last_status_tx_ms = now_ms;

    NodeStatusMsg msg;
    msg.uptime_sec = now_ms / 1000;
    msg.health = static_cast<uint8_t>(health);
    msg.mode = static_cast<uint8_t>(mode);
    uint8_t payload[8];
    size_t len = encode_node_status(msg, payload, sizeof(payload));

    hal::CANFrame frame;
    // Priority 30 (low), single-frame transfer
    if (make_broadcast_frames(30, MSG_ID_NODE_STATUS, _local_node_id, _status_transfer_id++, payload, len, &frame, 1) == 1) {
        _can->send_frame(frame);
    }
}

void DroneCANNode::process_rx() {
    if (!_can || !_time) return;

    hal::CANFrame frame;
    Transfer t;
    while (_can->receive_frame(frame)) {
        if (_rx.accept(frame, _time->millis(), t)) {
            handle_transfer(t);
        }
    }
}

template <typename Map, typename T>
static std::shared_ptr<T> get_or_create(Map& map, uint8_t node, std::vector<std::shared_ptr<Sensor>>& all) {
    auto it = map.find(node);
    if (it != map.end()) return it->second;
    // Allocation happens once per newly discovered node, never per frame.
    auto sensor = std::make_shared<T>(node);
    sensor->init();
    map[node] = sensor;
    all.push_back(sensor);
    return sensor;
}

void DroneCANNode::handle_transfer(const Transfer& t) {
    const uint32_t now = t.timestamp_ms;
    switch (t.data_type_id) {
        case MSG_ID_NODE_STATUS: {
            NodeStatusMsg m;
            if (!decode_node_status(t.payload, t.len, m)) { ++_bad_payloads; return; }
            DiscoveredNode node{t.source_node, static_cast<NodeHealth>(m.health), static_cast<NodeMode>(m.mode), now};
            _active_nodes[t.source_node] = node;
            break;
        }
        case MSG_ID_RAW_IMU: {
            RawIMUMsg m;
            if (!decode_raw_imu(t.payload, t.len, m)) { ++_bad_payloads; return; }
            auto imu = get_or_create<decltype(_imus), DroneCAN_IMU>(_imus, t.source_node, _all_can_sensors);
            imu->ingest_raw_imu(math::Vector3f(m.gyro[0], m.gyro[1], m.gyro[2]),
                                math::Vector3f(m.accel[0], m.accel[1], m.accel[2]), now);
            break;
        }
        case MSG_ID_STATIC_PRESSURE: {
            StaticPressureMsg m;
            if (!decode_static_pressure(t.payload, t.len, m)) { ++_bad_payloads; return; }
            auto baro = get_or_create<decltype(_baros), DroneCAN_Baro>(_baros, t.source_node, _all_can_sensors);
            baro->ingest_pressure(m.pressure_pa, now);
            break;
        }
        case MSG_ID_STATIC_TEMPERATURE: {
            StaticTemperatureMsg m;
            if (!decode_static_temperature(t.payload, t.len, m)) { ++_bad_payloads; return; }
            auto it = _baros.find(t.source_node);
            if (it != _baros.end()) it->second->ingest_temperature(m.temperature_k);
            break;
        }
        case MSG_ID_MAGNETIC_FIELD:
        case MSG_ID_MAGNETIC_FIELD2: {
            MagFieldMsg m;
            if (!decode_mag_field(t.payload, t.len, t.data_type_id == MSG_ID_MAGNETIC_FIELD2, m)) { ++_bad_payloads; return; }
            auto compass = get_or_create<decltype(_compasses), DroneCAN_Compass>(_compasses, t.source_node, _all_can_sensors);
            compass->ingest_mag_field(math::Vector3f(m.field_ga[0], m.field_ga[1], m.field_ga[2]), now);
            break;
        }
        case MSG_ID_GNSS_FIX2: {
            GnssFix2Msg m;
            if (!decode_gnss_fix2(t.payload, t.len, m)) { ++_bad_payloads; return; }
            auto gps = get_or_create<decltype(_gps_modules), DroneCAN_GPS>(_gps_modules, t.source_node, _all_can_sensors);
            const float vn = m.ned_velocity[0], ve = m.ned_velocity[1];
            const float speed = std::sqrt(vn * vn + ve * ve);
            float course_deg = std::atan2(ve, vn) * 57.2957795f;
            if (course_deg < 0.0f) course_deg += 360.0f;
            gps->ingest_gnss_fix(m.lat_deg, m.lon_deg, m.height_msl_m, speed, course_deg, m.status, m.sats_used, now);
            gps->ingest_velocity_ned(math::Vector3f(m.ned_velocity[0], m.ned_velocity[1], m.ned_velocity[2]));
            break;
        }
        default:
            break;
    }
}

std::shared_ptr<DroneCAN_IMU> DroneCANNode::get_imu(uint8_t node_id) const {
    if (node_id == 0 && !_imus.empty()) return _imus.begin()->second;
    auto it = _imus.find(node_id);
    return it != _imus.end() ? it->second : nullptr;
}

std::shared_ptr<DroneCAN_Baro> DroneCANNode::get_baro(uint8_t node_id) const {
    if (node_id == 0 && !_baros.empty()) return _baros.begin()->second;
    auto it = _baros.find(node_id);
    return it != _baros.end() ? it->second : nullptr;
}

std::shared_ptr<DroneCAN_Compass> DroneCANNode::get_compass(uint8_t node_id) const {
    if (node_id == 0 && !_compasses.empty()) return _compasses.begin()->second;
    auto it = _compasses.find(node_id);
    return it != _compasses.end() ? it->second : nullptr;
}

std::shared_ptr<DroneCAN_GPS> DroneCANNode::get_gps(uint8_t node_id) const {
    if (node_id == 0 && !_gps_modules.empty()) return _gps_modules.begin()->second;
    auto it = _gps_modules.find(node_id);
    return it != _gps_modules.end() ? it->second : nullptr;
}

} // namespace dronecan
} // namespace drivers
} // namespace autopilot
