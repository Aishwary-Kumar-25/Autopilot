#include "dronecan_node.hpp"
#include <iostream>

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
    if (now_ms - _last_status_tx_ms < 1000) {
        return;
    }
    _last_status_tx_ms = now_ms;

    hal::CANFrame frame;
    frame.is_extended = true;
    frame.dlc = 8;
    // Priority 30 (low), Data Type 341, Local Node ID
    frame.id = CANIDFields::construct_message_id(30, MSG_ID_NODE_STATUS, _local_node_id);

    uint32_t uptime_sec = now_ms / 1000;
    frame.data[0] = static_cast<uint8_t>(uptime_sec & 0xFF);
    frame.data[1] = static_cast<uint8_t>((uptime_sec >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((uptime_sec >> 16) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((uptime_sec >> 24) & 0xFF);

    uint8_t health_mode = (static_cast<uint8_t>(health) << 6) | (static_cast<uint8_t>(mode) << 3);
    frame.data[4] = health_mode;
    frame.data[5] = 0x00; // Vendor specific status
    frame.data[6] = 0x00;

    // Single-frame transfer (SOT=1, EOT=1, Toggle=0)
    frame.data[7] = TailByte::construct(true, true, false, _status_transfer_id++);
    _can->send_frame(frame);
}

void DroneCANNode::process_rx() {
    if (!_can) return;

    hal::CANFrame frame;
    while (_can->receive_frame(frame)) {
        if (!frame.is_extended || frame.dlc < 1) {
            continue;
        }
        handle_frame(frame);
    }
}

void DroneCANNode::handle_frame(const hal::CANFrame& frame) {
    auto id_fields = CANIDFields::parse(frame.id);
    if (id_fields.is_service) {
        return; // Service requests not currently handled
    }

    uint8_t tail = frame.data[frame.dlc - 1];
    auto tail_byte = TailByte::parse(tail);
    (void)tail_byte;

    const uint8_t* payload = frame.data;
    uint8_t payload_len = frame.dlc - 1; // Subtract tail byte

    switch (id_fields.data_type_id) {
        case MSG_ID_NODE_STATUS:
            handle_node_status(id_fields.source_node_id, payload, payload_len);
            break;
        case MSG_ID_RAW_IMU:
            handle_raw_imu(id_fields.source_node_id, payload, payload_len);
            break;
        case MSG_ID_STATIC_PRESSURE:
            handle_static_pressure(id_fields.source_node_id, payload, payload_len);
            break;
        case MSG_ID_STATIC_TEMPERATURE:
            handle_static_temperature(id_fields.source_node_id, payload, payload_len);
            break;
        case MSG_ID_MAGNETIC_FIELD:
            handle_magnetic_field(id_fields.source_node_id, payload, payload_len);
            break;
        case MSG_ID_GNSS_FIX:
        case MSG_ID_GNSS_FIX2:
            handle_gnss_fix(id_fields.source_node_id, payload, payload_len);
            break;
        default:
            break;
    }
}

void DroneCANNode::handle_node_status(uint8_t source_node, const uint8_t* payload, uint8_t len) {
    if (len < 5 || !_time) return;

    uint8_t health_code = (payload[4] >> 6) & 0x03;
    uint8_t mode_code = (payload[4] >> 3) & 0x07;

    DiscoveredNode node;
    node.node_id = source_node;
    node.health = static_cast<NodeHealth>(health_code);
    node.mode = static_cast<NodeMode>(mode_code);
    node.last_seen_ms = _time->millis();
    _active_nodes[source_node] = node;
}

void DroneCANNode::handle_raw_imu(uint8_t source_node, const uint8_t* payload, uint8_t len) {
    // Expect 6x float16 (12 bytes) or compact 3x float16 (6 bytes)
    if (len < 6 || !_time) return;

    auto it = _imus.find(source_node);
    if (it == _imus.end()) {
        auto imu = std::make_shared<DroneCAN_IMU>(source_node);
        imu->init();
        _imus[source_node] = imu;
        _all_can_sensors.push_back(imu);
        it = _imus.find(source_node);
    }

    uint16_t raw_gx = static_cast<uint16_t>(payload[0] | (payload[1] << 8));
    uint16_t raw_gy = static_cast<uint16_t>(payload[2] | (payload[3] << 8));
    uint16_t raw_gz = static_cast<uint16_t>(payload[4] | (payload[5] << 8));

    math::Vector3f gyro(
        float16_to_float32(raw_gx),
        float16_to_float32(raw_gy),
        float16_to_float32(raw_gz)
    );

    math::Vector3f accel(0.0f, 0.0f, 9.80665f);
    if (len >= 12) {
        uint16_t raw_ax = static_cast<uint16_t>(payload[6] | (payload[7] << 8));
        uint16_t raw_ay = static_cast<uint16_t>(payload[8] | (payload[9] << 8));
        uint16_t raw_az = static_cast<uint16_t>(payload[10] | (payload[11] << 8));
        accel.x = float16_to_float32(raw_ax);
        accel.y = float16_to_float32(raw_ay);
        accel.z = float16_to_float32(raw_az);
    }

    it->second->ingest_raw_imu(gyro, accel, _time->millis());
}

void DroneCANNode::handle_static_pressure(uint8_t source_node, const uint8_t* payload, uint8_t len) {
    if (len < 4 || !_time) return;

    auto it = _baros.find(source_node);
    if (it == _baros.end()) {
        auto baro = std::make_shared<DroneCAN_Baro>(source_node);
        baro->init();
        _baros[source_node] = baro;
        _all_can_sensors.push_back(baro);
        it = _baros.find(source_node);
    }

    float pressure_pa;
    std::memcpy(&pressure_pa, payload, sizeof(float));

    it->second->ingest_pressure(pressure_pa, _time->millis());
}

void DroneCANNode::handle_static_temperature(uint8_t source_node, const uint8_t* payload, uint8_t len) {
    if (len < 2) return;

    auto it = _baros.find(source_node);
    if (it != _baros.end()) {
        uint16_t raw_temp = static_cast<uint16_t>(payload[0] | (payload[1] << 8));
        float temp_kelvin = float16_to_float32(raw_temp);
        it->second->ingest_temperature(temp_kelvin);
    }
}

void DroneCANNode::handle_magnetic_field(uint8_t source_node, const uint8_t* payload, uint8_t len) {
    if (len < 7 || !_time) return; // 1 byte sensor_id + 6 bytes (3x float16)

    auto it = _compasses.find(source_node);
    if (it == _compasses.end()) {
        auto compass = std::make_shared<DroneCAN_Compass>(source_node);
        compass->init();
        _compasses[source_node] = compass;
        _all_can_sensors.push_back(compass);
        it = _compasses.find(source_node);
    }

    uint16_t raw_mx = static_cast<uint16_t>(payload[1] | (payload[2] << 8));
    uint16_t raw_my = static_cast<uint16_t>(payload[3] | (payload[4] << 8));
    uint16_t raw_mz = static_cast<uint16_t>(payload[5] | (payload[6] << 8));

    math::Vector3f field(
        float16_to_float32(raw_mx),
        float16_to_float32(raw_my),
        float16_to_float32(raw_mz)
    );

    it->second->ingest_mag_field(field, _time->millis());
}

void DroneCANNode::handle_gnss_fix(uint8_t source_node, const uint8_t* payload, uint8_t len) {
    if (len < 12 || !_time) return;

    auto it = _gps_modules.find(source_node);
    if (it == _gps_modules.end()) {
        auto gps = std::make_shared<DroneCAN_GPS>(source_node);
        gps->init();
        _gps_modules[source_node] = gps;
        _all_can_sensors.push_back(gps);
        it = _gps_modules.find(source_node);
    }

    // Latitude & Longitude in 1e-7 degrees (int32 each)
    int32_t raw_lat, raw_lon;
    std::memcpy(&raw_lat, payload, sizeof(int32_t));
    std::memcpy(&raw_lon, payload + 4, sizeof(int32_t));
    double lat_deg = raw_lat * 1e-7;
    double lon_deg = raw_lon * 1e-7;

    float alt_m = 0.0f;
    if (len >= 16) {
        std::memcpy(&alt_m, payload + 8, sizeof(float));
    }

    it->second->ingest_gnss_fix(lat_deg, lon_deg, alt_m, 0.0f, 0.0f, 3, 12, _time->millis());
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

