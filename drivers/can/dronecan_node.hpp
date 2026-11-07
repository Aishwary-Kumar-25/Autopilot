#pragma once

#include "hal/hal.hpp"
#include "dronecan_types.hpp"
#include "dronecan_sensors.hpp"
#include "dronecan_codec.hpp"
#include <unordered_map>
#include <memory>
#include <vector>

namespace autopilot {
namespace drivers {
namespace dronecan {

/**
 * @brief Autonomous DroneCAN / UAVCAN Node and Packet Dispatcher.
 * Listens to CANDevice, decodes standard DroneCAN frames, and manages
 * dynamic plug-and-play DroneCAN sensor drivers.
 */
class DroneCANNode {
public:
    DroneCANNode(hal::CANDevice* can, hal::Time* time, uint8_t local_node_id = 1);

    bool init(uint32_t bitrate_bps = 1000000);

    /**
     * @brief Process pending frames from the CAN controller FIFO.
     * Called in flight loop at high rate (e.g. 100Hz - 400Hz).
     */
    void process_rx();

    /**
     * @brief Broadcast local autopilot NodeStatus frame at 1 Hz.
     */
    void broadcast_node_status(NodeHealth health = NodeHealth::OK, NodeMode mode = NodeMode::OPERATIONAL);

    // Discovered sensor accessors
    std::shared_ptr<DroneCAN_IMU> get_imu(uint8_t node_id = 0) const;
    std::shared_ptr<DroneCAN_Baro> get_baro(uint8_t node_id = 0) const;
    std::shared_ptr<DroneCAN_Compass> get_compass(uint8_t node_id = 0) const;
    std::shared_ptr<DroneCAN_GPS> get_gps(uint8_t node_id = 0) const;

    const std::vector<std::shared_ptr<Sensor>>& get_discovered_sensors() const {
        return _all_can_sensors;
    }

    size_t get_active_node_count() const { return _active_nodes.size(); }
    const Reassembler& transport() const { return _rx; }
    uint32_t unknown_or_bad_payloads() const { return _bad_payloads; }

private:
    hal::CANDevice* _can{nullptr};
    hal::Time* _time{nullptr};
    uint8_t _local_node_id{1};
    uint8_t _status_transfer_id{0};
    uint32_t _last_status_tx_ms{0};

    struct DiscoveredNode {
        uint8_t node_id;
        NodeHealth health;
        NodeMode mode;
        uint32_t last_seen_ms;
    };

    std::unordered_map<uint8_t, DiscoveredNode> _active_nodes;
    std::unordered_map<uint8_t, std::shared_ptr<DroneCAN_IMU>> _imus;
    std::unordered_map<uint8_t, std::shared_ptr<DroneCAN_Baro>> _baros;
    std::unordered_map<uint8_t, std::shared_ptr<DroneCAN_Compass>> _compasses;
    std::unordered_map<uint8_t, std::shared_ptr<DroneCAN_GPS>> _gps_modules;
    std::vector<std::shared_ptr<Sensor>> _all_can_sensors;

    Reassembler _rx;
    uint32_t _bad_payloads{0};

    void handle_transfer(const Transfer& t);
};

} // namespace dronecan
} // namespace drivers
} // namespace autopilot

