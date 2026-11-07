#pragma once

#include "sensor_interface.hpp"
#include "hal/hal.hpp"
#include <vector>
#include <memory>
#include <functional>

namespace autopilot {
namespace drivers {

namespace dronecan {
class DroneCANNode;
}

/**
 * Owns all sensors, judges their health and picks the primary of each type.
 *
 * The manager does NOT trust a driver's own "healthy" flag alone. Per sensor it checks:
 *   - timeout     : no new sample for longer than the type's timeout (unplugged CAN node, dead bus)
 *   - stuck       : bit-identical IMU/compass samples many times in a row (frozen chip)
 *   - range       : NaN / physically impossible values
 *   - consistency : with 2+ IMUs, gyro disagreement vs the nearest other IMU (a single
 *                   bad IMU out of three is out-voted)
 * Failover: the primary is replaced when it FAILS, or when it is DEGRADED and an
 * OK alternative exists. While ARMED, a sensor that failed stays failed (latched):
 * a flapping sensor must not come back mid-flight. Re-probing/hot-swap of failed
 * sensors only happens while disarmed; brand-new sensors can still be added.
 */
class SensorManager {
public:
    static constexpr uint32_t IMU_TIMEOUT_MS = 50;
    static constexpr uint32_t BARO_TIMEOUT_MS = 500;
    static constexpr uint32_t COMPASS_TIMEOUT_MS = 500;
    static constexpr uint32_t GPS_TIMEOUT_MS = 1500;
    static constexpr uint32_t STUCK_SAMPLES = 50;
    static constexpr float GYRO_CONSISTENCY_RAD_S = 0.35f;

    SensorManager(hal::HAL* hal);
    ~SensorManager();

    /**
     * @brief Scan available buses (I2C/SPI) and auto-discover plugged-in sensors.
     */
    void auto_discover_sensors();

    /**
     * @brief Register an externally provided sensor instance.
     */
    bool register_sensor(std::shared_ptr<Sensor> sensor);

    /**
     * @brief Main periodic update called from flight loop (e.g. 100Hz - 400Hz).
     * Updates active sensors and checks health.
     */
    void update();

    /**
     * @brief Low-rate health check & hot-swap probe called at 1-2 Hz.
     * Detects if bad sensor was replaced and re-binds it.
     */
    void probe_and_recover_sensors();

    // Accessors for active primary sensors
    IMUSensor* get_active_imu() const { return _active_imu; }
    BarometerSensor* get_active_baro() const { return _active_baro; }
    CompassSensor* get_active_compass() const { return _active_compass; }
    GPSSensor* get_active_gps() const { return _active_gps; }

    bool has_healthy_imu() const;
    bool has_healthy_baro() const;
    bool has_healthy_compass() const;
    bool has_healthy_gps() const;

    /** Manager's verdict for a sensor (combines driver flag + monitor checks). */
    SensorHealth verdict(const Sensor* s) const;
    bool is_usable(const Sensor* s) const { return verdict(s) == SensorHealth::HEALTHY; }

    /** Tell the manager whether the vehicle is armed (enables failure latching). */
    void set_armed(bool armed) { _armed = armed; }

    uint32_t failover_count() const { return _failovers; }

    // Replacement & diagnostic callbacks
    using SensorEventCallback = std::function<void(const char* sensor_name, SensorHealth new_health)>;
    void set_event_callback(SensorEventCallback cb) { _event_callback = cb; }

    const std::vector<std::shared_ptr<Sensor>>& get_all_sensors() const { return _all_sensors; }
    dronecan::DroneCANNode* get_dronecan() const { return _dronecan_node.get(); }

private:
    hal::HAL* _hal{nullptr};
    std::unique_ptr<dronecan::DroneCANNode> _dronecan_node;
    std::vector<std::shared_ptr<Sensor>> _all_sensors;

    IMUSensor* _active_imu{nullptr};
    BarometerSensor* _active_baro{nullptr};
    CompassSensor* _active_compass{nullptr};
    GPSSensor* _active_gps{nullptr};

    SensorEventCallback _event_callback{nullptr};
    uint32_t _last_probe_time_ms{0};
    bool _armed{false};
    std::vector<std::pair<uint8_t, Sensor*>> _bound_i2c; // I2C address -> driver object using it
    bool i2c_addr_free_for_new_driver(uint8_t addr);
    void remove_sensor(Sensor* s);
    uint32_t _failovers{0};

    struct Monitor {
        Sensor* sensor{nullptr};
        SensorHealth verdict{SensorHealth::UNINITIALIZED};
        uint32_t last_sample_ms{0};     // manager clock when the last new sample arrived
        uint32_t last_sample_count{0};
        bool seen_sample{false};
        uint32_t stuck_run{0};
        math::Vector3f last_a{0, 0, 0};
        math::Vector3f last_b{0, 0, 0};
        float consistency_err{0.0f};
        bool latched_failed{false};
    };
    std::vector<Monitor> _monitors;

    Monitor* monitor_for(const Sensor* s);
    const Monitor* monitor_for(const Sensor* s) const;
    void evaluate(Monitor& m, uint32_t now_ms);
    void update_consistency();
    void check_health_and_failover();
    void notify_event(const char* name, SensorHealth health);
    template <typename T> void select_primary(T*& active, SensorType type);
};

} // namespace drivers
} // namespace autopilot
