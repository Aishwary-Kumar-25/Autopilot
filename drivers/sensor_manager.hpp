#pragma once

#include "sensor_interface.hpp"
#include "hal/hal.hpp"
#include <vector>
#include <memory>
#include <functional>

namespace autopilot {
namespace drivers {

class SensorManager {
public:
    static constexpr size_t MAX_IMUS = 2;
    static constexpr size_t MAX_BAROS = 2;
    static constexpr size_t MAX_COMPASSES = 2;

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

    // Replacement & diagnostic callbacks
    using SensorEventCallback = std::function<void(const char* sensor_name, SensorHealth new_health)>;
    void set_event_callback(SensorEventCallback cb) { _event_callback = cb; }

    const std::vector<std::shared_ptr<Sensor>>& get_all_sensors() const { return _all_sensors; }

private:
    hal::HAL* _hal{nullptr};
    std::vector<std::shared_ptr<Sensor>> _all_sensors;

    IMUSensor* _active_imu{nullptr};
    BarometerSensor* _active_baro{nullptr};
    CompassSensor* _active_compass{nullptr};
    GPSSensor* _active_gps{nullptr};

    SensorEventCallback _event_callback{nullptr};
    uint32_t _last_probe_time_ms{0};

    void check_health_and_failover();
    void notify_event(const char* name, SensorHealth health);
};

} // namespace drivers
} // namespace autopilot
