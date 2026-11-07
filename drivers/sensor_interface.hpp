#pragma once

#include "math/vector3.hpp"
#include <cstdint>
#include <string>

namespace autopilot {
namespace drivers {

enum class SensorType {
    UNKNOWN = 0,
    IMU,
    BAROMETER,
    MAGNETOMETER,
    GPS,
    DISTANCE
};

enum class SensorHealth {
    UNINITIALIZED = 0,
    HEALTHY,
    DEGRADED,
    FAILED,
    DISCONNECTED
};

/**
 * @brief Base class for any plug-and-play sensor module.
 */
class Sensor {
public:
    virtual ~Sensor() = default;

    virtual SensorType type() const = 0;
    virtual const char* name() const = 0;

    /**
     * @brief Attempt initialization or re-initialization (e.g. after hot swap).
     */
    virtual bool init() = 0;

    /**
     * @brief Periodic data read / sample update.
     */
    virtual bool update() = 0;

    virtual SensorHealth health() const { return _health; }
    virtual bool is_healthy() const { return _health == SensorHealth::HEALTHY; }
    virtual uint32_t last_update_ms() const { return _last_update_ms; }
    virtual uint32_t error_count() const { return _error_count; }

    virtual void set_health(SensorHealth h) { _health = h; }

protected:
    SensorHealth _health{SensorHealth::UNINITIALIZED};
    uint32_t _last_update_ms{0};
    uint32_t _error_count{0};
    uint32_t _consecutive_errors{0};
    static constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 5;

    void record_success(uint32_t current_time_ms) {
        _last_update_ms = current_time_ms;
        _consecutive_errors = 0;
        _health = SensorHealth::HEALTHY;
    }

    void record_failure() {
        _error_count++;
        _consecutive_errors++;
        if (_consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            _health = SensorHealth::FAILED;
        } else {
            _health = SensorHealth::DEGRADED;
        }
    }
};

/**
 * @brief IMU Sensor (Accelerometer in m/s^2, Gyroscope in rad/s).
 */
class IMUSensor : public Sensor {
public:
    SensorType type() const override { return SensorType::IMU; }

    virtual const math::Vector3f& get_accel() const = 0; // m/s^2 (X: forward, Y: right, Z: down)
    virtual const math::Vector3f& get_gyro() const = 0;  // rad/s (Roll rate, Pitch rate, Yaw rate)
    virtual float get_temperature() const = 0;           // Celsius

    virtual void calibrate_gyro_bias(uint16_t sample_count = 500) = 0;
    virtual const math::Vector3f& get_gyro_bias() const = 0;
};

/**
 * @brief Barometer Sensor (Pressure in Pa, Altitude in meters relative to sea level / ground).
 */
class BarometerSensor : public Sensor {
public:
    SensorType type() const override { return SensorType::BAROMETER; }

    virtual float get_pressure_pa() const = 0;
    virtual float get_altitude_m() const = 0;
    virtual float get_temperature_c() const = 0;
    virtual void set_ground_reference(float ground_pressure_pa) = 0;
};

/**
 * @brief 3-Axis Magnetometer / Digital Compass (in Gauss).
 */
class CompassSensor : public Sensor {
public:
    SensorType type() const override { return SensorType::MAGNETOMETER; }

    virtual const math::Vector3f& get_field_gauss() const = 0;
    virtual float get_heading_rad() const = 0;
};

/**
 * @brief GPS Data Structure and Sensor Interface.
 */
struct GPSData {
    bool has_fix{false};
    uint8_t fix_type{0};        // 0=No fix, 2=2D, 3=3D
    uint8_t satellites{0};
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    float altitude_msl_m{0.0f};
    float speed_m_s{0.0f};
    float ground_course_deg{0.0f};
    uint32_t timestamp_ms{0};
};

class GPSSensor : public Sensor {
public:
    SensorType type() const override { return SensorType::GPS; }

    virtual const GPSData& get_data() const = 0;
    virtual bool has_3d_fix() const = 0;
};

} // namespace drivers
} // namespace autopilot
