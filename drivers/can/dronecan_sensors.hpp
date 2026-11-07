#pragma once

#include "drivers/sensor_interface.hpp"
#include "dronecan_types.hpp"
#include "math/vector3.hpp"
#include "math/low_pass_filter.hpp"

namespace autopilot {
namespace drivers {
namespace dronecan {

/**
 * @brief DroneCAN IMU Sensor implementation (uavcan.equipment.ahrs.RawIMU).
 */
class DroneCAN_IMU : public IMUSensor {
public:
    DroneCAN_IMU(uint8_t node_id);

    const char* name() const override { return "DroneCAN-IMU"; }
    bool init() override;
    bool update() override;

    const math::Vector3f& get_accel() const override { return _accel; }
    const math::Vector3f& get_gyro() const override { return _gyro; }
    float get_temperature() const override { return _temperature; }

    void calibrate_gyro_bias(uint16_t sample_count = 500) override;
    const math::Vector3f& get_gyro_bias() const override { return _gyro_bias; }

    uint8_t get_node_id() const { return _node_id; }
    void ingest_raw_imu(const math::Vector3f& rate_gyro, const math::Vector3f& accel, uint32_t timestamp_ms);

private:
    uint8_t _node_id{0};
    math::Vector3f _accel{0.0f, 0.0f, 9.80665f};
    math::Vector3f _gyro{0.0f, 0.0f, 0.0f};
    math::Vector3f _gyro_bias{0.0f, 0.0f, 0.0f};
    float _temperature{25.0f};
    uint32_t _last_packet_ms{0};
};

/**
 * @brief DroneCAN Barometer implementation (uavcan.equipment.air_data.StaticPressure).
 */
class DroneCAN_Baro : public BarometerSensor {
public:
    DroneCAN_Baro(uint8_t node_id);

    const char* name() const override { return "DroneCAN-Barometer"; }
    bool init() override;
    bool update() override;

    float get_pressure_pa() const override { return _pressure_pa; }
    float get_altitude_m() const override { return _altitude_m; }
    float get_temperature_c() const override { return _temperature_c; }

    void set_ground_reference(float ground_pressure_pa) override;
    uint8_t get_node_id() const { return _node_id; }

    void ingest_pressure(float pressure_pa, uint32_t timestamp_ms);
    void ingest_temperature(float temp_kelvin);

private:
    uint8_t _node_id{0};
    float _pressure_pa{101325.0f};
    float _ground_pressure_pa{101325.0f};
    float _altitude_m{0.0f};
    float _temperature_c{20.0f};
    uint32_t _last_packet_ms{0};
    math::LowPassFilter _alt_filter{10.0f, 50.0f};
};

/**
 * @brief DroneCAN Magnetometer / Compass implementation (uavcan.equipment.ahrs.MagneticFieldStrength).
 */
class DroneCAN_Compass : public CompassSensor {
public:
    DroneCAN_Compass(uint8_t node_id);

    const char* name() const override { return "DroneCAN-Compass"; }
    bool init() override;
    bool update() override;

    const math::Vector3f& get_field_gauss() const override { return _field_gauss; }
    float get_heading_rad() const override { return _heading_rad; }

    uint8_t get_node_id() const { return _node_id; }
    void ingest_mag_field(const math::Vector3f& field_gauss, uint32_t timestamp_ms);

private:
    uint8_t _node_id{0};
    math::Vector3f _field_gauss{0.0f, 0.0f, 0.0f};
    float _heading_rad{0.0f};
    uint32_t _last_packet_ms{0};
};

/**
 * @brief DroneCAN GNSS implementation (uavcan.equipment.gnss.Fix / Fix2).
 */
class DroneCAN_GPS : public GPSSensor {
public:
    DroneCAN_GPS(uint8_t node_id);

    const char* name() const override { return "DroneCAN-GPS"; }
    bool init() override;
    bool update() override;

    const GPSData& get_data() const override { return _data; }
    bool has_3d_fix() const override { return _data.has_fix && _data.fix_type >= 3; }

    uint8_t get_node_id() const { return _node_id; }
    void ingest_gnss_fix(double lat_deg, double lon_deg, float alt_m, float speed_m_s,
                         float heading_deg, uint8_t fix_type, uint8_t sats, uint32_t timestamp_ms);

private:
    uint8_t _node_id{0};
    GPSData _data;
    uint32_t _last_packet_ms{0};
};

} // namespace dronecan
} // namespace drivers
} // namespace autopilot

