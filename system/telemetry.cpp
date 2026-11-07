#include "telemetry.hpp"
#include <sstream>
#include <iomanip>

namespace autopilot {
namespace system {

Telemetry::Telemetry(hal::UARTDevice* uart, vehicles::VehicleBase* vehicle)
    : _uart(uart), _vehicle(vehicle) {}

void Telemetry::send_string(const std::string& msg) {
    if (_uart) {
        _uart->write(reinterpret_cast<const uint8_t*>(msg.c_str()), msg.length());
    }
}

void Telemetry::send_heartbeat() {
    if (!_vehicle) return;

    std::ostringstream ss;
    ss << "[HEARTBEAT] Type: " << _vehicle->get_type_name()
       << " | Armed: " << (_vehicle->is_armed() ? "YES" : "NO")
       << "\r\n";
    send_string(ss.str());
}

void Telemetry::send_attitude() {
    if (!_vehicle) return;
    const auto& state = _vehicle->get_state();

    float roll_deg = state.euler_rad.x * (180.0f / static_cast<float>(M_PI));
    float pitch_deg = state.euler_rad.y * (180.0f / static_cast<float>(M_PI));
    float yaw_deg = state.euler_rad.z * (180.0f / static_cast<float>(M_PI));

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "[ATTITUDE] R: " << roll_deg
       << " deg | P: " << pitch_deg
       << " deg | Y: " << yaw_deg
       << " deg | Alt: " << state.altitude_m << " m\r\n";
    send_string(ss.str());
}

void Telemetry::send_sensor_status() {
    if (!_vehicle) return;
    auto* sm = _vehicle->get_sensors();
    if (!sm) return;

    std::ostringstream ss;
    ss << "[SENSORS] IMU: " << (sm->has_healthy_imu() ? sm->get_active_imu()->name() : "NONE")
       << " | BARO: " << (sm->has_healthy_baro() ? sm->get_active_baro()->name() : "NONE")
       << " | MAG: " << (sm->has_healthy_compass() ? sm->get_active_compass()->name() : "NONE")
       << " | GPS: " << (sm->has_healthy_gps() ? "3D_FIX" : "NO_FIX")
       << "\r\n";
    send_string(ss.str());
}

} // namespace system
} // namespace autopilot
