#pragma once

#include "hal/hal.hpp"
#include "vehicles/vehicle_base.hpp"
#include <string>

namespace autopilot {
namespace system {

class Telemetry {
public:
    Telemetry(hal::UARTDevice* uart, vehicles::VehicleBase* vehicle);

    void send_heartbeat();
    void send_attitude();
    void send_sensor_status();

private:
    hal::UARTDevice* _uart{nullptr};
    vehicles::VehicleBase* _vehicle{nullptr};

    void send_string(const std::string& msg);
};

} // namespace system
} // namespace autopilot
