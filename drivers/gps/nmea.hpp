#pragma once

#include "drivers/sensor_interface.hpp"
#include "hal/hal.hpp"
#include <string>

namespace autopilot {
namespace drivers {

class NMEAGPS : public GPSSensor {
public:
    NMEAGPS(hal::UARTDevice* uart, hal::Time* time);

    const char* name() const override { return "NMEA-GPS"; }
    bool init() override;
    bool update() override;

    const GPSData& get_data() const override { return _data; }
    bool has_3d_fix() const override { return _data.has_fix && _data.fix_type >= 3; }

    // Direct injection for simulation / testing
    void parse_sentence(const std::string& line);

private:
    hal::UARTDevice* _uart{nullptr};
    hal::Time* _time{nullptr};
    GPSData _data;
    std::string _rx_line_buffer;

    void parse_gga(const std::string& line);
    void parse_rmc(const std::string& line);
    static double convert_nmea_coord(const std::string& coord_str, char dir);
};

} // namespace drivers
} // namespace autopilot
