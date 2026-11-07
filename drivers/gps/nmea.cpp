#include <cmath>
#include "nmea.hpp"
#include <sstream>
#include <vector>
#include <cstdlib>

namespace autopilot {
namespace drivers {

NMEAGPS::NMEAGPS(hal::UARTDevice* uart, hal::Time* time) : _uart(uart), _time(time) {}

bool NMEAGPS::init() {
    if (!_uart) {
        _health = SensorHealth::FAILED;
        return false;
    }
    _uart->init(38400); // Standard Pixhawk GPS baud rate (38400 or 115200)
    _health = SensorHealth::HEALTHY;
    return true;
}

static std::vector<std::string> split_csv(const std::string& str) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

double NMEAGPS::convert_nmea_coord(const std::string& coord_str, char dir) {
    if (coord_str.empty()) return 0.0;
    double raw = std::strtod(coord_str.c_str(), nullptr);
    int degrees = static_cast<int>(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

void NMEAGPS::parse_gga(const std::string& line) {
    auto tokens = split_csv(line);
    // $GPGGA,time,lat,N/S,lon,E/W,fix_quality,num_sats,hdop,alt,M,...
    if (tokens.size() > 9) {
        int fix_qual = std::atoi(tokens[6].c_str());
        _data.has_fix = (fix_qual > 0);
        _data.fix_type = (fix_qual > 0) ? 3 : 0;
        _data.satellites = std::atoi(tokens[7].c_str());

        if (tokens.size() > 3 && !tokens[2].empty() && !tokens[3].empty()) {
            _data.latitude_deg = convert_nmea_coord(tokens[2], tokens[3][0]);
        }
        if (tokens.size() > 5 && !tokens[4].empty() && !tokens[5].empty()) {
            _data.longitude_deg = convert_nmea_coord(tokens[4], tokens[5][0]);
        }
        if (tokens.size() > 9 && !tokens[9].empty()) {
            _data.altitude_msl_m = std::strtof(tokens[9].c_str(), nullptr);
        }
        if (_time) {
            _data.timestamp_ms = _time->millis();
        }
    }
}

void NMEAGPS::parse_rmc(const std::string& line) {
    auto tokens = split_csv(line);
    // $GPRMC,time,status,lat,N/S,lon,E/W,speed_knots,course_deg,...
    if (tokens.size() > 8) {
        char status = tokens[2].empty() ? 'V' : tokens[2][0];
        if (status == 'A') { // Valid
            _data.has_fix = true;
            if (!tokens[7].empty()) {
                float speed_knots = std::strtof(tokens[7].c_str(), nullptr);
                _data.speed_m_s = speed_knots * 0.514444f;
            }
            if (!tokens[8].empty()) {
                _data.ground_course_deg = std::strtof(tokens[8].c_str(), nullptr);
            }
            const float course = _data.ground_course_deg * 0.0174532925f;
            _data.velocity_ned = math::Vector3f(_data.speed_m_s * std::cos(course), _data.speed_m_s * std::sin(course), 0.0f);
            _data.has_velocity_ned = false; // horizontal only
        }
    }
}

// NMEA checksum: XOR of all characters between '$' and '*', sent as two hex digits.
static bool nmea_checksum_ok(const std::string& line) {
    const size_t star = line.find('*');
    if (line.empty() || line[0] != '$' || star == std::string::npos || star + 2 >= line.size() + 0 || star + 3 > line.size()) {
        return false;
    }
    uint8_t sum = 0;
    for (size_t i = 1; i < star; ++i) sum ^= static_cast<uint8_t>(line[i]);
    const unsigned long given = std::strtoul(line.substr(star + 1, 2).c_str(), nullptr, 16);
    return given == sum;
}

void NMEAGPS::parse_sentence(const std::string& line) {
    if (!nmea_checksum_ok(line)) {
        record_failure(); // corrupted sentence (noise on the UART): never trust its contents
        return;
    }
    if (line.rfind("$GPGGA", 0) == 0 || line.rfind("$GNGGA", 0) == 0) {
        parse_gga(line);
        if (_time) record_success(_time->millis());
    } else if (line.rfind("$GPRMC", 0) == 0 || line.rfind("$GNRMC", 0) == 0) {
        parse_rmc(line);
        if (_time) record_success(_time->millis());
    }
}

bool NMEAGPS::update() {
    if (!_uart || !_time) {
        record_failure();
        return false;
    }

    uint8_t buffer[64];
    size_t count = _uart->read(buffer, sizeof(buffer));
    for (size_t i = 0; i < count; ++i) {
        char c = static_cast<char>(buffer[i]);
        if (c == '\r' || c == '\n') {
            if (!_rx_line_buffer.empty()) {
                parse_sentence(_rx_line_buffer);
                _rx_line_buffer.clear();
            }
        } else {
            if (_rx_line_buffer.length() < 128) {
                _rx_line_buffer.push_back(c);
            }
        }
    }

    return true;
}

} // namespace drivers
} // namespace autopilot
