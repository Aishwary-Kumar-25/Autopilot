#include "nmea.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace autopilot {
namespace drivers {

NMEAGPS::NMEAGPS(hal::UARTDevice* uart, hal::Time* time) : _uart(uart), _time(time) {}

bool NMEAGPS::init() {
    if (!_uart) {
        _health = SensorHealth::FAILED;
        return false;
    }
    _uart->init(38400); // Standard Pixhawk GPS baud rate (38400 or 115200)
    _rx_len = 0;
    _rx_overflow = false;
    _health = SensorHealth::HEALTHY;
    return true;
}

double NMEAGPS::convert_nmea_coord(const char* coord, char dir) {
    if (!coord || !*coord) return 0.0;
    const double raw = std::strtod(coord, nullptr);
    const int degrees = static_cast<int>(raw / 100.0);
    const double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool NMEAGPS::checksum_ok(const char* line) {
    if (!line || line[0] != '$') return false;
    uint8_t sum = 0;
    const char* p = line + 1;
    while (*p && *p != '*') sum ^= static_cast<uint8_t>(*p++);
    if (*p != '*') return false;
    const int hi = hex_val(p[1]);
    const int lo = hi < 0 ? -1 : hex_val(p[2]);
    if (hi < 0 || lo < 0) return false;
    return static_cast<uint8_t>((hi << 4) | lo) == sum;
}

// $GPGGA,time,lat,N/S,lon,E/W,fix_quality,num_sats,hdop,alt,M,...
void NMEAGPS::parse_gga(char* const* f, size_t n) {
    if (n < 10) return;
    const int fix_qual = std::atoi(f[6]);
    _data.satellites = static_cast<uint8_t>(std::atoi(f[7]));
    if (*f[8]) _data.hdop = std::strtof(f[8], nullptr);
    if (fix_qual > 0) {
        _data.has_fix = true;
        // GGA can't tell 2D from 3D; trust GSA when the receiver sends it.
        _data.fix_type = _have_gsa ? _gsa_fix_type : 3;
        if (*f[2] && *f[3]) _data.latitude_deg = convert_nmea_coord(f[2], f[3][0]);
        if (*f[4] && *f[5]) _data.longitude_deg = convert_nmea_coord(f[4], f[5][0]);
        if (*f[9]) _data.altitude_msl_m = std::strtof(f[9], nullptr);
    } else {
        _data.has_fix = false;
        _data.fix_type = 0;
    }
    if (_time) _data.timestamp_ms = _time->millis();
}

// $GPRMC,time,status,lat,N/S,lon,E/W,speed_knots,course_deg,...
void NMEAGPS::parse_rmc(char* const* f, size_t n) {
    if (n < 9) return;
    const char status = *f[2] ? f[2][0] : 'V';
    if (status != 'A') {
        // Receiver says the fix is invalid: don't keep flying on the last good one.
        _data.has_fix = false;
        _data.fix_type = 0;
        return;
    }
    _data.has_fix = true;
    if (*f[7]) _data.speed_m_s = std::strtof(f[7], nullptr) * 0.514444f;
    if (*f[8]) _data.ground_course_deg = std::strtof(f[8], nullptr);
    const float course = _data.ground_course_deg * 0.0174532925f;
    _data.velocity_ned = math::Vector3f(_data.speed_m_s * std::cos(course), _data.speed_m_s * std::sin(course), 0.0f);
    _data.has_velocity_ned = false; // horizontal only
}

// $GPGSA,mode,fix_type(1=none,2=2D,3=3D),sv1..sv12,pdop,hdop,vdop
void NMEAGPS::parse_gsa(char* const* f, size_t n) {
    if (n < 3) return;
    const int t = std::atoi(f[2]);
    _have_gsa = true;
    _gsa_fix_type = static_cast<uint8_t>(t >= 2 && t <= 3 ? t : 0);
    if (_data.has_fix) _data.fix_type = _gsa_fix_type;
    if (n > 16 && *f[16]) _data.hdop = std::strtof(f[16], nullptr);
}

void NMEAGPS::parse_sentence(const char* line) {
    if (!checksum_ok(line)) {
        ++_bad_sentences;
        record_failure(); // corrupted sentence (noise on the UART): never trust its contents
        return;
    }

    // Split a copy in place: ',' and '*' become '\0', fields point into buf.
    char buf[MAX_SENTENCE + 1];
    const size_t len = std::strlen(line);
    if (len > MAX_SENTENCE || len < 7) {
        ++_bad_sentences;
        return;
    }
    std::memcpy(buf, line, len + 1);
    char* fields[MAX_FIELDS];
    size_t n = 0;
    fields[n++] = buf;
    for (char* p = buf; *p; ++p) {
        if (*p == ',' || *p == '*') {
            const bool star = *p == '*';
            *p = '\0';
            if (star) break;
            if (n < MAX_FIELDS) fields[n++] = p + 1;
        }
    }

    // "$xxGGA": talker (2 chars) + sentence type (3 chars)
    const char* type = buf + 3;
    if (std::strncmp(type, "GGA", 3) == 0) {
        parse_gga(fields, n);
    } else if (std::strncmp(type, "RMC", 3) == 0) {
        parse_rmc(fields, n);
    } else if (std::strncmp(type, "GSA", 3) == 0) {
        parse_gsa(fields, n);
        return; // GSA alone carries no position: not a "sample"
    } else {
        return; // other sentences are fine but unused
    }
    if (_time) record_success(_time->millis());
}

bool NMEAGPS::update() {
    if (!_uart || !_time) {
        record_failure();
        return false;
    }

    uint8_t buffer[64];
    size_t count;
    // Drain everything that is waiting (a 38400 baud GPS sends ~4 bytes per 1 ms).
    while ((count = _uart->read(buffer, sizeof(buffer))) > 0) {
        for (size_t i = 0; i < count; ++i) {
            const char c = static_cast<char>(buffer[i]);
            if (c == '$') {
                // Start of sentence: resync. If the previous one lost its CR/LF, still
                // try it (the checksum decides whether it's complete).
                if (_rx_len > 0 && !_rx_overflow) {
                    _rx[_rx_len] = '\0';
                    parse_sentence(_rx);
                }
                _rx_len = 0;
                _rx_overflow = false;
            }
            if (c == '\r' || c == '\n') {
                if (_rx_len > 0 && !_rx_overflow) {
                    _rx[_rx_len] = '\0';
                    parse_sentence(_rx);
                }
                _rx_len = 0;
                _rx_overflow = false;
            } else if (_rx_len < MAX_SENTENCE) {
                _rx[_rx_len++] = c;
            } else {
                _rx_overflow = true; // too long: drop the whole sentence, not a truncated one
            }
        }
        if (count < sizeof(buffer)) break;
    }

    return true;
}

} // namespace drivers
} // namespace autopilot
