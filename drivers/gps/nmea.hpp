#pragma once

#include "drivers/sensor_interface.hpp"
#include "hal/hal.hpp"
#include <cstddef>
#include <string>

namespace autopilot {
namespace drivers {

/**
 * NMEA 0183 GPS driver (GGA, RMC, GSA; any talker: GP/GN/GL/GA/GB).
 *
 * Zero heap allocation: sentences are assembled in a fixed buffer and split in
 * place (no std::string / std::vector / stringstream on the flight path, which
 * also keeps iostreams out of the firmware image).
 */
class NMEAGPS : public GPSSensor {
public:
    NMEAGPS(hal::UARTDevice* uart, hal::Time* time);

    const char* name() const override { return "NMEA-GPS"; }
    bool init() override;
    bool update() override;

    const GPSData& get_data() const override { return _data; }
    bool has_3d_fix() const override { return _data.has_fix && _data.fix_type >= 3; }

    // Direct injection for simulation / testing (one sentence, no CR/LF needed)
    void parse_sentence(const char* line);
    void parse_sentence(const std::string& line) { parse_sentence(line.c_str()); }

    /** Sentences rejected (bad checksum / malformed). */
    uint32_t bad_sentences() const { return _bad_sentences; }

    static constexpr size_t MAX_SENTENCE = 96; // NMEA limit is 82 incl. $ and CRLF
    static constexpr size_t MAX_FIELDS = 24;

private:
    hal::UARTDevice* _uart{nullptr};
    hal::Time* _time{nullptr};
    GPSData _data;

    char _rx[MAX_SENTENCE + 1]{};
    size_t _rx_len{0};
    bool _rx_overflow{false};
    bool _have_gsa{false};       // receiver reports 2D/3D explicitly
    uint8_t _gsa_fix_type{0};
    uint32_t _bad_sentences{0};

    void parse_gga(char* const* f, size_t n);
    void parse_rmc(char* const* f, size_t n);
    void parse_gsa(char* const* f, size_t n);

public:
    /** ddmm.mmmm + hemisphere -> signed decimal degrees (exposed for tests). */
    static double convert_nmea_coord(const char* coord, char dir);
    /** XOR checksum between '$' and '*' matches the two hex digits after '*'. */
    static bool checksum_ok(const char* line);
};

} // namespace drivers
} // namespace autopilot
