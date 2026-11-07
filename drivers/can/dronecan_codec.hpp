#pragma once
/**
 * DroneCAN (UAVCAN v0) serialization + transport, written to match the
 * reference implementation (libcanard) bit for bit:
 *
 *  - Bit packing: each scalar is laid out little-endian, its last partial byte
 *    is left-aligned, and the bits are streamed most-significant-bit first.
 *    (This is why 14-bit ESC commands can't simply be OR-ed into a uint64.)
 *  - Multi-frame transfers: the first frame starts with a 16-bit CRC
 *    (CRC-16-CCITT, seeded with the 64-bit data type signature), every frame
 *    ends with a tail byte (SOT/EOT/toggle/transfer-id), toggle starts at 0.
 *  - Tail array optimisation: a dynamic array that is the LAST field and has
 *    elements >= 8 bits carries no length prefix.
 *
 * Zero heap: the reassembler uses a fixed pool of buffers.
 */

#include "dronecan_types.hpp"
#include "hal/hal.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace autopilot {
namespace drivers {
namespace dronecan {

// ---------------- CRC ----------------
uint16_t crc16_add(uint16_t crc, const uint8_t* data, size_t len);
uint16_t crc16_add_signature(uint16_t crc, uint64_t signature);
uint16_t transfer_crc(uint64_t signature, const uint8_t* payload, size_t len);

// ---------------- bit-level codec ----------------
class BitWriter {
public:
    BitWriter(uint8_t* buf, size_t cap_bytes) : _buf(buf), _cap_bits(cap_bytes * 8) {}
    bool write_u(uint64_t value, uint8_t bit_len);
    bool write_i(int64_t value, uint8_t bit_len) { return write_u(static_cast<uint64_t>(value), bit_len); }
    bool write_f16(float v);
    bool write_f32(float v);
    size_t bit_pos() const { return _pos; }
    size_t byte_len() const { return (_pos + 7) / 8; }

private:
    uint8_t* _buf;
    size_t _cap_bits;
    size_t _pos{0};
};

class BitReader {
public:
    BitReader(const uint8_t* buf, size_t len_bytes) : _buf(buf), _len_bits(len_bytes * 8) {}
    bool read_u(uint8_t bit_len, uint64_t& out);
    bool read_i(uint8_t bit_len, int64_t& out);
    bool read_f16(float& out);
    bool read_f32(float& out);
    bool skip(size_t bits);
    size_t bit_pos() const { return _pos; }
    size_t remaining_bits() const { return _len_bits > _pos ? _len_bits - _pos : 0; }

private:
    const uint8_t* _buf;
    size_t _len_bits;
    size_t _pos{0};
};

// ---------------- message structs + codecs ----------------
struct NodeStatusMsg {
    uint32_t uptime_sec{0};
    uint8_t health{0};
    uint8_t mode{0};
    uint8_t sub_mode{0};
    uint16_t vendor_status{0};
};
struct RawIMUMsg {
    uint64_t timestamp_usec{0};
    float gyro[3]{};    // rad/s, body frame (FRD)
    float accel[3]{};   // m/s^2, specific force, body frame (FRD)
};
struct MagFieldMsg {
    uint8_t sensor_id{0};
    float field_ga[3]{};
};
struct StaticPressureMsg {
    float pressure_pa{0.0f};
    float variance{0.0f};
};
struct StaticTemperatureMsg {
    float temperature_k{0.0f};
    float variance{0.0f};
};
struct GnssFix2Msg {
    double lat_deg{0.0};
    double lon_deg{0.0};
    float height_msl_m{0.0f};
    float ned_velocity[3]{};
    uint8_t sats_used{0};
    uint8_t status{0};   // 0 none, 1 time only, 2 2D, 3 3D
};

size_t encode_node_status(const NodeStatusMsg& m, uint8_t* buf, size_t cap);
bool   decode_node_status(const uint8_t* p, size_t len, NodeStatusMsg& m);
size_t encode_raw_imu(const RawIMUMsg& m, uint8_t* buf, size_t cap);
bool   decode_raw_imu(const uint8_t* p, size_t len, RawIMUMsg& m);
size_t encode_mag_field(const MagFieldMsg& m, bool v2, uint8_t* buf, size_t cap);
bool   decode_mag_field(const uint8_t* p, size_t len, bool v2, MagFieldMsg& m);
size_t encode_static_pressure(const StaticPressureMsg& m, uint8_t* buf, size_t cap);
bool   decode_static_pressure(const uint8_t* p, size_t len, StaticPressureMsg& m);
size_t encode_static_temperature(const StaticTemperatureMsg& m, uint8_t* buf, size_t cap);
bool   decode_static_temperature(const uint8_t* p, size_t len, StaticTemperatureMsg& m);
size_t encode_gnss_fix2(const GnssFix2Msg& m, uint8_t* buf, size_t cap);
bool   decode_gnss_fix2(const uint8_t* p, size_t len, GnssFix2Msg& m);
/** RawCommand: int14[<=20], values in [-8192, 8191]. Uses tail-array optimisation. */
size_t encode_esc_raw_command(const int16_t* cmd, uint8_t count, uint8_t* buf, size_t cap);
bool   decode_esc_raw_command(const uint8_t* p, size_t len, int16_t* cmd, uint8_t& count);

// ---------------- transport ----------------
static constexpr size_t MAX_TRANSFER_PAYLOAD = 128;

/**
 * Splits a broadcast transfer into CAN frames. Returns number of frames written
 * (0 if they don't fit in out_frames).
 */
size_t make_broadcast_frames(uint8_t priority, uint16_t data_type_id, uint8_t source_node,
                             uint8_t transfer_id, const uint8_t* payload, size_t len,
                             hal::CANFrame* out_frames, size_t max_frames);

struct Transfer {
    uint16_t data_type_id{0};
    uint8_t source_node{0};
    uint8_t transfer_id{0};
    uint8_t priority{0};
    uint32_t timestamp_ms{0};
    const uint8_t* payload{nullptr};
    size_t len{0};
};

/** Reassembles multi-frame broadcast transfers and validates CRC/toggle/transfer-id. */
class Reassembler {
public:
    static constexpr size_t SLOTS = 12;
    static constexpr uint32_t TRANSFER_TIMEOUT_MS = 2000;

    /** Feed one frame. Returns true when a full, valid transfer is available in `out`. */
    bool accept(const hal::CANFrame& frame, uint32_t now_ms, Transfer& out);

    uint32_t crc_errors() const { return _crc_errors; }
    uint32_t sequence_errors() const { return _seq_errors; }

private:
    struct Slot {
        bool active{false};
        uint16_t dtid{0};
        uint8_t node{0};
        uint8_t transfer_id{0};
        bool next_toggle{false};
        uint16_t expected_crc{0};
        uint16_t crc{0};
        uint64_t signature{0};
        uint32_t started_ms{0};
        size_t len{0};
        uint8_t buf[MAX_TRANSFER_PAYLOAD]{};
    };
    std::array<Slot, SLOTS> _slots{};
    uint8_t _single_buf[7]{};
    uint32_t _crc_errors{0};
    uint32_t _seq_errors{0};

    Slot* find_slot(uint16_t dtid, uint8_t node, uint32_t now_ms, bool create);
};

} // namespace dronecan
} // namespace drivers
} // namespace autopilot
