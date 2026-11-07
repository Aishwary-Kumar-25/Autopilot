#include "dronecan_codec.hpp"
#include <cmath>
#include <cstring>

namespace autopilot {
namespace drivers {
namespace dronecan {

// ---------------- CRC-16-CCITT (poly 0x1021, init 0xFFFF) ----------------

static uint16_t crc16_add_byte(uint16_t crc, uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

uint16_t crc16_add(uint16_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) crc = crc16_add_byte(crc, data[i]);
    return crc;
}

uint16_t crc16_add_signature(uint16_t crc, uint64_t signature) {
    for (int shift = 0; shift < 64; shift += 8) {
        crc = crc16_add_byte(crc, static_cast<uint8_t>(signature >> shift));
    }
    return crc;
}

uint16_t transfer_crc(uint64_t signature, const uint8_t* payload, size_t len) {
    return crc16_add(crc16_add_signature(0xFFFFU, signature), payload, len);
}

// ---------------- float16 with round-to-nearest ----------------

static uint16_t f32_to_f16(float value) {
    uint32_t f;
    std::memcpy(&f, &value, 4);
    const uint32_t sign = (f >> 16) & 0x8000U;
    const uint32_t abs = f & 0x7FFFFFFFU;
    if (abs >= 0x7F800000U) {  // inf / nan
        return static_cast<uint16_t>(sign | 0x7C00U | (abs > 0x7F800000U ? 0x200U : 0U));
    }
    if (abs >= 0x477FF000U) {  // overflow (>= 65520) -> inf
        return static_cast<uint16_t>(sign | 0x7C00U);
    }
    if (abs < 0x38800000U) {   // subnormal or zero in half precision
        float a;
        uint32_t a_bits = abs;
        std::memcpy(&a, &a_bits, 4);
        const uint32_t h = static_cast<uint32_t>(std::lround(a * 16777216.0f)); // a / 2^-24
        return static_cast<uint16_t>(sign | h);
    }
    // normal: rebias exponent, round mantissa to nearest even
    uint32_t h = ((abs >> 13) - (112U << 10));
    const uint32_t rem = abs & 0x1FFFU;
    if (rem > 0x1000U || (rem == 0x1000U && (h & 1U))) h++;
    return static_cast<uint16_t>(sign | h);
}

// ---------------- bit codec (libcanard-compatible) ----------------

bool BitWriter::write_u(uint64_t value, uint8_t bit_len) {
    if (bit_len == 0 || bit_len > 64 || _pos + bit_len > _cap_bits) return false;
    if (bit_len < 64) value &= (1ULL << bit_len) - 1ULL;
    uint8_t storage[8];
    for (int i = 0; i < 8; ++i) storage[i] = static_cast<uint8_t>(value >> (8 * i)); // little-endian
    if (bit_len % 8U) {
        // left-align the bits of the last partial byte
        storage[bit_len / 8U] = static_cast<uint8_t>(storage[bit_len / 8U] << (8U - bit_len % 8U));
    }
    for (uint8_t i = 0; i < bit_len; ++i) {
        const uint8_t bit = (storage[i / 8U] >> (7U - i % 8U)) & 1U;
        const size_t p = _pos + i;
        if (bit) _buf[p / 8U] |= static_cast<uint8_t>(0x80U >> (p % 8U));
        else     _buf[p / 8U] &= static_cast<uint8_t>(~(0x80U >> (p % 8U)));
    }
    _pos += bit_len;
    return true;
}

bool BitWriter::write_f16(float v) { return write_u(f32_to_f16(v), 16); }

bool BitWriter::write_f32(float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    return write_u(u, 32);
}

bool BitReader::read_u(uint8_t bit_len, uint64_t& out) {
    if (bit_len == 0 || bit_len > 64 || _pos + bit_len > _len_bits) return false;
    uint8_t storage[8] = {0};
    for (uint8_t i = 0; i < bit_len; ++i) {
        const size_t p = _pos + i;
        const uint8_t bit = (_buf[p / 8U] >> (7U - p % 8U)) & 1U;
        if (bit) storage[i / 8U] |= static_cast<uint8_t>(0x80U >> (i % 8U));
    }
    if (bit_len % 8U) {
        storage[bit_len / 8U] = static_cast<uint8_t>(storage[bit_len / 8U] >> (8U - bit_len % 8U));
    }
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(storage[i]) << (8 * i);
    out = v;
    _pos += bit_len;
    return true;
}

bool BitReader::read_i(uint8_t bit_len, int64_t& out) {
    uint64_t u;
    if (!read_u(bit_len, u)) return false;
    if (bit_len < 64 && (u & (1ULL << (bit_len - 1)))) u |= ~((1ULL << bit_len) - 1ULL); // sign-extend
    out = static_cast<int64_t>(u);
    return true;
}

bool BitReader::read_f16(float& out) {
    uint64_t u;
    if (!read_u(16, u)) return false;
    out = float16_to_float32(static_cast<uint16_t>(u));
    return true;
}

bool BitReader::read_f32(float& out) {
    uint64_t u;
    if (!read_u(32, u)) return false;
    const uint32_t v = static_cast<uint32_t>(u);
    std::memcpy(&out, &v, 4);
    return true;
}

bool BitReader::skip(size_t bits) {
    if (_pos + bits > _len_bits) return false;
    _pos += bits;
    return true;
}

// ---------------- messages ----------------

size_t encode_node_status(const NodeStatusMsg& m, uint8_t* buf, size_t cap) {
    BitWriter w(buf, cap);
    bool ok = w.write_u(m.uptime_sec, 32) && w.write_u(m.health, 2) && w.write_u(m.mode, 3) &&
              w.write_u(m.sub_mode, 3) && w.write_u(m.vendor_status, 16);
    return ok ? w.byte_len() : 0;
}

bool decode_node_status(const uint8_t* p, size_t len, NodeStatusMsg& m) {
    BitReader r(p, len);
    uint64_t up, h, mo, sm, vs;
    if (!(r.read_u(32, up) && r.read_u(2, h) && r.read_u(3, mo) && r.read_u(3, sm) && r.read_u(16, vs))) return false;
    m.uptime_sec = static_cast<uint32_t>(up);
    m.health = static_cast<uint8_t>(h);
    m.mode = static_cast<uint8_t>(mo);
    m.sub_mode = static_cast<uint8_t>(sm);
    m.vendor_status = static_cast<uint16_t>(vs);
    return true;
}

// RawIMU: timestamp(56) integration_interval(f32) gyro_latest(f16[3]) gyro_integral(f32[3])
//         accel_latest(f16[3]) accel_integral(f32[3]) covariance(f16[<=36], tail-array, no prefix)
size_t encode_raw_imu(const RawIMUMsg& m, uint8_t* buf, size_t cap) {
    BitWriter w(buf, cap);
    bool ok = w.write_u(m.timestamp_usec, 56) && w.write_f32(0.0f);
    for (int i = 0; i < 3 && ok; ++i) ok = w.write_f16(m.gyro[i]);
    for (int i = 0; i < 3 && ok; ++i) ok = w.write_f32(0.0f);
    for (int i = 0; i < 3 && ok; ++i) ok = w.write_f16(m.accel[i]);
    for (int i = 0; i < 3 && ok; ++i) ok = w.write_f32(0.0f);
    return ok ? w.byte_len() : 0;
}

bool decode_raw_imu(const uint8_t* p, size_t len, RawIMUMsg& m) {
    BitReader r(p, len);
    uint64_t ts;
    float dummy;
    if (!r.read_u(56, ts) || !r.read_f32(dummy)) return false;
    for (int i = 0; i < 3; ++i) if (!r.read_f16(m.gyro[i])) return false;
    if (!r.skip(3 * 32)) return false;
    for (int i = 0; i < 3; ++i) if (!r.read_f16(m.accel[i])) return false;
    if (!r.skip(3 * 32)) return false;
    m.timestamp_usec = ts;
    return true;
}

size_t encode_mag_field(const MagFieldMsg& m, bool v2, uint8_t* buf, size_t cap) {
    BitWriter w(buf, cap);
    bool ok = v2 ? w.write_u(m.sensor_id, 8) : true;
    for (int i = 0; i < 3 && ok; ++i) ok = w.write_f16(m.field_ga[i]);
    return ok ? w.byte_len() : 0;
}

bool decode_mag_field(const uint8_t* p, size_t len, bool v2, MagFieldMsg& m) {
    BitReader r(p, len);
    m.sensor_id = 0;
    if (v2) {
        uint64_t id;
        if (!r.read_u(8, id)) return false;
        m.sensor_id = static_cast<uint8_t>(id);
    }
    for (int i = 0; i < 3; ++i) if (!r.read_f16(m.field_ga[i])) return false;
    return true;
}

size_t encode_static_pressure(const StaticPressureMsg& m, uint8_t* buf, size_t cap) {
    BitWriter w(buf, cap);
    return (w.write_f32(m.pressure_pa) && w.write_f16(m.variance)) ? w.byte_len() : 0;
}

bool decode_static_pressure(const uint8_t* p, size_t len, StaticPressureMsg& m) {
    BitReader r(p, len);
    return r.read_f32(m.pressure_pa) && r.read_f16(m.variance);
}

size_t encode_static_temperature(const StaticTemperatureMsg& m, uint8_t* buf, size_t cap) {
    BitWriter w(buf, cap);
    return (w.write_f16(m.temperature_k) && w.write_f16(m.variance)) ? w.byte_len() : 0;
}

bool decode_static_temperature(const uint8_t* p, size_t len, StaticTemperatureMsg& m) {
    BitReader r(p, len);
    return r.read_f16(m.temperature_k) && r.read_f16(m.variance);
}

// Fix2 (fields we use come first; everything after `status` is skipped on decode)
size_t encode_gnss_fix2(const GnssFix2Msg& m, uint8_t* buf, size_t cap) {
    BitWriter w(buf, cap);
    const int64_t lon = static_cast<int64_t>(std::llround(m.lon_deg * 1e8));
    const int64_t lat = static_cast<int64_t>(std::llround(m.lat_deg * 1e8));
    const int64_t h_mm = static_cast<int64_t>(std::llround(static_cast<double>(m.height_msl_m) * 1000.0));
    bool ok = w.write_u(0, 56) && w.write_u(0, 56) &&      // timestamp, gnss_timestamp
              w.write_u(0, 3) && w.write_u(0, 13) &&       // time standard, void13
              w.write_u(0, 8) &&                           // num_leap_seconds
              w.write_i(lon, 37) && w.write_i(lat, 37) &&
              w.write_i(h_mm, 27) && w.write_i(h_mm, 27);  // height ellipsoid, msl
    for (int i = 0; i < 3 && ok; ++i) ok = w.write_f32(m.ned_velocity[i]);
    ok = ok && w.write_u(m.sats_used, 6) && w.write_u(m.status, 2) &&
         w.write_u(0, 4) && w.write_u(0, 6) &&             // mode, sub_mode
         w.write_u(0, 6) &&                                // covariance length (not last field -> has prefix)
         w.write_f16(0.0f);                                // pdop
    // ecef_position_velocity[<=1]: last field, tail-array optimised, zero elements
    return ok ? w.byte_len() : 0;
}

bool decode_gnss_fix2(const uint8_t* p, size_t len, GnssFix2Msg& m) {
    BitReader r(p, len);
    int64_t lon, lat, h_ell, h_msl;
    uint64_t sats, status;
    if (!r.skip(56 + 56 + 3 + 13 + 8)) return false;
    if (!(r.read_i(37, lon) && r.read_i(37, lat) && r.read_i(27, h_ell) && r.read_i(27, h_msl))) return false;
    for (int i = 0; i < 3; ++i) if (!r.read_f32(m.ned_velocity[i])) return false;
    if (!(r.read_u(6, sats) && r.read_u(2, status))) return false;
    m.lon_deg = static_cast<double>(lon) * 1e-8;
    m.lat_deg = static_cast<double>(lat) * 1e-8;
    m.height_msl_m = static_cast<float>(static_cast<double>(h_msl) * 1e-3);
    m.sats_used = static_cast<uint8_t>(sats);
    m.status = static_cast<uint8_t>(status);
    return true;
}

size_t encode_esc_raw_command(const int16_t* cmd, uint8_t count, uint8_t* buf, size_t cap) {
    if (count > 20) return 0;
    BitWriter w(buf, cap);
    for (uint8_t i = 0; i < count; ++i) {
        int16_t v = cmd[i] < -8192 ? -8192 : (cmd[i] > 8191 ? 8191 : cmd[i]);
        if (!w.write_i(v, 14)) return 0;
    }
    return w.byte_len();
}

bool decode_esc_raw_command(const uint8_t* p, size_t len, int16_t* cmd, uint8_t& count) {
    BitReader r(p, len);
    count = 0;
    while (r.remaining_bits() >= 14 && count < 20) {
        int64_t v = 0;
        if (!r.read_i(14, v)) break;
        cmd[count++] = static_cast<int16_t>(v);
    }
    return true;
}

// ---------------- transport ----------------

size_t make_broadcast_frames(uint8_t priority, uint16_t dtid, uint8_t node, uint8_t tid,
                             const uint8_t* payload, size_t len, hal::CANFrame* out, size_t max_frames) {
    const uint32_t can_id = CANIDFields::construct_message_id(priority, dtid, node);
    tid &= 0x1F;
    if (len <= 7) {
        if (max_frames < 1) return 0;
        out[0] = hal::CANFrame{};
        out[0].id = can_id;
        out[0].is_extended = true;
        std::memcpy(out[0].data, payload, len);
        out[0].data[len] = TailByte::construct(true, true, false, tid);
        out[0].dlc = static_cast<uint8_t>(len + 1);
        return 1;
    }
    const uint64_t sig = signature_for(dtid);
    const uint16_t crc = transfer_crc(sig, payload, len);
    size_t n = 0, off = 0;
    bool toggle = false;
    while (off < len) {
        if (n >= max_frames) return 0;
        hal::CANFrame& f = out[n];
        f = hal::CANFrame{};
        f.id = can_id;
        f.is_extended = true;
        size_t i = 0;
        if (n == 0) {
            f.data[i++] = static_cast<uint8_t>(crc & 0xFF);
            f.data[i++] = static_cast<uint8_t>(crc >> 8);
        }
        while (i < 7 && off < len) f.data[i++] = payload[off++];
        const bool sot = (n == 0), eot = (off >= len);
        f.data[i++] = TailByte::construct(sot, eot, toggle, tid);
        f.dlc = static_cast<uint8_t>(i);
        toggle = !toggle;
        ++n;
    }
    return n;
}

Reassembler::Slot* Reassembler::find_slot(uint16_t dtid, uint8_t node, uint32_t now_ms, bool create) {
    Slot* free_slot = nullptr;
    Slot* oldest = nullptr;
    for (auto& s : _slots) {
        if (s.active && now_ms - s.started_ms > TRANSFER_TIMEOUT_MS) s.active = false; // stale
        if (s.active && s.dtid == dtid && s.node == node) return &s;
        if (!s.active && !free_slot) free_slot = &s;
        if (!oldest || s.started_ms < oldest->started_ms) oldest = &s;
    }
    if (!create) return nullptr;
    return free_slot ? free_slot : oldest;
}

bool Reassembler::accept(const hal::CANFrame& frame, uint32_t now_ms, Transfer& out) {
    if (!frame.is_extended || frame.is_rtr || frame.dlc < 1 || frame.dlc > 8) return false;
    const auto id = CANIDFields::parse(frame.id);
    if (id.is_service || id.source_node_id == 0) return false; // services / anonymous not handled
    const auto tail = TailByte::parse(frame.data[frame.dlc - 1]);
    const uint8_t data_len = static_cast<uint8_t>(frame.dlc - 1);

    if (tail.sot && tail.eot) {
        if (tail.toggle) { ++_seq_errors; return false; }
        std::memcpy(_single_buf, frame.data, data_len);
        out = Transfer{id.data_type_id, id.source_node_id, tail.transfer_id, id.priority, now_ms, _single_buf, data_len};
        return true;
    }

    const uint64_t sig = signature_for(id.data_type_id);
    if (sig == 0) return false; // unknown type: we can't verify its CRC, so ignore

    if (tail.sot) {
        if (tail.toggle || data_len < 2) { ++_seq_errors; return false; }
        Slot* s = find_slot(id.data_type_id, id.source_node_id, now_ms, true);
        s->active = true;
        s->dtid = id.data_type_id;
        s->node = id.source_node_id;
        s->transfer_id = tail.transfer_id;
        s->next_toggle = true;
        s->signature = sig;
        s->started_ms = now_ms;
        s->expected_crc = static_cast<uint16_t>(frame.data[0] | (frame.data[1] << 8));
        s->len = data_len - 2U;
        std::memcpy(s->buf, frame.data + 2, s->len);
        s->crc = crc16_add(crc16_add_signature(0xFFFFU, sig), s->buf, s->len);
        return false;
    }

    Slot* s = find_slot(id.data_type_id, id.source_node_id, now_ms, false);
    if (!s) return false; // middle of a transfer we never saw the start of
    if (tail.transfer_id != s->transfer_id || tail.toggle != s->next_toggle) {
        ++_seq_errors;
        s->active = false;
        return false;
    }
    if (s->len + data_len > MAX_TRANSFER_PAYLOAD) {
        ++_seq_errors;
        s->active = false;
        return false;
    }
    std::memcpy(s->buf + s->len, frame.data, data_len);
    s->crc = crc16_add(s->crc, frame.data, data_len);
    s->len += data_len;
    s->next_toggle = !s->next_toggle;
    if (!tail.eot) return false;

    s->active = false;
    if (s->crc != s->expected_crc) {
        ++_crc_errors;
        return false;
    }
    out = Transfer{s->dtid, s->node, s->transfer_id, id.priority, now_ms, s->buf, s->len};
    return true;
}

} // namespace dronecan
} // namespace drivers
} // namespace autopilot
