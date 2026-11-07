#pragma once
/**
 * MAVLink framing (v2 send, v1 + v2 receive), written from the protocol spec
 * (https://mavlink.io/en/guide/serialization.html). No generated headers, no
 * heap: a frame is at most 280 bytes on the stack.
 *
 * v2 frame: FD len incompat compat seq sysid compid msgid[3] payload[len] crc[2] (sig[13])
 * v1 frame: FE len seq sysid compid msgid payload[len] crc[2]
 * CRC = X.25 (MCRF4XX) over everything after the start byte, then the
 * message's CRC_EXTRA byte (a hash of its field layout; both ends must agree).
 *
 * v2 senders strip trailing zero bytes from the payload; receivers zero-fill
 * up to the length they know. Payload structs in mavlink_messages.hpp decode
 * from a zero-filled buffer, so truncation and newer/older extension fields
 * are handled for free.
 */
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace autopilot {
namespace comms {
namespace mavlink {

constexpr uint8_t STX_V1 = 0xFE;
constexpr uint8_t STX_V2 = 0xFD;
constexpr uint8_t INCOMPAT_FLAG_SIGNED = 0x01;
constexpr size_t MAX_PAYLOAD = 255;
constexpr size_t SIGNATURE_LEN = 13;
constexpr size_t MAX_FRAME = 10 + MAX_PAYLOAD + 2 + SIGNATURE_LEN; // 280

/** X.25 / MCRF4XX accumulate, as used by MAVLink. */
inline uint16_t crc_accumulate(uint8_t b, uint16_t crc) {
    uint8_t tmp = b ^ static_cast<uint8_t>(crc & 0xFF);
    tmp ^= static_cast<uint8_t>(tmp << 4);
    return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                                 (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}
inline uint16_t crc_calculate(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < len; ++i) crc = crc_accumulate(data[i], crc);
    return crc;
}

/** CRC_EXTRA for the message ids this stack understands. Returns false if unknown. */
bool crc_extra_for(uint32_t msgid, uint8_t& crc_extra);

/** A received message with its payload zero-filled to MAX_PAYLOAD. */
struct Message {
    uint32_t msgid{0};
    uint8_t sysid{0};
    uint8_t compid{0};
    uint8_t seq{0};
    uint8_t len{0};          // payload bytes actually on the wire
    bool v2{true};
    bool signed_frame{false}; // v2 signature present (verified by GcsLink when signing is on)
    uint8_t payload[MAX_PAYLOAD]{};
};

/**
 * Build a v2 frame. Trailing zero payload bytes are trimmed (min length 1),
 * as the spec requires. Returns the frame length written to out (>= MAX_FRAME bytes).
 * incompat_flags = INCOMPAT_FLAG_SIGNED marks a frame that Signing::sign() will
 * complete (the flag is covered by the CRC, so it must be set before).
 */
size_t encode_v2(uint8_t* out, uint8_t seq, uint8_t sysid, uint8_t compid, uint32_t msgid,
                 const uint8_t* payload, size_t payload_len, uint8_t crc_extra, uint8_t incompat_flags = 0);

/**
 * Byte-at-a-time frame parser (v1 and v2). Resynchronises on garbage; frames
 * with a bad CRC or an unknown message id (whose CRC can't be checked) are
 * dropped and counted.
 */
class Parser {
public:
    /** Feed one byte. Returns true when a complete, valid message is in msg(). */
    bool parse(uint8_t byte);
    const Message& msg() const { return _msg; }
    /** Raw bytes of the message just returned (valid until the next parse()); for signature checks. */
    const uint8_t* frame() const { return _buf; }
    size_t frame_len() const { return _frame_len; }

    uint32_t crc_errors() const { return _crc_errors; }
    uint32_t unknown_ids() const { return _unknown_ids; }
    uint32_t frames_ok() const { return _frames_ok; }
    /** Messages missed, from gaps in the sender's sequence numbers. */
    uint32_t seq_lost() const { return _seq_lost; }

private:
    enum class State : uint8_t { IDLE, HEADER, PAYLOAD, CRC, SIGNATURE };
    State _state{State::IDLE};
    uint8_t _buf[MAX_FRAME]{};
    size_t _pos{0};
    size_t _header_len{0};
    size_t _need{0};
    size_t _frame_len{0};
    Message _msg;
    uint32_t _crc_errors{0}, _unknown_ids{0}, _frames_ok{0}, _seq_lost{0};
    bool _have_seq{false};
    uint8_t _last_seq{0};

    bool finish();
};

// ---- little-endian payload helpers ----

class Writer {
public:
    explicit Writer(uint8_t* buf) : _b(buf) {}
    void u8(uint8_t v) { _b[_n++] = v; }
    void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
    void u16(uint16_t v) { u8(v & 0xFF); u8(v >> 8); }
    void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
    void u32(uint32_t v) { u16(v & 0xFFFF); u16(v >> 16); }
    void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
    void u64(uint64_t v) { u32(static_cast<uint32_t>(v)); u32(static_cast<uint32_t>(v >> 32)); }
    void i64(int64_t v) { u64(static_cast<uint64_t>(v)); }
    void f32(float v) { uint32_t u; std::memcpy(&u, &v, 4); u32(u); }
    void chars(const char* s, size_t n) {   // fixed-size, NUL padded, NOT necessarily terminated
        size_t i = 0;
        for (; s && i < n && s[i]; ++i) u8(static_cast<uint8_t>(s[i]));
        for (; i < n; ++i) u8(0);
    }
    void bytes(const uint8_t* p, size_t n) { for (size_t i = 0; i < n; ++i) u8(p ? p[i] : 0); }
    size_t size() const { return _n; }

private:
    uint8_t* _b;
    size_t _n{0};
};

class Reader {
public:
    explicit Reader(const uint8_t* buf) : _b(buf) {}
    uint8_t u8() { return _b[_n++]; }
    int8_t i8() { return static_cast<int8_t>(u8()); }
    uint16_t u16() { uint16_t lo = u8(); return static_cast<uint16_t>(lo | (u8() << 8)); }
    int16_t i16() { return static_cast<int16_t>(u16()); }
    uint32_t u32() { uint32_t lo = u16(); return lo | (static_cast<uint32_t>(u16()) << 16); }
    int32_t i32() { return static_cast<int32_t>(u32()); }
    uint64_t u64() { uint64_t lo = u32(); return lo | (static_cast<uint64_t>(u32()) << 32); }
    int64_t i64() { return static_cast<int64_t>(u64()); }
    float f32() { uint32_t u = u32(); float f; std::memcpy(&f, &u, 4); return f; }
    /** Copy n chars and always NUL-terminate (out must hold n + 1). */
    void chars(char* out, size_t n) {
        for (size_t i = 0; i < n; ++i) out[i] = static_cast<char>(u8());
        out[n] = '\0';
    }

private:
    const uint8_t* _b;
    size_t _n{0};
};

} // namespace mavlink
} // namespace comms
} // namespace autopilot
