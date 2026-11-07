#pragma once
/**
 * MAVLink 2 message signing (https://mavlink.io/en/guide/message_signing.html).
 *
 * A signed frame carries 13 extra bytes after the CRC:
 *     link_id u8 | timestamp u48 (10 us units since 2015-01-01 UTC) | signature[6]
 * signature = first 6 bytes of SHA-256(secret_key[32] + frame up to and
 * including the timestamp). Only someone holding the 32-byte key can produce
 * it, and the timestamp makes every frame usable once: for each stream
 * (link_id, sysid, compid) a timestamp must be strictly greater than the last
 * one accepted, and a stream we have not seen yet may be at most one minute
 * behind our own clock. Replaying a captured "arm" command therefore fails.
 *
 * Our clock: the timestamp saved with the key, moved forward by elapsed boot
 * time, and pulled forward to any newer timestamp in a verified frame (the
 * ground station's clock is real UTC time), as the spec recommends for boards
 * without a real-time clock.
 */
#include "hal/hal.hpp"
#include <cstddef>
#include <cstdint>

namespace autopilot {
namespace comms {
namespace mavlink {

class Signing {
public:
    static constexpr size_t KEY_LEN = 32;
    static constexpr uint64_t ONE_MINUTE = 6000000ull;      // in 10 us units
    static constexpr uint64_t UNIX_2015 = 1420070400ull;    // 2015-01-01 00:00:00 UTC in Unix seconds
    static constexpr size_t MAX_STREAMS = 16;

    enum class Result : uint8_t {
        OK,             // signature valid, timestamp fresh
        UNSIGNED,       // no signature on the frame
        BAD_SIGNATURE,  // wrong key or tampered frame
        REPLAY,         // timestamp not newer than the last one on this stream
        TOO_OLD,        // new stream more than a minute behind our clock
        DISABLED,       // signing is off: nothing to check
    };

    bool enabled() const { return _enabled; }
    uint8_t link_id() const { return _link_id; }
    void set_link_id(uint8_t id) { _link_id = id; }
    uint64_t timestamp() const { return _ts; }

    /**
     * Install a key (SETUP_SIGNING). The clock never goes backwards:
     * timestamp = max(current, initial_timestamp). A key of 32 zero bytes
     * turns signing off (as the spec defines).
     */
    void set_key(const uint8_t key[KEY_LEN], uint64_t initial_timestamp);
    void disable();
    /** Key from a passphrase, as MAVProxy's "signing setup" does: SHA-256(passphrase). */
    static void key_from_passphrase(const char* passphrase, uint8_t key[KEY_LEN]);
    /** Convert Unix time (seconds) to a signing timestamp, e.g. from GPS time. */
    static uint64_t timestamp_from_unix(uint64_t unix_s) {
        return unix_s > UNIX_2015 ? (unix_s - UNIX_2015) * 100000ull : 0;
    }
    /** Move the clock forward to `ts` if it is ahead (e.g. GPS time). */
    void advance_to(uint64_t ts) { if (ts > _ts) _ts = ts; }

    /** Advance the clock with the boot clock. Call regularly (each update). */
    void tick(uint32_t now_ms);

    /**
     * Append the 13-byte signature to a complete v2 frame that was encoded
     * WITH the signed incompat flag (the flag is covered by the CRC).
     * Returns the new frame length (len + 13), or len if signing is off.
     */
    size_t sign(uint8_t* frame, size_t len);

    /** Check a received frame (full bytes, including any signature). */
    Result verify(const uint8_t* frame, size_t len, uint8_t sysid, uint8_t compid);

    // ---- persistence (comms/storage_layout.hpp SIGNING region) ----
    /** Load key + timestamp. Returns true if a valid record was found. */
    bool load(hal::Storage* storage);
    /** Write key + current timestamp into the storage RAM copy (commit with flush()). */
    bool save(hal::Storage* storage) const;

    uint32_t accepted() const { return _accepted; }
    uint32_t rejected() const { return _rejected; }

private:
    bool _enabled{false};
    uint8_t _key[KEY_LEN]{};
    uint8_t _link_id{0};
    uint64_t _ts{0};
    bool _have_tick{false};
    uint32_t _last_tick_ms{0};
    uint32_t _sub_ms{0};

    struct Stream { bool used; uint8_t link_id, sysid, compid; uint64_t ts; };
    Stream _streams[MAX_STREAMS]{};
    uint32_t _accepted{0}, _rejected{0};

    void signature(const uint8_t* frame, size_t len_without_sig6, uint8_t out[6]) const;
};

} // namespace mavlink
} // namespace comms
} // namespace autopilot
