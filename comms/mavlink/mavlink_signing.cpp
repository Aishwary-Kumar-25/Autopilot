#include "comms/mavlink/mavlink_signing.hpp"
#include "comms/mavlink/mavlink_protocol.hpp"
#include "comms/mavlink/sha256.hpp"
#include "comms/storage_layout.hpp"
#include "hal/crc32.hpp"
#include <cstring>

namespace autopilot {
namespace comms {
namespace mavlink {

namespace {
constexpr uint32_t RECORD_MAGIC = 0x47535041u;   // "APSG"
// record: magic u32 | enabled u8 | link_id u8 | pad u16 | key[32] | timestamp u64 | crc32 u32  = 52 bytes
constexpr size_t RECORD_LEN = 52;

void put_le(uint8_t* p, uint64_t v, size_t n) { for (size_t i = 0; i < n; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
uint64_t get_le(const uint8_t* p, size_t n) {
    uint64_t v = 0;
    for (size_t i = 0; i < n; ++i) v |= static_cast<uint64_t>(p[i]) << (8 * i);
    return v;
}
/** Constant-time compare: no early exit that would leak how many bytes matched. */
bool equal_ct(const uint8_t* a, const uint8_t* b, size_t n) {
    uint8_t d = 0;
    for (size_t i = 0; i < n; ++i) d |= static_cast<uint8_t>(a[i] ^ b[i]);
    return d == 0;
}
} // namespace

void Signing::set_key(const uint8_t key[KEY_LEN], uint64_t initial_timestamp) {
    bool all_zero = true;
    for (size_t i = 0; i < KEY_LEN; ++i) all_zero &= (key[i] == 0);
    if (all_zero) {
        disable();
        return;
    }
    std::memcpy(_key, key, KEY_LEN);
    _enabled = true;
    advance_to(initial_timestamp);
    for (auto& s : _streams) s.used = false;   // a new key starts new streams
}

void Signing::disable() {
    std::memset(_key, 0, KEY_LEN);
    _enabled = false;
    for (auto& s : _streams) s.used = false;
}

void Signing::key_from_passphrase(const char* passphrase, uint8_t key[KEY_LEN]) {
    Sha256::hash(reinterpret_cast<const uint8_t*>(passphrase ? passphrase : ""),
                 passphrase ? std::strlen(passphrase) : 0, key);
}

void Signing::tick(uint32_t now_ms) {
    if (_have_tick) {
        const uint32_t dt = now_ms - _last_tick_ms;
        _ts += static_cast<uint64_t>(dt) * 100u;   // 1 ms = 100 x 10 us
    }
    _have_tick = true;
    _last_tick_ms = now_ms;
}

void Signing::signature(const uint8_t* frame, size_t n, uint8_t out[6]) const {
    Sha256 h;
    h.update(_key, KEY_LEN);
    h.update(frame, n);
    uint8_t d[Sha256::DIGEST_LEN];
    h.final(d);
    std::memcpy(out, d, 6);
}

size_t Signing::sign(uint8_t* frame, size_t len) {
    if (!_enabled || len < 12 || frame[0] != STX_V2 || !(frame[2] & INCOMPAT_FLAG_SIGNED)) return len;
    frame[len] = _link_id;
    put_le(frame + len + 1, _ts, 6);
    signature(frame, len + 7, frame + len + 7);
    ++_ts;   // every frame gets a unique timestamp
    return len + SIGNATURE_LEN;
}

Signing::Result Signing::verify(const uint8_t* f, size_t len, uint8_t sysid, uint8_t compid) {
    if (!_enabled) return Result::DISABLED;
    if (len < 12 || f[0] != STX_V2 || !(f[2] & INCOMPAT_FLAG_SIGNED) || len != 12u + f[1] + SIGNATURE_LEN) {
        ++_rejected;
        return Result::UNSIGNED;
    }
    const uint8_t* sig = f + len - SIGNATURE_LEN;
    const uint8_t link = sig[0];
    const uint64_t ts = get_le(sig + 1, 6);

    // Signature first, so a forged frame can't disturb the replay state.
    uint8_t expect[6];
    signature(f, len - 6, expect);
    if (!equal_ct(expect, sig + 7, 6)) {
        ++_rejected;
        return Result::BAD_SIGNATURE;
    }

    Stream* st = nullptr;
    for (auto& s : _streams) {
        if (s.used && s.link_id == link && s.sysid == sysid && s.compid == compid) { st = &s; break; }
    }
    if (st) {
        if (ts <= st->ts) { ++_rejected; return Result::REPLAY; }
    } else {
        if (ts + ONE_MINUTE < _ts) { ++_rejected; return Result::TOO_OLD; }
        // New stream: free slot, else replace the stalest one.
        for (auto& s : _streams) if (!s.used) { st = &s; break; }
        if (!st) {
            st = &_streams[0];
            for (auto& s : _streams) if (s.ts < st->ts) st = &s;
        }
        st->used = true;
        st->link_id = link;
        st->sysid = sysid;
        st->compid = compid;
    }
    st->ts = ts;
    advance_to(ts);
    ++_accepted;
    return Result::OK;
}

bool Signing::load(hal::Storage* storage) {
    if (!storage) return false;
    uint8_t r[RECORD_LEN];
    if (!storage->read(storage_layout::SIGNING_OFFSET, r, RECORD_LEN)) return false;
    if (get_le(r, 4) != RECORD_MAGIC || get_le(r + 48, 4) != hal::crc32(r, 48)) return false;
    _link_id = r[5];
    if (r[4]) {
        std::memcpy(_key, r + 8, KEY_LEN);
        _enabled = true;
    } else {
        disable();
    }
    advance_to(get_le(r + 40, 8));
    return true;
}

bool Signing::save(hal::Storage* storage) const {
    if (!storage || storage->size() < storage_layout::SIGNING_OFFSET + RECORD_LEN) return false;
    uint8_t r[RECORD_LEN] = {};
    put_le(r, RECORD_MAGIC, 4);
    r[4] = _enabled ? 1 : 0;
    r[5] = _link_id;
    if (_enabled) std::memcpy(r + 8, _key, KEY_LEN);
    put_le(r + 40, _ts, 8);
    put_le(r + 48, hal::crc32(r, 48), 4);
    return storage->write(storage_layout::SIGNING_OFFSET, r, RECORD_LEN);
}

} // namespace mavlink
} // namespace comms
} // namespace autopilot
