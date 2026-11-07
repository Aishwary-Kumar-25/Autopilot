#pragma once
/**
 * SHA-256 (FIPS 180-4), small and heap-free, for MAVLink 2 message signing.
 * Incremental: init, update as many times as needed, final.
 */
#include <cstddef>
#include <cstdint>

namespace autopilot {
namespace comms {
namespace mavlink {

class Sha256 {
public:
    static constexpr size_t DIGEST_LEN = 32;
    Sha256() { reset(); }
    void reset();
    void update(const uint8_t* data, size_t len);
    void final(uint8_t out[DIGEST_LEN]);

    /** One-shot convenience. */
    static void hash(const uint8_t* data, size_t len, uint8_t out[DIGEST_LEN]) {
        Sha256 h;
        h.update(data, len);
        h.final(out);
    }

private:
    uint32_t _state[8];
    uint64_t _bits;
    uint8_t _block[64];
    size_t _fill;
    void compress(const uint8_t block[64]);
};

} // namespace mavlink
} // namespace comms
} // namespace autopilot
