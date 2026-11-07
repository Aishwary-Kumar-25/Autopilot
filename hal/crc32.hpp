#pragma once
/** CRC-32 (IEEE 802.3, reflected 0xEDB88320), as zlib.crc32. Bitwise: small, no table. */
#include <cstddef>
#include <cstdint>

namespace autopilot {
namespace hal {

inline uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc = 0) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

} // namespace hal
} // namespace autopilot
