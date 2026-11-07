#pragma once
/**
 * Where each kind of saved setting lives inside hal::Storage. Every region
 * carries its own magic number and CRC, so a blank, old-format or damaged
 * region is detected and ignored (the defaults are used instead).
 *
 *   0    .. 127   MAVLink signing key + last timestamp   (comms/mavlink/mavlink_signing.hpp)
 *   128  .. end   parameters                             (comms/settings.hpp)
 */
#include <cstddef>

namespace autopilot {
namespace comms {
namespace storage_layout {

constexpr size_t SIGNING_OFFSET = 0;
constexpr size_t SIGNING_SIZE = 128;
constexpr size_t PARAMS_OFFSET = 128;
constexpr size_t MIN_STORAGE_SIZE = 1024;

} // namespace storage_layout
} // namespace comms
} // namespace autopilot
