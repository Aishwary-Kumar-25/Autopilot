#include "hal/flash_storage.hpp"
#include "hal/crc32.hpp"
#include <cstring>

namespace autopilot {
namespace hal {

namespace {
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
uint32_t get32(const uint8_t* p) {
    return p[0] | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
} // namespace

FlashStorage::FlashStorage(FlashDevice* dev, uint8_t* mirror, size_t size)
    : _dev(dev), _mirror(mirror), _size(size) {
    if (_mirror) std::memset(_mirror, 0xFF, _size);
}

bool FlashStorage::slot_valid(uint8_t slot, uint32_t& seq) {
    uint8_t h[16];
    if (!_dev->read(slot, 0, h, sizeof(h))) return false;
    if (get32(h) != MAGIC || get32(h + 8) != _size) return false;
    // CRC over the data, streamed in chunks (no second full-size buffer).
    uint8_t chunk[64];
    uint32_t crc = 0;
    for (size_t off = 0; off < _size; off += sizeof(chunk)) {
        const size_t n = (_size - off < sizeof(chunk)) ? _size - off : sizeof(chunk);
        if (!_dev->read(slot, HEADER_SPACE + off, chunk, n)) return false;
        crc = crc32(chunk, n, crc);
    }
    if (crc != get32(h + 12)) return false;
    seq = get32(h + 4);
    return true;
}

bool FlashStorage::mount() {
    _mounted = false;
    if (!_dev || !_mirror || _size == 0 || HEADER_SPACE + _size > _dev->slot_size()) return false;
    if (_dev->program_unit() == 0 || HEADER_SPACE % _dev->program_unit() != 0) return false;
    uint8_t probe;
    if (!_dev->read(0, 0, &probe, 1) || !_dev->read(1, 0, &probe, 1)) return false;

    uint32_t s0 = 0, s1 = 0;
    const bool v0 = slot_valid(0, s0), v1 = slot_valid(1, s1);
    _active = -1;
    if (v0 && v1) _active = (static_cast<int32_t>(s1 - s0) > 0) ? 1 : 0;  // wrap-safe "newer"
    else if (v0) _active = 0;
    else if (v1) _active = 1;

    if (_active >= 0) {
        _seq = (_active == 0) ? s0 : s1;
        if (!_dev->read(static_cast<uint8_t>(_active), HEADER_SPACE, _mirror, _size)) return false;
    } else {
        _seq = 0;
        std::memset(_mirror, 0xFF, _size);
    }
    _dirty = false;
    _mounted = true;
    return true;
}

bool FlashStorage::read(size_t offset, uint8_t* data, size_t len) {
    if (!_mounted || offset > _size || len > _size - offset) return false;
    std::memcpy(data, _mirror + offset, len);
    return true;
}

bool FlashStorage::write(size_t offset, const uint8_t* data, size_t len) {
    if (!_mounted || offset > _size || len > _size - offset) return false;
    if (std::memcmp(_mirror + offset, data, len) != 0) {
        std::memcpy(_mirror + offset, data, len);
        _dirty = true;
    }
    return true;
}

bool FlashStorage::flush() {
    if (!_mounted) return false;
    if (!_dirty) return true;
    const uint8_t target = (_active == 0) ? 1 : 0;   // never touch the current good copy
    const size_t unit = _dev->program_unit();
    if (!_dev->erase(target)) return false;

    // Data first, in program-unit pieces (the tail padded with 0xFF).
    uint8_t buf[64];
    const size_t step = (sizeof(buf) / unit) * unit;
    for (size_t off = 0; off < _size; off += step) {
        size_t n = (_size - off < step) ? _size - off : step;
        std::memset(buf, 0xFF, sizeof(buf));
        std::memcpy(buf, _mirror + off, n);
        n = (n + unit - 1) / unit * unit;
        if (!_dev->program(target, HEADER_SPACE + off, buf, n)) return false;
    }
    // Header last: until it is in place the new copy doesn't exist.
    uint8_t h[HEADER_SPACE];
    std::memset(h, 0xFF, sizeof(h));
    const uint32_t seq = _seq + 1;
    put32(h, MAGIC);
    put32(h + 4, seq);
    put32(h + 8, static_cast<uint32_t>(_size));
    put32(h + 12, crc32(_mirror, _size));
    if (!_dev->program(target, 0, h, sizeof(h))) return false;

    uint32_t check_seq = 0;
    if (!slot_valid(target, check_seq) || check_seq != seq) return false;  // read-back verify
    _active = target;
    _seq = seq;
    _dirty = false;
    return true;
}

} // namespace hal
} // namespace autopilot
