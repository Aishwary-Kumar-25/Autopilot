#pragma once
/**
 * Power-loss-safe settings storage on raw flash (portable part; the chip
 * access is a FlashDevice, see hal/stm32/stm32_flash.hpp).
 *
 * Two flash slots (sectors) hold alternate copies. Each copy is
 *     header { magic, sequence, length, crc32(data) }  (padded to 32 bytes)
 *     data[length]
 * flush() erases the OLDER slot, programs the data, and programs the header
 * last. Erased flash reads 0xFF, so a copy whose header never got written (or
 * whose CRC does not match, e.g. a power cut mid-write) is ignored and mount()
 * falls back to the other, intact copy. One flush = one sector erase, so
 * callers batch changes (ParamPersistence waits for a quiet second) to keep
 * well inside flash endurance (~10k erases per sector).
 *
 * No heap: the caller provides the RAM mirror.
 */
#include "hal/hal.hpp"

namespace autopilot {
namespace hal {

/** Two equal flash slots with erase / program / read. */
class FlashDevice {
public:
    virtual ~FlashDevice() = default;
    /** Bytes per slot (the sector size). */
    virtual size_t slot_size() const = 0;
    /** Smallest programmable unit in bytes (4 on F4/F7, 8 on G4/L4, 32 on H7). */
    virtual size_t program_unit() const = 0;
    virtual bool erase(uint8_t slot) = 0;
    /** offset and len are multiples of program_unit(); the area must be erased. */
    virtual bool program(uint8_t slot, size_t offset, const uint8_t* data, size_t len) = 0;
    virtual bool read(uint8_t slot, size_t offset, uint8_t* data, size_t len) = 0;
};

class FlashStorage : public Storage {
public:
    static constexpr uint32_t MAGIC = 0x54535041u;   // "APST"
    static constexpr size_t HEADER_SPACE = 32;       // header padded to the largest program unit

    /** mirror/size: RAM copy of the stored bytes (size + HEADER_SPACE must fit in a slot). */
    FlashStorage(FlashDevice* dev, uint8_t* mirror, size_t size);

    /** Load the newest valid copy. False only if the flash can't be read or is too small. */
    bool mount();
    bool mounted() const { return _mounted; }

    size_t size() const override { return _size; }
    bool read(size_t offset, uint8_t* data, size_t len) override;
    bool write(size_t offset, const uint8_t* data, size_t len) override;
    bool flush() override;
    bool dirty() const override { return _dirty; }

    /** Slot holding the current copy (-1 = none yet), and its sequence number. */
    int active_slot() const { return _active; }
    uint32_t sequence() const { return _seq; }

private:
    FlashDevice* _dev;
    uint8_t* _mirror;
    size_t _size;
    bool _mounted{false};
    bool _dirty{false};
    int _active{-1};
    uint32_t _seq{0};

    bool slot_valid(uint8_t slot, uint32_t& seq);
};

/** RAM-backed FlashDevice for tests: emulates erase-to-0xFF, 1->0 only programming and power cuts. */
class SimFlashDevice : public FlashDevice {
public:
    static constexpr size_t SLOT = 8192;
    explicit SimFlashDevice(size_t unit = 4) : _unit(unit) {
        for (auto& s : _mem) for (auto& b : s) b = 0xFF;
    }
    size_t slot_size() const override { return SLOT; }
    size_t program_unit() const override { return _unit; }
    bool erase(uint8_t slot) override {
        if (slot > 1 || fail_now()) return false;
        for (auto& b : _mem[slot]) b = 0xFF;
        ++erases[slot];
        return true;
    }
    bool program(uint8_t slot, size_t off, const uint8_t* d, size_t n) override {
        if (slot > 1 || off % _unit || n % _unit || off + n > SLOT) return false;
        for (size_t i = 0; i < n; i += _unit) {
            if (fail_now()) return false;            // power cut part-way through
            for (size_t k = 0; k < _unit; ++k) _mem[slot][off + i + k] &= d[i + k];  // flash only clears bits
        }
        return true;
    }
    bool read(uint8_t slot, size_t off, uint8_t* d, size_t n) override {
        if (slot > 1 || off + n > SLOT) return false;
        for (size_t i = 0; i < n; ++i) d[i] = _mem[slot][off + i];
        return true;
    }
    /** Fail every operation after this many more erase/program steps (-1 = never). */
    long fail_after{-1};
    unsigned erases[2]{0, 0};
    uint8_t* raw(uint8_t slot) { return _mem[slot]; }

private:
    size_t _unit;
    uint8_t _mem[2][SLOT];
    bool fail_now() {
        if (fail_after < 0) return false;
        if (fail_after == 0) return true;
        --fail_after;
        return false;
    }
};

} // namespace hal
} // namespace autopilot
