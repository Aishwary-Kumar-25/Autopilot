#pragma once
/**
 * STM32 internal flash as the FlashDevice under hal::FlashStorage (settings
 * that survive a reboot). Two whole sectors are used as alternate copies.
 *
 * Default sectors (override with -D at build time, per board):
 *   STM32F405/F407/F427 (1 MB): sectors 10 and 11, 128 KB each at 0x080C0000
 *   and 0x080E0000. The LINKER SCRIPT MUST NOT PLACE CODE THERE: set the FLASH
 *   region LENGTH to 768K in the CubeMX .ld file.
 *   H7 / G4 / other sizes: set AP_STORAGE_SLOT{0,1}_ADDR, AP_STORAGE_SLOT{0,1}_SECTOR
 *   (the page number on G4), AP_STORAGE_SLOT_SIZE and on dual-bank H7 AP_STORAGE_BANK.
 *
 * Programming granularity follows the family: 32-bit words (F4/F7),
 * 64-bit double words (G4/L4), 256-bit flash words (H7).
 *
 * An erase stalls the CPU for 1-2 s on single-bank parts: storage users only
 * flush while disarmed (see comms/param_persistence.hpp).
 */
#include "hal/flash_storage.hpp"

#ifndef AP_STORAGE_SLOT0_ADDR
#define AP_STORAGE_SLOT0_ADDR 0x080C0000u
#define AP_STORAGE_SLOT1_ADDR 0x080E0000u
#define AP_STORAGE_SLOT0_SECTOR 10u
#define AP_STORAGE_SLOT1_SECTOR 11u
#define AP_STORAGE_SLOT_SIZE (128u * 1024u)
#endif
#ifndef AP_STORAGE_BANK
#define AP_STORAGE_BANK 1u
#endif

namespace autopilot {
namespace hal {

class STM32FlashDevice : public FlashDevice {
public:
    STM32FlashDevice(uint32_t addr0 = AP_STORAGE_SLOT0_ADDR, uint32_t sector0 = AP_STORAGE_SLOT0_SECTOR,
                     uint32_t addr1 = AP_STORAGE_SLOT1_ADDR, uint32_t sector1 = AP_STORAGE_SLOT1_SECTOR,
                     size_t slot_size = AP_STORAGE_SLOT_SIZE);

    size_t slot_size() const override { return _slot_size; }
    size_t program_unit() const override;
    bool erase(uint8_t slot) override;
    bool program(uint8_t slot, size_t offset, const uint8_t* data, size_t len) override;
    bool read(uint8_t slot, size_t offset, uint8_t* data, size_t len) override;

private:
    uint32_t _addr[2];
    uint32_t _sector[2];
    size_t _slot_size;
};

} // namespace hal
} // namespace autopilot
