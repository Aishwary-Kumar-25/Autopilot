#include "stm32_flash.hpp"
#include <cstring>

#if defined(USE_HAL_DRIVER)
#include "main.h"
#define AP_ON_TARGET 1
#else
#define AP_ON_TARGET 0
#endif

namespace autopilot {
namespace hal {

STM32FlashDevice::STM32FlashDevice(uint32_t addr0, uint32_t sector0, uint32_t addr1, uint32_t sector1,
                                   size_t slot_size)
    : _addr{addr0, addr1}, _sector{sector0, sector1}, _slot_size(slot_size) {}

size_t STM32FlashDevice::program_unit() const {
#if AP_ON_TARGET && defined(FLASH_TYPEPROGRAM_FLASHWORD)
    return 32;   // H7: 256-bit flash word
#elif AP_ON_TARGET && defined(FLASH_TYPEPROGRAM_DOUBLEWORD) && !defined(FLASH_TYPEPROGRAM_WORD)
    return 8;    // G4 / L4: 64-bit double word
#else
    return 4;    // F4 / F7: 32-bit word
#endif
}

bool STM32FlashDevice::erase(uint8_t slot) {
    if (slot > 1) return false;
#if AP_ON_TARGET
    FLASH_EraseInitTypeDef e{};
    uint32_t bad = 0;
#if defined(FLASH_TYPEERASE_PAGES) && !defined(FLASH_TYPEERASE_SECTORS)
    e.TypeErase = FLASH_TYPEERASE_PAGES;           // G4 / L4: pages
    e.Page = _sector[slot];
    e.NbPages = static_cast<uint32_t>(_slot_size / FLASH_PAGE_SIZE);
#if defined(FLASH_BANK_1)
    e.Banks = AP_STORAGE_BANK == 2 ? FLASH_BANK_2 : FLASH_BANK_1;
#endif
#else
    e.TypeErase = FLASH_TYPEERASE_SECTORS;         // F4 / F7 / H7: sectors
    e.Sector = _sector[slot];
    e.NbSectors = 1;
#if defined(FLASH_VOLTAGE_RANGE_3)
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;        // 2.7-3.6 V, 32-bit parallelism
#endif
#if defined(FLASH_BANK_2)
    e.Banks = AP_STORAGE_BANK == 2 ? FLASH_BANK_2 : FLASH_BANK_1;
#endif
#endif
    HAL_FLASH_Unlock();
    const bool ok = HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
    HAL_FLASH_Lock();
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)   // F7 / H7: drop stale cached flash
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(_addr[slot]), static_cast<int32_t>(_slot_size));
#endif
    return ok;
#else
    return false;   // host build: no flash
#endif
}

bool STM32FlashDevice::program(uint8_t slot, size_t offset, const uint8_t* data, size_t len) {
    const size_t unit = program_unit();
    if (slot > 1 || !data || offset % unit || len % unit || offset + len > _slot_size) return false;
#if AP_ON_TARGET
    bool ok = true;
    HAL_FLASH_Unlock();
    for (size_t i = 0; ok && i < len; i += unit) {
        const uint32_t addr = _addr[slot] + static_cast<uint32_t>(offset + i);
#if defined(FLASH_TYPEPROGRAM_FLASHWORD)
        alignas(32) uint32_t w[8];
        std::memcpy(w, data + i, 32);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(w))) == HAL_OK;
#elif defined(FLASH_TYPEPROGRAM_DOUBLEWORD) && !defined(FLASH_TYPEPROGRAM_WORD)
        uint64_t d;
        std::memcpy(&d, data + i, 8);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, d) == HAL_OK;
#else
        uint32_t w;
        std::memcpy(&w, data + i, 4);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w) == HAL_OK;
#endif
    }
    HAL_FLASH_Lock();
    return ok;
#else
    return false;
#endif
}

bool STM32FlashDevice::read(uint8_t slot, size_t offset, uint8_t* data, size_t len) {
    if (slot > 1 || !data || offset + len > _slot_size) return false;
#if AP_ON_TARGET
    std::memcpy(data, reinterpret_cast<const void*>(_addr[slot] + offset), len);  // memory-mapped
    return true;
#else
    return false;
#endif
}

} // namespace hal
} // namespace autopilot
