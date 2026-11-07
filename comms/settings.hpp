#pragma once
/**
 * Saved settings: parameters that survive a reboot, on any hal::Storage
 * (a file in SITL, flash on STM32).
 *
 *   load()    at boot, after the parameters are registered: every saved value
 *             is applied through ParamStore::set(), so range checks and the
 *             on_set hooks run exactly as for a ground-station change. The
 *             values are applied in registration order, so VEHICLE_TYPE (the
 *             airframe) is switched before the tuning is pushed into it.
 *   update()  from the main loop. A parameter change is saved once nothing
 *             has changed for `save_delay_ms` (a GCS writing 20 params costs
 *             one flash erase, not 20), and storage is only committed while
 *             DISARMED: a flash erase stalls an STM32 for 1-2 s.
 *
 * Only values that differ from their defaults are stored, by name, so a
 * firmware that adds, removes or re-orders parameters still loads the rest,
 * and a changed default in new firmware takes effect unless the user set it.
 *
 * Region format (at storage_layout::PARAMS_OFFSET):
 *   magic "APPM" u32 | version u16 | count u16 | crc32(entries) u32 |
 *   count x { name char[16] (NUL padded) | value f32 }
 */
#include "comms/params.hpp"
#include "hal/hal.hpp"
#include <cstdint>

namespace autopilot {
namespace comms {

class Settings {
public:
    static constexpr uint32_t MAGIC = 0x4D505041u;  // "APPM"
    static constexpr uint16_t VERSION = 1;
    static constexpr size_t HEADER_LEN = 12;
    static constexpr size_t ENTRY_LEN = ParamStore::NAME_LEN + 4;

    Settings(ParamStore* params, hal::Storage* storage);

    bool available() const { return _storage != nullptr && _params != nullptr; }
    hal::Storage* storage() const { return _storage; }

    struct LoadResult {
        bool found{false};        // a valid saved block exists
        uint16_t loaded{0};       // values applied
        uint16_t unknown{0};      // names this firmware doesn't have (ignored)
        uint16_t rejected{0};     // out of range / refused by on_set
    };
    /** Apply saved values. Call once at boot, disarmed. */
    LoadResult load();

    /** Write the current non-default values into storage (RAM copy). Returns false if they don't fit. */
    bool save();
    /** save() and commit to non-volatile memory now (caller guarantees disarmed). */
    bool save_now();
    /** Forget all saved parameters and commit (does not change the live values). */
    bool erase_saved();

    /** Main-loop hook: debounced save + commit while disarmed. */
    void update(bool armed, uint32_t now_ms);

    /** Number of entries the region can hold. */
    size_t capacity() const;
    uint32_t commits() const { return _commits; }
    uint32_t commit_failures() const { return _commit_failures; }
    /** True after a save that didn't fit or a failed commit (cleared by the next success). */
    bool error() const { return _error; }

    uint32_t save_delay_ms{1000};
    uint32_t retry_ms{5000};

private:
    ParamStore* _params;
    hal::Storage* _storage;
    uint32_t _seen_revision{0};
    bool _save_pending{false};
    uint32_t _change_ms{0};
    uint32_t _last_commit_attempt_ms{0};
    bool _attempted{false};
    uint32_t _commits{0}, _commit_failures{0};
    bool _error{false};

    bool commit();
};

} // namespace comms
} // namespace autopilot
