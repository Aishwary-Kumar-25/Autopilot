/**
 * Saved settings tests.
 *   1. FileStorage (SITL): write, commit, reopen.
 *   2. FlashStorage (STM32 A/B sectors) on a simulated flash: alternating
 *      copies, and a power cut at EVERY possible step of a save never loses
 *      the settings (you get the old copy or the new one, never a mix).
 *   3. Settings: parameters survive a reboot, including the vehicle type;
 *      only changed values are stored; saves are debounced and never happen
 *      while armed; damaged or foreign data is ignored.
 *   4. MAV_CMD_PREFLIGHT_STORAGE from a GCS: write, read back, reset to defaults.
 */
#include "comms/storage_layout.hpp"
#include "hal/crc32.hpp"
#include "hal/flash_storage.hpp"
#include "hal/posix/file_storage.hpp"
#include "tests/gcs_rig.hpp"
#include "tests/test_common.hpp"

#include <cstdio>
#include <string>

using namespace rig;

namespace {

float pval(ParamStore& ps, const char* name) {
    const int i = ps.find(name);
    return i < 0 ? -12345.0f : ps.value(static_cast<size_t>(i));
}

void test_file_storage() {
    std::printf("-- FileStorage --\n");
    const uint8_t chk[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(hal::crc32(chk, 9) == 0xCBF43926u);   // CRC-32 check value

    const char* path = "test_storage.eeprom";
    std::remove(path);
    hal::FileStorage fs(path, 2048);
    CHECK(fs.open());
    uint8_t b[4];
    CHECK(fs.read(100, b, 4) && b[0] == 0xFF && b[3] == 0xFF);   // blank = erased
    const uint8_t data[] = {1, 2, 3, 4};
    CHECK(fs.write(2044, data, 4));
    CHECK(!fs.write(2045, data, 4));                            // past the end
    CHECK(fs.dirty());
    CHECK(fs.flush() && !fs.dirty() && fs.flush_count() == 1);
    CHECK(fs.write(2044, data, 4) && !fs.dirty());              // same bytes: nothing to commit
    hal::FileStorage again(path, 2048);
    CHECK(again.open());
    CHECK(again.read(2044, b, 4) && std::memcmp(b, data, 4) == 0);
    std::remove(path);
}

void test_flash_storage() {
    std::printf("-- FlashStorage (A/B sectors) --\n");
    for (size_t unit : {4u, 8u, 32u}) {     // F4/F7, G4, H7 programming granularity
        hal::SimFlashDevice flash(unit);
        uint8_t mirror[1000];
        hal::FlashStorage st(&flash, mirror, sizeof(mirror));
        CHECK(st.mount() && st.active_slot() == -1);
        uint8_t v = 0xAA;
        CHECK(st.write(10, &v, 1) && st.flush());
        CHECK(st.active_slot() == 0 && st.sequence() == 1);
        v = 0xBB;
        CHECK(st.write(999, &v, 1) && st.flush());
        CHECK(st.active_slot() == 1 && st.sequence() == 2);        // alternates
        CHECK(flash.erases[0] == 1 && flash.erases[1] == 1);
        CHECK(st.flush() && flash.erases[0] + flash.erases[1] == 2); // clean: no erase

        hal::FlashStorage reboot(&flash, mirror, sizeof(mirror));
        CHECK(reboot.mount() && reboot.active_slot() == 1 && reboot.sequence() == 2);
        uint8_t r1, r2;
        CHECK(reboot.read(10, &r1, 1) && reboot.read(999, &r2, 1) && r1 == 0xAA && r2 == 0xBB);

        // Damage the newest copy (bit rot / half-written): the older one is used.
        flash.raw(1)[hal::FlashStorage::HEADER_SPACE + 500] ^= 0x01;
        hal::FlashStorage fallback(&flash, mirror, sizeof(mirror));
        CHECK(fallback.mount() && fallback.active_slot() == 0);
        CHECK(fallback.read(10, &r1, 1) && fallback.read(999, &r2, 1) && r1 == 0xAA && r2 == 0xFF);
    }

    // Power cut at every step of a save.
    int cuts = 0, old_kept = 0, new_seen = 0, bad = 0;
    for (long cut = 0; cut < 400; ++cut) {
        hal::SimFlashDevice flash(4);
        uint8_t m[256];
        hal::FlashStorage st(&flash, m, sizeof(m));
        st.mount();
        uint8_t a[256], b[256];
        for (int i = 0; i < 256; ++i) { a[i] = static_cast<uint8_t>(i); b[i] = static_cast<uint8_t>(255 - i); }
        st.write(0, a, 256);
        st.flush();
        st.write(0, b, 256);
        st.flush();                     // now copy "b" (seq 2, slot 1) over "a" (slot 0)
        st.write(0, a, 256);
        flash.fail_after = cut;         // ...and the power fails part-way through saving "a" again
        const bool finished = st.flush();
        flash.fail_after = -1;
        if (!finished) ++cuts;

        uint8_t m2[256], got[256];
        hal::FlashStorage after(&flash, m2, sizeof(m2));
        if (!after.mount()) { ++bad; continue; }
        after.read(0, got, 256);
        if (std::memcmp(got, b, 256) == 0) ++old_kept;
        else if (std::memcmp(got, a, 256) == 0) ++new_seen;
        else ++bad;
        if (finished) break;
    }
    std::printf("  power cut at %d points: previous copy %d, new copy %d, corrupt %d\n", cuts, old_kept, new_seen, bad);
    // A cut inside the header padding still leaves a complete new copy: either outcome is fine, a mix is not.
    CHECK(cuts > 60 && bad == 0 && old_kept > 60 && new_seen >= 1);
}

void test_settings_reboot() {
    std::printf("-- parameters across a reboot --\n");
    const char* path = "test_params.eeprom";
    std::remove(path);
    hal::FileStorage storage(path);
    CHECK(storage.open());
    size_t defaults_count = 0;
    {
        Rig r(&storage);
        CHECK(!r.loaded.found);   // first boot: blank
        defaults_count = r.params.count();
        r.run_ms(100);
        r.gcs.param_set("MC_ROLLRATE_P", 0.25f);
        r.gcs.param_set("FS_RC_TIMEOUT", 800, MAV_PARAM_TYPE_INT32);
        r.gcs.param_set("FLTMODE", 2, MAV_PARAM_TYPE_INT32);      // state, not saved
        r.run_ms(500);
        CHECK(r.settings->commits() == 0);                        // debounced: waits for a quiet second
        r.run_ms(800);
        CHECK(r.settings->commits() == 1);
    }
    {
        Rig r(&storage);
        CHECK(r.loaded.found && r.loaded.loaded == 2 && r.loaded.rejected == 0);
        CHECK_NEAR(pval(r.params, "MC_ROLLRATE_P"), 0.25, 1e-6);
        CHECK_NEAR(pval(r.params, "FS_RC_TIMEOUT"), 800, 0);
        CHECK(r.vm.get_active_vehicle()->get_failsafe().config().rc_timeout_ms == 800);  // pushed into the vehicle
        CHECK(pval(r.params, "FLTMODE") != 2.0f);                 // booted in the default mode
        CHECK(r.params.count() == defaults_count);

        // Changes while ARMED are only saved after landing + disarm.
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
        r.run_ms(100);
        CHECK(r.vm.is_armed());
        const uint32_t commits = r.settings->commits();
        r.gcs.param_set("MC_YAWRATE_P", 0.3f);
        r.run_ms(3000);
        CHECK(r.settings->commits() == commits);                  // no flash erase in flight
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 0, ARM_DISARM_FORCE_MAGIC);
        r.run_ms(100);
        CHECK(!r.vm.is_armed());
        r.run_ms(100);
        CHECK(r.settings->commits() == commits + 1);              // saved once disarmed

        // The airframe choice is a saved setting too.
        r.gcs.param_set("VEHICLE_TYPE", 1, MAV_PARAM_TYPE_INT32); // UGV rover
        r.run_ms(1200);
        CHECK(r.vm.get_active_type() == vehicles::VehicleType::UGV_GROUND_ROVER);
    }
    {
        Rig r(&storage);
        CHECK(r.loaded.loaded == 4);
        CHECK(r.vm.get_active_type() == vehicles::VehicleType::UGV_GROUND_ROVER);
        // the failsafe setting reached the vehicle created by the type switch
        CHECK(r.vm.get_active_vehicle()->get_failsafe().config().rc_timeout_ms == 800);
        CHECK_NEAR(pval(r.params, "MC_YAWRATE_P"), 0.3, 1e-6);
    }

    // Only changed values are stored: an untouched vehicle saves an empty list.
    {
        hal::FileStorage s2("test_params2.eeprom");
        std::remove("test_params2.eeprom");
        s2.open();
        Rig r(&s2);
        CHECK(r.settings->save_now());
        uint8_t h[12];
        s2.read(storage_layout::PARAMS_OFFSET, h, 12);
        CHECK(h[6] == 0 && h[7] == 0);
        CHECK(r.settings->capacity() >= 100);
        std::remove("test_params2.eeprom");
    }

    // Damaged block: nothing is applied (defaults), nothing crashes.
    {
        uint8_t byte;
        storage.read(storage_layout::PARAMS_OFFSET + Settings::HEADER_LEN + 3, &byte, 1);
        byte ^= 0x20;
        storage.write(storage_layout::PARAMS_OFFSET + Settings::HEADER_LEN + 3, &byte, 1);
        Rig r(&storage);
        CHECK(!r.loaded.found && r.loaded.loaded == 0);
        CHECK(r.vm.get_active_type() == vehicles::VehicleType::UAV_MULTICOPTER);
    }
    std::remove(path);
}

void test_unknown_and_range() {
    std::printf("-- foreign / out-of-range saved values --\n");
    hal::FileStorage storage("test_params3.eeprom");
    std::remove("test_params3.eeprom");
    storage.open();
    // Hand-made block: one name this firmware doesn't have, one out of range, one good.
    struct E { const char* n; float v; } entries[] = {{"OLD_PARAM_X", 1.0f}, {"MC_ROLL_P", 999.0f}, {"MC_PITCH_P", 6.0f}};
    uint32_t crc = 0;
    size_t off = storage_layout::PARAMS_OFFSET + Settings::HEADER_LEN;
    for (const auto& e : entries) {
        uint8_t b[Settings::ENTRY_LEN] = {};
        std::memcpy(b, e.n, std::strlen(e.n));
        std::memcpy(b + 16, &e.v, 4);
        storage.write(off, b, sizeof(b));
        crc = hal::crc32(b, sizeof(b), crc);
        off += sizeof(b);
    }
    uint8_t h[12] = {'A', 'P', 'P', 'M', 1, 0, 3, 0};
    for (int i = 0; i < 4; ++i) h[8 + i] = static_cast<uint8_t>(crc >> (8 * i));
    storage.write(storage_layout::PARAMS_OFFSET, h, 12);
    Rig r(&storage);
    CHECK(r.loaded.found && r.loaded.loaded == 1 && r.loaded.unknown == 1 && r.loaded.rejected == 1);
    CHECK_NEAR(pval(r.params, "MC_PITCH_P"), 6.0, 1e-6);
    CHECK(pval(r.params, "MC_ROLL_P") < 20.0f);
    std::remove("test_params3.eeprom");
}

void test_preflight_storage() {
    std::printf("-- MAV_CMD_PREFLIGHT_STORAGE --\n");
    hal::FileStorage storage("test_params4.eeprom");
    std::remove("test_params4.eeprom");
    storage.open();
    Rig r(&storage);
    r.run_ms(100);
    const float def = pval(r.params, "MC_ALT_P");
    r.gcs.param_set("MC_ALT_P", 2.0f);
    r.run_ms(50);
    r.gcs.command(MAV_CMD_PREFLIGHT_STORAGE, 1);       // write now
    r.run_ms(50);
    CHECK(r.gcs.ack_result(MAV_CMD_PREFLIGHT_STORAGE) == MAV_RESULT_ACCEPTED);
    CHECK(!storage.dirty());
    r.gcs.param_set("MC_ALT_P", 3.0f);                 // unsaved change...
    r.run_ms(50);
    r.gcs.inbox.clear();
    r.gcs.command(MAV_CMD_PREFLIGHT_STORAGE, 0);       // ...dropped by "read from storage"
    r.run_ms(50);
    CHECK(r.gcs.ack_result(MAV_CMD_PREFLIGHT_STORAGE) == MAV_RESULT_ACCEPTED);
    CHECK_NEAR(pval(r.params, "MC_ALT_P"), 2.0, 1e-6);
    r.gcs.inbox.clear();
    r.gcs.command(MAV_CMD_PREFLIGHT_STORAGE, 2);       // QGC "reset all to defaults"
    r.run_ms(50);
    CHECK(r.gcs.ack_result(MAV_CMD_PREFLIGHT_STORAGE) == MAV_RESULT_ACCEPTED);
    CHECK_NEAR(pval(r.params, "MC_ALT_P"), def, 1e-6);
    Rig after(&storage);
    CHECK(after.loaded.found && after.loaded.loaded == 0);
    CHECK_NEAR(pval(after.params, "MC_ALT_P"), def, 1e-6);

    // armed: refused
    r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    r.run_ms(50);
    r.gcs.inbox.clear();
    r.gcs.command(MAV_CMD_PREFLIGHT_STORAGE, 2);
    r.run_ms(50);
    CHECK(r.vm.is_armed() && r.gcs.ack_result(MAV_CMD_PREFLIGHT_STORAGE) == MAV_RESULT_DENIED);

    // no storage on the board: UNSUPPORTED, and the link still works
    Rig none(nullptr);
    none.gcs.command(MAV_CMD_PREFLIGHT_STORAGE, 1);
    none.run_ms(50);
    CHECK(none.gcs.ack_result(MAV_CMD_PREFLIGHT_STORAGE) == MAV_RESULT_UNSUPPORTED);
    std::remove("test_params4.eeprom");
}

} // namespace

int main() {
    test_file_storage();
    test_flash_storage();
    test_settings_reboot();
    test_unknown_and_range();
    test_preflight_storage();
    return test::finish("test_storage");
}
