#include "comms/settings.hpp"
#include "comms/storage_layout.hpp"
#include "hal/crc32.hpp"
#include <cstring>

namespace autopilot {
namespace comms {

namespace {
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
uint32_t get32(const uint8_t* p) {
    return p[0] | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
} // namespace

Settings::Settings(ParamStore* params, hal::Storage* storage) : _params(params), _storage(storage) {
    if (_params) _seen_revision = _params->revision();
}

size_t Settings::capacity() const {
    if (!_storage || _storage->size() < storage_layout::PARAMS_OFFSET + HEADER_LEN) return 0;
    return (_storage->size() - storage_layout::PARAMS_OFFSET - HEADER_LEN) / ENTRY_LEN;
}

Settings::LoadResult Settings::load() {
    LoadResult r;
    if (!available()) return r;
    const size_t base = storage_layout::PARAMS_OFFSET;
    uint8_t h[HEADER_LEN];
    if (!_storage->read(base, h, sizeof(h))) return r;
    const uint16_t version = static_cast<uint16_t>(h[4] | (h[5] << 8));
    const uint16_t count = static_cast<uint16_t>(h[6] | (h[7] << 8));
    if (get32(h) != MAGIC || version != VERSION || count > capacity()) return r;

    // Verify the whole block before applying any of it.
    uint32_t crc = 0;
    uint8_t e[ENTRY_LEN];
    for (size_t i = 0; i < count; ++i) {
        if (!_storage->read(base + HEADER_LEN + i * ENTRY_LEN, e, ENTRY_LEN)) return r;
        crc = hal::crc32(e, ENTRY_LEN, crc);
    }
    if (crc != get32(h + 8)) return r;
    r.found = true;

    // Apply in registration order (VEHICLE_TYPE before the tuning that goes into the vehicle).
    for (size_t idx = 0; idx < _params->count(); ++idx) {
        const ParamStore::Param* p = _params->at(idx);
        if (!p->persist) continue;
        for (size_t i = 0; i < count; ++i) {
            _storage->read(base + HEADER_LEN + i * ENTRY_LEN, e, ENTRY_LEN);
            if (std::strncmp(reinterpret_cast<const char*>(e), p->name, ParamStore::NAME_LEN) != 0) continue;
            float v;
            std::memcpy(&v, e + ParamStore::NAME_LEN, 4);
            if (_params->set(idx, v) == ParamStore::SetResult::OK) ++r.loaded;
            else ++r.rejected;
            break;
        }
    }
    // Names in storage that no registered (saved) parameter has.
    for (size_t i = 0; i < count; ++i) {
        _storage->read(base + HEADER_LEN + i * ENTRY_LEN, e, ENTRY_LEN);
        char name[ParamStore::NAME_LEN + 1] = {};
        std::memcpy(name, e, ParamStore::NAME_LEN);
        const int idx = _params->find(name);
        if (idx < 0 || !_params->at(static_cast<size_t>(idx))->persist) ++r.unknown;
    }
    // Loading is not a user change: don't schedule a save for it.
    _seen_revision = _params->revision();
    _save_pending = false;
    return r;
}

bool Settings::save() {
    if (!available()) return false;
    const size_t base = storage_layout::PARAMS_OFFSET;
    const size_t cap = capacity();
    uint16_t count = 0;
    uint32_t crc = 0;
    bool fits = true;
    for (size_t idx = 0; idx < _params->count(); ++idx) {
        const ParamStore::Param* p = _params->at(idx);
        const float v = _params->value(idx);
        if (!p->persist || v == p->default_value) continue;
        if (count >= cap) { fits = false; break; }
        uint8_t e[ENTRY_LEN] = {};
        std::memcpy(e, p->name, std::strlen(p->name));
        std::memcpy(e + ParamStore::NAME_LEN, &v, 4);
        if (!_storage->write(base + HEADER_LEN + count * ENTRY_LEN, e, ENTRY_LEN)) return false;
        crc = hal::crc32(e, ENTRY_LEN, crc);
        ++count;
    }
    uint8_t h[HEADER_LEN];
    put32(h, MAGIC);
    h[4] = VERSION & 0xFF;
    h[5] = VERSION >> 8;
    h[6] = static_cast<uint8_t>(count);
    h[7] = static_cast<uint8_t>(count >> 8);
    put32(h + 8, crc);
    if (!_storage->write(base, h, sizeof(h))) return false;
    _seen_revision = _params->revision();
    _save_pending = false;
    _error = !fits;
    return fits;
}

bool Settings::commit() {
    if (!_storage) return false;
    if (_storage->flush()) {
        ++_commits;
        _error = false;
        return true;
    }
    ++_commit_failures;
    _error = true;
    return false;
}

bool Settings::save_now() {
    const bool ok = save();
    return commit() && ok;
}

bool Settings::erase_saved() {
    if (!available()) return false;
    uint8_t h[HEADER_LEN];
    std::memset(h, 0xFF, sizeof(h));
    if (!_storage->write(storage_layout::PARAMS_OFFSET, h, sizeof(h))) return false;
    _seen_revision = _params->revision();
    _save_pending = false;
    return commit();
}

void Settings::update(bool armed, uint32_t now_ms) {
    if (!available()) return;
    if (_params->revision() != _seen_revision) {
        _seen_revision = _params->revision();
        _save_pending = true;
        _change_ms = now_ms;
    }
    if (armed) return;   // never stall the CPU with a flash erase in flight
    if (_save_pending && now_ms - _change_ms >= save_delay_ms) save();
    // Commit anything waiting (our parameters, or another region such as the signing key).
    if (!_save_pending && _storage->dirty() &&
        (!_attempted || now_ms - _last_commit_attempt_ms >= retry_ms)) {
        _last_commit_attempt_ms = now_ms;
        _attempted = !commit();   // rate-limit retries only after a failure
    }
}

} // namespace comms
} // namespace autopilot
