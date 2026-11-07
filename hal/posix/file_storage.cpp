#include "hal/posix/file_storage.hpp"
#include <cstdio>
#include <cstring>

namespace autopilot {
namespace hal {

bool FileStorage::open() {
    std::fill(_data.begin(), _data.end(), 0xFF);
    _dirty = false;
    std::FILE* f = std::fopen(_path.c_str(), "rb");
    if (!f) return true;   // first run: blank storage
    const size_t n = std::fread(_data.data(), 1, _data.size(), f);
    const bool err = std::ferror(f) != 0;
    std::fclose(f);
    if (err) return false;
    if (n < _data.size()) std::fill(_data.begin() + static_cast<long>(n), _data.end(), 0xFF);
    return true;
}

bool FileStorage::wipe() {
    std::fill(_data.begin(), _data.end(), 0xFF);
    _dirty = false;
    std::remove(_path.c_str());
    return true;
}

bool FileStorage::read(size_t offset, uint8_t* data, size_t len) {
    if (offset > _data.size() || len > _data.size() - offset) return false;
    std::memcpy(data, _data.data() + offset, len);
    return true;
}

bool FileStorage::write(size_t offset, const uint8_t* data, size_t len) {
    if (offset > _data.size() || len > _data.size() - offset) return false;
    if (std::memcmp(_data.data() + offset, data, len) != 0) {
        std::memcpy(_data.data() + offset, data, len);
        _dirty = true;
    }
    return true;
}

bool FileStorage::flush() {
    if (!_dirty) return true;
    const std::string tmp = _path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(_data.data(), 1, _data.size(), f) == _data.size() && std::fflush(f) == 0;
    std::fclose(f);
    if (!ok || std::rename(tmp.c_str(), _path.c_str()) != 0) {
        std::remove(tmp.c_str());
        return false;
    }
    _dirty = false;
    ++_flushes;
    return true;
}

} // namespace hal
} // namespace autopilot
