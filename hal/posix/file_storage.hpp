#pragma once
/**
 * Settings storage in a file, for SITL and desktop builds (the SITL's
 * "EEPROM"). The file is loaded at open(); flush() writes a temporary file and
 * renames it over the old one, so a crash mid-write never leaves a half file.
 */
#include "hal/hal.hpp"
#include <string>
#include <vector>

namespace autopilot {
namespace hal {

class FileStorage : public Storage {
public:
    explicit FileStorage(std::string path, size_t size = 4096) : _path(std::move(path)), _data(size, 0xFF) {}

    /** Load the file (missing file = blank storage). False if it exists but can't be read. */
    bool open();
    /** Forget all contents (and delete the file). */
    bool wipe();

    size_t size() const override { return _data.size(); }
    bool read(size_t offset, uint8_t* data, size_t len) override;
    bool write(size_t offset, const uint8_t* data, size_t len) override;
    bool flush() override;
    bool dirty() const override { return _dirty; }

    const std::string& path() const { return _path; }
    uint32_t flush_count() const { return _flushes; }

private:
    std::string _path;
    std::vector<uint8_t> _data;
    bool _dirty{false};
    uint32_t _flushes{0};
};

} // namespace hal
} // namespace autopilot
