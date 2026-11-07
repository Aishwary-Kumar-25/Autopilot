#pragma once
/**
 * Named, typed parameters that a ground station can list, read and set
 * (MAVLink parameter protocol). Names are at most 16 characters.
 *
 * A parameter is either
 *   - stored : the store owns the value; on_set pushes it into the flight code
 *              and apply_all() re-pushes every stored value (call it after the
 *              active vehicle object changes, so tuning survives a profile switch)
 *   - live   : get() reads the value from the flight code each time (e.g. the
 *              active vehicle type, which other code can change too)
 * on_set returns false to refuse a change (e.g. "only while disarmed"); the
 * stored value then stays as it was.
 */
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace autopilot {
namespace comms {

class ParamStore {
public:
    enum class Type : uint8_t { FLOAT, INT32 };
    static constexpr size_t NAME_LEN = 16;

    struct Param {
        char name[NAME_LEN + 1]{};
        Type type{Type::FLOAT};
        float value{0.0f};
        float default_value{0.0f};
        float min{-1e9f}, max{1e9f};
        std::function<bool(float)> on_set;
        std::function<float()> get;  // set => live parameter
    };

    /** Add a stored parameter. Returns its index, or -1 (bad/duplicate name). */
    int add(const char* name, Type type, float default_value, float min, float max,
            std::function<bool(float)> on_set = nullptr);
    /** Add a live parameter read through get(). */
    int add_live(const char* name, Type type, float min, float max, std::function<float()> get,
                 std::function<bool(float)> on_set);

    size_t count() const { return _params.size(); }
    int find(const char* name) const;   // compares at most NAME_LEN chars; -1 if absent
    const Param* at(size_t index) const { return index < _params.size() ? &_params[index] : nullptr; }
    float value(size_t index) const;

    enum class SetResult : uint8_t { OK, NOT_FOUND, OUT_OF_RANGE, REJECTED };
    SetResult set(size_t index, float v);
    SetResult set(const char* name, float v);

    /** Re-run on_set for every stored parameter (live ones are skipped). */
    void apply_all();
    /** Bumped on every successful change (lets the GCS link notice local changes). */
    uint32_t revision() const { return _revision; }

private:
    std::vector<Param> _params;
    uint32_t _revision{0};
    static bool valid_name(const char* name);
};

} // namespace comms
} // namespace autopilot
