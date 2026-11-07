#include "comms/params.hpp"
#include <cmath>
#include <cstring>

namespace autopilot {
namespace comms {

bool ParamStore::valid_name(const char* name) {
    if (!name || !name[0]) return false;
    return std::strlen(name) <= NAME_LEN;
}

int ParamStore::add(const char* name, Type type, float default_value, float min, float max,
                    std::function<bool(float)> on_set) {
    if (!valid_name(name) || find(name) >= 0 || min > max) return -1;
    Param p;
    std::strncpy(p.name, name, NAME_LEN);
    p.type = type;
    p.default_value = (type == Type::INT32) ? std::round(default_value) : default_value;
    p.value = p.default_value;
    p.min = min;
    p.max = max;
    p.on_set = std::move(on_set);
    _params.push_back(std::move(p));
    return static_cast<int>(_params.size() - 1);
}

int ParamStore::add_live(const char* name, Type type, float min, float max, std::function<float()> get,
                         std::function<bool(float)> on_set) {
    if (!get) return -1;
    const int idx = add(name, type, 0.0f, min, max, std::move(on_set));
    if (idx >= 0) {
        _params[idx].get = std::move(get);
        _params[idx].default_value = _params[idx].get();
    }
    return idx;
}

int ParamStore::find(const char* name) const {
    if (!name) return -1;
    for (size_t i = 0; i < _params.size(); ++i) {
        if (std::strncmp(_params[i].name, name, NAME_LEN) == 0) return static_cast<int>(i);
    }
    return -1;
}

float ParamStore::value(size_t index) const {
    if (index >= _params.size()) return 0.0f;
    const Param& p = _params[index];
    return p.get ? p.get() : p.value;
}

ParamStore::SetResult ParamStore::set(size_t index, float v) {
    if (index >= _params.size()) return SetResult::NOT_FOUND;
    Param& p = _params[index];
    if (!std::isfinite(v)) return SetResult::OUT_OF_RANGE;
    if (p.type == Type::INT32) v = std::round(v);
    if (v < p.min || v > p.max) return SetResult::OUT_OF_RANGE;
    if (p.on_set && !p.on_set(v)) return SetResult::REJECTED;
    p.value = v;
    ++_revision;
    return SetResult::OK;
}

ParamStore::SetResult ParamStore::set(const char* name, float v) {
    const int idx = find(name);
    return idx < 0 ? SetResult::NOT_FOUND : set(static_cast<size_t>(idx), v);
}

void ParamStore::apply_all() {
    for (Param& p : _params) {
        if (!p.get && p.on_set) p.on_set(p.value);
    }
}

size_t ParamStore::reset_to_defaults() {
    size_t refused = 0;
    for (size_t i = 0; i < _params.size(); ++i) {
        const Param& p = _params[i];
        if (!p.persist || value(i) == p.default_value) continue;
        if (set(i, p.default_value) != SetResult::OK) ++refused;
    }
    return refused;
}

} // namespace comms
} // namespace autopilot
