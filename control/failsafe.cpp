#include "failsafe.hpp"

namespace autopilot {
namespace control {

void Failsafe::reset() {
    _action = FailsafeAction::NONE;
    _reasons = FS_NONE;
    _battery_latched = false;
    _batt_low_timing = false;
    _batt_low_since_ms = 0;
    _committed_land = false;
}

FailsafeAction Failsafe::update(const FailsafeInputs& in) {
    uint8_t reasons = FS_NONE;

    // RC link: armed without ever hearing from the pilot also counts as lost.
    if (!in.rc_ever_received || (in.now_ms - in.last_rc_ms) > _cfg.rc_timeout_ms) {
        reasons |= FS_RC_LOSS;
    }

    // Battery: debounce sag under load, then latch (voltage recovers when load drops).
    if (_cfg.battery_low_v > 0.0f && in.battery_voltage >= 0.0f) {
        if (in.battery_voltage < _cfg.battery_low_v) {
            if (!_batt_low_timing) {
                _batt_low_timing = true;
                _batt_low_since_ms = in.now_ms;
            } else if (in.now_ms - _batt_low_since_ms >= _cfg.battery_debounce_ms) {
                _battery_latched = true;
            }
        } else {
            _batt_low_timing = false;
        }
    }
    if (_battery_latched) reasons |= FS_BATTERY;

    if (!in.attitude_valid) reasons |= FS_ESTIMATOR;
    if (!in.imu_usable) reasons |= FS_NO_IMU;

    FailsafeAction act = FailsafeAction::NONE;
    if (reasons & FS_NO_IMU) {
        act = FailsafeAction::TERMINATE;
    } else if (reasons & (FS_RC_LOSS | FS_BATTERY | FS_ESTIMATOR)) {
        act = FailsafeAction::LAND;
    }

    // Battery / estimator landings are committed: they don't cancel if the
    // condition clears. Only a pure RC loss gives control back on RC recovery.
    if (act == FailsafeAction::LAND && (reasons & (FS_BATTERY | FS_ESTIMATOR))) {
        _committed_land = true;
    }
    if (_committed_land && act == FailsafeAction::NONE) {
        act = FailsafeAction::LAND;
    }
    if (_action == FailsafeAction::TERMINATE) {
        act = FailsafeAction::TERMINATE; // never relax a termination while armed
    }

    _reasons = reasons;
    _action = act;
    return _action;
}

} // namespace control
} // namespace autopilot
