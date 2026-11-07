#pragma once
/**
 * Failsafe manager: decides what the vehicle must do when something goes wrong.
 *
 * Vehicle-agnostic: it only reports an action. Each vehicle maps the action to
 * its own behaviour (multicopter lands then disarms, rover/sub go to neutral).
 *
 * Triggers (most severe action wins, and an action never relaxes while armed
 * except RC recovery from RC_LOSS):
 *   - RC link loss        : no set_user_inputs() for RC_TIMEOUT_MS
 *   - Low battery          : voltage below the threshold for BATT_DEBOUNCE_MS (latched)
 *   - Attitude estimate    : attitude_valid false
 *   - No usable IMU        : nothing left to fly on -> TERMINATE (motors off)
 */
#include <cstdint>

namespace autopilot {
namespace control {

enum class FailsafeAction : uint8_t {
    NONE = 0,
    LAND = 1,       // UAV: controlled descent then disarm; UGV/ROV: stop (neutral)
    TERMINATE = 2,  // disarm now (no IMU: any output is worse than none)
};

enum FailsafeReason : uint8_t {
    FS_NONE = 0,
    FS_RC_LOSS = 1 << 0,
    FS_BATTERY = 1 << 1,
    FS_ESTIMATOR = 1 << 2,
    FS_NO_IMU = 1 << 3,
};

struct FailsafeInputs {
    uint32_t now_ms{0};
    uint32_t last_rc_ms{0};
    bool rc_ever_received{false};
    float battery_voltage{-1.0f};  // < 0 means "no battery monitor"
    bool attitude_valid{false};
    bool imu_usable{false};
};

class Failsafe {
public:
    struct Config {
        uint32_t rc_timeout_ms{500};
        float battery_low_v{0.0f};        // 0 disables the battery failsafe
        uint32_t battery_debounce_ms{2000};
    };

    Failsafe() = default;
    explicit Failsafe(const Config& c) : _cfg(c) {}

    void set_config(const Config& c) { _cfg = c; }
    const Config& config() const { return _cfg; }

    /** Call on arm/disarm: clears latches. */
    void reset();

    FailsafeAction update(const FailsafeInputs& in);

    FailsafeAction action() const { return _action; }
    uint8_t reasons() const { return _reasons; }
    bool active() const { return _action != FailsafeAction::NONE; }

private:
    Config _cfg;
    FailsafeAction _action{FailsafeAction::NONE};
    uint8_t _reasons{FS_NONE};
    bool _battery_latched{false};
    bool _batt_low_timing{false};
    uint32_t _batt_low_since_ms{0};
    bool _committed_land{false};
};

} // namespace control
} // namespace autopilot
