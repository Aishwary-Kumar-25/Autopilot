#pragma once

#include "control/vehicle_mixer.hpp"
#include "hal/hal.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include <string>

namespace autopilot {
namespace control {

/**
 * @brief Configuration and allocation coefficients for a single actuator.
 * Represents a row in the Control Allocation Matrix B:
 *   u_i = roll*C_roll + pitch*C_pitch + yaw*C_yaw + heave*C_heave + surge*C_surge + sway*C_sway
 */
struct ActuatorConfig {
    float roll{0.0f};       // Contribution to Roll torque
    float pitch{0.0f};      // Contribution to Pitch torque
    float yaw{0.0f};        // Contribution to Yaw torque
    float heave{0.0f};      // Contribution to Heave (vertical thrust / throttle)
    float surge{0.0f};      // Contribution to Surge (forward/backward thrust)
    float sway{0.0f};       // Contribution to Sway (lateral/sideways thrust)

    bool bidirectional{true};  // true: ROV/AUV reversible thrusters (1500us center), false: UAV unidirectional ESC (1000us min)
    bool reversed{false};       // Invert motor spin / thrust direction
    uint16_t min_pwm{1000};     // Microseconds
    uint16_t max_pwm{2000};     // Microseconds
    uint16_t neutral_pwm{1500}; // Center / stopped PWM (1500 for bidirectional, 1000 for unidirectional)
    bool enabled{true};
};

/**
 * @brief Standard built-in vehicle geometry presets.
 */
enum class MixerPreset {
    CUSTOM,
    // ROV / Submarine Presets
    ROV_3_SIMPLE,       // 3 Thrusters: 2 Differential Horizontal + 1 Vertical Heave
    ROV_4_BASIC,        // 4 Thrusters: 2 Differential Horizontal + 2 Vertical (Roll + Heave)
    ROV_6_VECTORED,     // 6 Thrusters: 4 Vectored 45-deg Horizontal + 2 Vertical (ArduSub BlueROV2)
    ROV_8_HEAVY,        // 8 Thrusters: 4 Vectored Horizontal + 4 Vectored Vertical (Decoupled 6-DOF BlueROV2 Heavy & RoboSub AUV)

    // UAV Multicopter Presets
    UAV_TRI,            // 3 Rotors Tricopter
    UAV_QUAD_X,         // 4 Rotors Quad-X
    UAV_HEXA_X,         // 6 Rotors Hexa-X
    UAV_OCTA_X,         // 8 Rotors Octa-X
    UAV_DODECA_12,      // 12 Rotors Coaxial / Heavy-Lift
    UAV_HEXADEC_16      // 16 Rotors High-Redundancy Heavy-Lift
};

/**
 * What the mixer can still do after one or more motors were declared failed
 * (see ActuatorMatrixMixer::set_motor_failed()).
 */
enum class AllocationStatus : uint8_t {
    NOMINAL = 0,      // every motor healthy, original allocation
    DEGRADED = 1,     // motor(s) lost, roll/pitch/thrust/yaw re-allocated to the rest
                      // (yaw authority usually reduced: it is given up first)
    NO_YAW = 2,       // motor(s) lost, yaw is uncontrollable: roll/pitch/thrust only
                      // (e.g. a quad on 3 motors: it spins, but can stay upright and land)
    UNCONTROLLABLE = 3 // not enough motors left to hold roll/pitch and thrust
};

const char* allocation_status_name(AllocationStatus s);

/**
 * @brief Generalized Actuator Matrix Mixer supporting 2 to 16 thrusters/motors.
 * Replaces fixed hardcoded mixers with dynamic control allocation:
 *   u = B * tau
 * 
 * Features:
 * - Dynamic runtime configuration: 2 to 16 actuators
 * - Runtime interface to choose number of thrusters / motors
 * - Desaturation and dynamic scaling to prevent clipping and loss of attitude control
 * - Support for both bidirectional (ROV) and unidirectional (UAV) ESCs
 * - Built-in DroneCAN ESC command generation (UAVCAN RawCommand DSDL 1030)
 */
class ActuatorMatrixMixer : public VehicleMixer {
public:
    ActuatorMatrixMixer();
    explicit ActuatorMatrixMixer(MixerPreset preset);
    ~ActuatorMatrixMixer() override = default;

    /**
     * @brief Load one of the standard geometry presets.
     */
    bool load_preset(MixerPreset preset);

    /**
     * @brief Quick configure for ROV/AUV by specifying the number of thrusters (2 to 16).
     * Automatically assigns the optimal layout preset or symmetric geometry.
     */
    bool configure_rov(uint8_t num_thrusters);

    /**
     * @brief Quick configure for UAV multicopter by specifying number of rotors (3 to 16).
     */
    bool configure_uav(uint8_t num_rotors);

    /**
     * @brief Configure custom actuator row in allocation matrix.
     */
    bool set_actuator_config(uint8_t channel, const ActuatorConfig& config);
    const ActuatorConfig& get_actuator_config(uint8_t channel) const;
    ActuatorConfig& get_actuator_config_mut(uint8_t channel);

    /**
     * @brief Set number of active channels (2 to 16).
     */
    void set_num_channels(uint8_t count);
    uint8_t get_num_channels() const override { return _num_channels; }

    MixerPreset get_current_preset() const { return _current_preset; }
    const char* get_preset_name() const;

    /**
     * @brief Compute actuator outputs and write directly to PWM hardware.
     */
    void output_to_actuators(const ControlDemands& demands, hal::PWMOutput* pwm, bool armed) override;

    /**
     * @brief Compute normalized outputs [-1.0, 1.0] or [0.0, 1.0] for external consumers.
     */
    void get_normalized_outputs(const ControlDemands& demands, float* out_values, size_t max_len) const;

    // ---------------- Motor failure: control re-allocation ----------------

    /**
     * Declare multirotor motor `channel` failed (dead ESC/motor, lost propeller)
     * or healthy again, and re-solve the allocation for the motors that are left.
     *
     * The re-allocation keeps the healthy torque/thrust per unit demand, so the
     * attitude and altitude loops need no re-tuning:
     *   rows' = E^T (E E^T)^-1 G
     * where E is the remaining motors' effectiveness (their nominal rows) and G
     * the healthy frame's Gram matrix R^T R. Roll, pitch and yaw come from the
     * full 4-axis solve; the collective comes from a roll/pitch/thrust-only
     * solve, so hovering never depends on yaw (yaw is what gets given up). When
     * the 4-axis problem is singular (a quad on 3 motors) yaw is dropped.
     *
     * Only for unidirectional (multirotor) outputs. Returns false if the channel
     * is out of range or not a unidirectional motor.
     */
    bool set_motor_failed(uint8_t channel, bool failed = true);
    /** Back to the original allocation with every motor healthy. */
    void clear_motor_failures();
    uint16_t failed_motor_mask() const { return _failed_mask; }
    bool is_motor_failed(uint8_t channel) const {
        return channel < hal::PWMOutput::MAX_CHANNELS && (_failed_mask & (1u << channel));
    }
    AllocationStatus allocation_status() const { return _alloc_status; }
    /** Healthy-original row of a motor (before any re-allocation). */
    const ActuatorConfig& get_nominal_config(uint8_t channel) const;
    /**
     * Highest collective demand the remaining motors can deliver (1.0 when
     * healthy; e.g. ~0.67 for a hexa on 5 motors: hover needs to stay below it).
     */
    float collective_limit() const { return _collective_limit; }

    /** Normalised outputs of the last output_to_actuators() call (0 when disarmed). */
    float last_output(uint8_t channel) const {
        return channel < hal::PWMOutput::MAX_CHANNELS ? _last_outputs[channel] : 0.0f;
    }

    /**
     * @brief Generate DroneCAN RawCommand frame (DSDL 1030) for CAN-based ESCs.
     */
    bool generate_dronecan_esc_frame(const ControlDemands& demands, uint8_t& transfer_id, hal::CANFrame& out_frame, bool armed) const;

    /**
     * @brief Transmit DroneCAN RawCommand to hardware CAN bus.
     */
    bool send_dronecan_esc_commands(const ControlDemands& demands, hal::CANDevice* can, uint8_t& transfer_id, bool armed) const;

private:
    uint8_t _num_channels{0};
    MixerPreset _current_preset{MixerPreset::CUSTOM};
    std::array<ActuatorConfig, hal::PWMOutput::MAX_CHANNELS> _actuators;

    // Motor failure handling: _nominal holds the healthy rows while any motor is failed.
    std::array<ActuatorConfig, hal::PWMOutput::MAX_CHANNELS> _nominal;
    uint16_t _failed_mask{0};
    AllocationStatus _alloc_status{AllocationStatus::NOMINAL};
    float _collective_limit{1.0f};
    float _last_outputs[hal::PWMOutput::MAX_CHANNELS]{};
    void reset_failures();
    void reallocate();

    void setup_rov_3_simple();
    void setup_rov_4_basic();
    void setup_rov_6_vectored();
    void setup_rov_8_heavy();
    void setup_rov_n_symmetric(uint8_t n);

    void setup_uav_tri();
    void setup_uav_quad_x();
    void setup_uav_hexa_x();
    void setup_uav_octa_x();
    void setup_uav_n_symmetric(uint8_t n);
};

} // namespace control
} // namespace autopilot

