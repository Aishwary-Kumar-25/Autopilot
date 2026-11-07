#pragma once
/**
 * MAVLink v2 ground-station link (QGroundControl, Mission Planner, MAVProxy, pymavlink).
 *
 * Transport-agnostic: it reads and writes bytes through a hal::UARTDevice, so
 * the same code runs over a telemetry radio on a real UART or over UDP in SITL
 * (hal/posix/udp_link.hpp). Call update() from the main loop at 50-400 Hz; it
 * parses everything received, answers it, and sends telemetry streams when due.
 *
 * Sends   HEARTBEAT 1 Hz, SYS_STATUS 1 Hz, ATTITUDE 10 Hz, VFR_HUD 4 Hz,
 *         GLOBAL_POSITION_INT 5 Hz + GPS_RAW_INT 2 Hz (when a GPS exists),
 *         SERVO_OUTPUT_RAW 2 Hz (the PWM pins), STATUSTEXT on events
 *         (arm/disarm, failsafe, sensor health, profile switch, refused commands).
 * Handles HEARTBEAT (GCS presence), PARAM_REQUEST_LIST / _READ / PARAM_SET,
 *         COMMAND_LONG + COMMAND_INT: ARM_DISARM, DO_SET_MODE, SET_MESSAGE_INTERVAL,
 *         REQUEST_MESSAGE, REQUEST_AUTOPILOT_CAPABILITIES (others -> UNSUPPORTED),
 *         SET_MODE, MANUAL_CONTROL (joystick / QGC virtual joystick -> RC input),
 *         MISSION_REQUEST_LIST (answers "0 items"), TIMESYNC, PING.
 *
 * Safety rules: disarming a multicopter that is not landed needs the force
 * flag (param2 = 21196); vehicle profile / frame changes are refused while armed.
 */
#include "comms/mavlink/mavlink_messages.hpp"
#include "comms/params.hpp"
#include "hal/hal.hpp"
#include "vehicles/vehicle_manager.hpp"
#include <cstdint>

namespace autopilot {
namespace comms {

class GcsLink {
public:
    struct Config {
        uint8_t sysid{1};
        uint8_t compid{mavlink::MAV_COMP_ID_AUTOPILOT1};
        uint32_t gcs_timeout_ms{3000};
        bool accept_manual_control{true}; // false when a real RC receiver is the pilot input
        size_t params_per_update{10};     // PARAM_VALUE burst size while streaming the list
    };

    struct Stats {
        uint32_t rx_messages{0};
        uint32_t rx_crc_errors{0};
        uint32_t rx_lost{0};
        uint32_t tx_messages{0};
        uint32_t manual_control_msgs{0};
    };

    GcsLink(hal::UARTDevice* port, vehicles::VehicleManager* vm, ParamStore* params, const Config& cfg);
    GcsLink(hal::UARTDevice* port, vehicles::VehicleManager* vm, ParamStore* params)
        : GcsLink(port, vm, params, Config{}) {}
    ~GcsLink();
    GcsLink(const GcsLink&) = delete;
    GcsLink& operator=(const GcsLink&) = delete;

    /** Receive + handle + stream. Uses the HAL clock. */
    void update();

    /** Queue a STATUSTEXT (truncated to 50 chars). Safe to call from anywhere. */
    void send_statustext(mavlink::MavSeverity severity, const char* text);

    /** Battery monitor values for SYS_STATUS (voltage < 0 = no monitor). */
    void set_battery(float voltage_v, float current_a = -1.0f, int remaining_pct = -1);

    /** Change a telemetry stream rate. interval_ms 0 disables. Returns false if not a stream. */
    bool set_stream_interval(uint32_t msgid, uint32_t interval_ms);

    bool gcs_connected() const { return _gcs_seen && (now() - _last_gcs_heartbeat_ms) < _cfg.gcs_timeout_ms; }
    const Stats& stats() const { return _stats; }
    const Config& config() const { return _cfg; }

private:
    hal::UARTDevice* _port;
    vehicles::VehicleManager* _vm;
    ParamStore* _params;
    Config _cfg;
    Stats _stats;
    mavlink::Parser _parser;
    uint8_t _tx_seq{0};

    // GCS identity (learned from its HEARTBEAT / first message)
    bool _gcs_seen{false};
    uint32_t _last_gcs_heartbeat_ms{0};
    uint8_t _gcs_sysid{255}, _gcs_compid{190};

    // Telemetry streams
    struct Stream { uint32_t msgid; uint32_t default_ms; uint32_t interval_ms; uint32_t last_ms; };
    static constexpr size_t NUM_STREAMS = 7;
    Stream _streams[NUM_STREAMS];

    // Parameter list streaming
    bool _param_streaming{false};
    size_t _param_next{0};

    // Status text ring
    static constexpr size_t TEXT_QUEUE = 8;
    struct Text { uint8_t severity; char text[51]; };
    Text _texts[TEXT_QUEUE]{};
    size_t _text_head{0}, _text_count{0};

    // Event detection
    const vehicles::VehicleBase* _last_vehicle{nullptr};
    bool _last_armed{false};
    control::FailsafeAction _last_fs{control::FailsafeAction::NONE};
    bool _sensor_callback_installed{false};

    // Battery
    float _batt_v{-1.0f}, _batt_a{-1.0f};
    int _batt_pct{-1};

    uint32_t now() const;
    void receive();
    void handle(const mavlink::Message& msg);
    void handle_command(uint16_t command, const float p[7], uint8_t src_sys, uint8_t src_comp);
    void handle_manual_control(const mavlink::ManualControl& mc);
    bool set_custom_mode(uint32_t custom_mode);
    bool set_base_mode(uint8_t base_mode, uint32_t custom_mode);
    void detect_events();
    void stream_telemetry(uint32_t now_ms);
    void stream_params();
    void flush_texts();
    bool send_message_by_id(uint32_t msgid);   // one-shot (REQUEST_MESSAGE)
    bool for_us(uint8_t target_sys, uint8_t target_comp) const;

    template <typename Msg> void send(const Msg& m) {
        uint8_t frame[mavlink::MAX_FRAME];
        const size_t n = mavlink::pack(frame, _tx_seq++, _cfg.sysid, _cfg.compid, m);
        if (_port) _port->write(frame, n);
        ++_stats.tx_messages;
    }

    // Message builders
    void send_heartbeat();
    void send_sys_status();
    void send_attitude();
    void send_vfr_hud();
    void send_global_position();
    void send_gps_raw();
    void send_servo_output();
    void send_autopilot_version();
    void send_param(size_t index);
    void send_ack(uint16_t command, uint8_t result, uint8_t target_sys, uint8_t target_comp);

    uint8_t mav_type() const;
    const drivers::GPSSensor* telemetry_gps() const;
    uint32_t custom_mode() const;
};

} // namespace comms
} // namespace autopilot
