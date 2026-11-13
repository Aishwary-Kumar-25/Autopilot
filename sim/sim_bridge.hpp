#pragma once
/**
 * Simulator bridge: a UDP side channel of autopilot_sitl for test harnesses,
 * ROS 2 (ros2/autopilot_sim) and data collection. Test/SITL only.
 *
 * Telemetry (SITL -> client): one JSON object per datagram at --bridge-rate Hz
 * of simulated time, with BOTH the true state of the simulated vehicle and
 * what the firmware believes (its estimate), the raw sensor readings, PWM
 * outputs, failsafe state and the faults that are active right now. Truth +
 * sensors + labels in one record is what a dataset for estimator / fault
 * detection work needs.
 *
 * Commands (client -> SITL): one text line per datagram, answered with an
 * {"type":"ack"} datagram. Prefix any command with "at <t>" to run it at
 * simulated time t (seconds since start), which makes scripted scenarios
 * exact and independent of how fast the simulation runs. See
 * docs/ROS2_SIMULATOR.md for the full list.
 *
 * Frames: NED position/velocity, FRD body axes, angles in degrees in "rpy".
 */
#include "sim/sim_battery.hpp"
#include "sim/sim_world.hpp"
#include "sim/udp_socket.hpp"
#include "vehicles/vehicle_manager.hpp"
#include <memory>
#include <string>
#include <vector>

namespace autopilot {
namespace sim {

/** Simulated environment and pilot input the bridge can change. */
struct SimEnv {
    V3 wind;                       // NED, m/s (multirotor)
    V3 current;                    // NED, m/s (boat)
    double motor_health[16];       // per output, 1 = healthy, 0 = dead
    double quad_mass{0.0};         // 0 = model default
    double quad_max_thrust{0.0};   // 0 = model default
    // Pilot sticks from the bridge (override joystick / --rc-neutral while active).
    bool sticks_active{false};
    float sticks[6]{0, 0, 0, 0, 0, 0};   // roll, pitch, yaw, throttle, forward, lateral
    bool rc_lost{false};           // stop feeding any pilot input -> RC-loss failsafe
    bool quit{false};              // "quit" received
    SimEnv() { for (double& h : motor_health) h = 1.0; }
};

class SimBridge {
public:
    struct Sensors {
        std::shared_ptr<SimIMU> imu[2];
        std::shared_ptr<SimBaro> baro;
        std::shared_ptr<SimCompass> mag;
        std::shared_ptr<SimGPS> gps;
    };

    SimBridge(vehicles::VehicleManager& vm, SimWorld& world, SimEnv& env, Sensors sensors, SimBattery& battery)
        : _vm(vm), _world(world), _env(env), _s(std::move(sensors)), _bat(battery) {}

    /** Bind the command port (any_interface: accept clients from other machines / WSL host). */
    bool open(uint16_t port, double rate_hz, bool any_interface = false);
    const std::string& error() const { return _sock.error(); }
    uint16_t port() const { return _sock.port(); }

    /**
     * Call once per flight-loop tick, AFTER vm.step(): runs received and
     * scheduled commands, then sends telemetry when it is due.
     * t = simulated seconds since start.
     */
    void update(double t);

    /** Apply wind / current / airframe overrides to the current physics body (also after a swap). */
    void apply_env();

    /** Run one command line now; returns false (with msg) if it was refused. */
    bool execute(const std::string& line, std::string& msg);

    /** Comma separated labels of the faults active now, e.g. "gps:dropout,motor3:0.00". */
    std::string active_faults() const;

    uint32_t telemetry_sent() const { return _sent; }
    /** True once a client has sent anything (telemetry goes to the last sender). */
    bool has_client() const { return _sock.has_peer(); }
    /**
     * True once the client said "start" (or sent any command that acts now, i.e.
     * not hello / ping / at). The SITL holds simulated time at 0 until then, so a
     * script can queue all its "at" commands first, even at --speedup 0.
     */
    bool started() const { return _started; }

private:
    vehicles::VehicleManager& _vm;
    SimWorld& _world;
    SimEnv& _env;
    Sensors _s;
    SimBattery& _bat;
    UdpSocket _sock;
    double _period{0.02}, _next_t{0.0}, _t{0.0};
    uint32_t _sent{0};
    struct Scheduled { double t; std::string cmd; uint64_t seq; };
    std::vector<Scheduled> _queue;
    uint64_t _seq{0};
    bool _started{false};

    void handle_line(const std::string& line);
    void send_ack(const std::string& cmd, bool ok, const std::string& msg);
    void send_telemetry();
};

} // namespace sim
} // namespace autopilot
