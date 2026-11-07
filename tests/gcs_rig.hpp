#pragma once
/**
 * Flight stack + GcsLink + a scripted ground station over an in-memory radio,
 * for tests of the link features (signing, saved settings). The GCS side can
 * sign its frames with its own mavlink::Signing.
 */
#include "comms/gcs_link.hpp"
#include "comms/mavlink/mavlink_messages.hpp"
#include "comms/mavlink/mavlink_signing.hpp"
#include "comms/settings.hpp"
#include "comms/vehicle_params.hpp"
#include "hal/posix/posix_hal.hpp"
#include "sim/quad_sim.hpp"
#include "vehicles/vehicle_manager.hpp"

#include <algorithm>
#include <cstring>
#include <memory>
#include <vector>

namespace rig {

using namespace autopilot;
using namespace autopilot::comms;
using namespace autopilot::comms::mavlink;

class FakeRadio : public hal::UARTDevice {
public:
    std::vector<uint8_t> to_vehicle, from_vehicle;
    bool init(uint32_t) override { return true; }
    size_t write(const uint8_t* d, size_t n) override { from_vehicle.insert(from_vehicle.end(), d, d + n); return n; }
    size_t read(uint8_t* buf, size_t max) override {
        const size_t n = std::min(max, to_vehicle.size());
        std::memcpy(buf, to_vehicle.data(), n);
        to_vehicle.erase(to_vehicle.begin(), to_vehicle.begin() + static_cast<long>(n));
        return n;
    }
    size_t available() override { return to_vehicle.size(); }
};

struct RxFrame { Message msg; std::vector<uint8_t> raw; };

struct Gcs {
    FakeRadio radio;
    Parser parser;
    uint8_t seq{0};
    std::vector<RxFrame> inbox;
    Signing signing;                 // set a key to sign what the GCS sends
    std::vector<uint8_t> last_sent;  // raw bytes of the last frame sent (for replay tests)

    template <typename Msg> void send(const Msg& m) {
        uint8_t f[MAX_FRAME];
        const bool sign = signing.enabled();
        size_t n = pack(f, seq++, 255, 190, m, sign ? INCOMPAT_FLAG_SIGNED : 0);
        if (sign) n = signing.sign(f, n);
        last_sent.assign(f, f + n);
        radio.to_vehicle.insert(radio.to_vehicle.end(), f, f + n);
    }
    void send_raw(const std::vector<uint8_t>& f) { radio.to_vehicle.insert(radio.to_vehicle.end(), f.begin(), f.end()); }
    void drain() {
        for (uint8_t b : radio.from_vehicle) {
            if (parser.parse(b)) inbox.push_back({parser.msg(), {parser.frame(), parser.frame() + parser.frame_len()}});
        }
        radio.from_vehicle.clear();
    }
    size_t count(uint32_t id) const {
        size_t c = 0;
        for (const auto& f : inbox) c += (f.msg.msgid == id);
        return c;
    }
    bool has_text(const char* needle) const {
        for (const auto& f : inbox) {
            if (f.msg.msgid == StatusText::ID && std::strstr(StatusText::decode(f.msg.payload).text, needle)) return true;
        }
        return false;
    }
    void command(uint16_t cmd, float p1 = 0, float p2 = 0) {
        CommandLong c;
        c.command = cmd; c.target_system = 1; c.target_component = 1; c.param[0] = p1; c.param[1] = p2;
        send(c);
    }
    int ack_result(uint16_t cmd) const {
        for (auto it = inbox.rbegin(); it != inbox.rend(); ++it) {
            if (it->msg.msgid == CommandAck::ID) {
                const auto a = CommandAck::decode(it->msg.payload);
                if (a.command == cmd) return a.result;
            }
        }
        return -1;
    }
    void param_set(const char* name, float v, uint8_t type = MAV_PARAM_TYPE_REAL32) {
        ParamSet s;
        s.target_system = 1; s.target_component = 1; std::strcpy(s.param_id, name); s.param_type = type;
        if (type == MAV_PARAM_TYPE_INT32) { const int32_t i = static_cast<int32_t>(v); std::memcpy(&s.param_value, &i, 4); }
        else s.param_value = v;
        send(s);
    }
    void setup_signing(const uint8_t key[32], uint64_t ts) {
        SetupSigning s;
        s.target_system = 1; s.target_component = 1; s.initial_timestamp = ts;
        std::memcpy(s.secret_key, key, 32);
        send(s);
    }
};

/** One "boot" of the vehicle. Pass the same storage to a second Rig to simulate a reboot. */
struct Rig {
    hal::PosixHAL hal;
    sim::QuadModel quad;
    sim::SimContext ctx;
    vehicles::VehicleManager vm{&hal, vehicles::VehicleType::UAV_MULTICOPTER};
    ParamStore params;
    Gcs gcs;
    Signing signing;
    std::unique_ptr<Settings> settings;
    std::unique_ptr<GcsLink> link;
    Settings::LoadResult loaded;

    explicit Rig(hal::Storage* storage = nullptr, bool trusted_link = false) {
        hal.time().set_manual_time_mode(true);
        hal.time().advance_time_ms(1000);
        ctx.model = &quad;
        ctx.time = &hal.time();
        vm.init();
        vm.get_sensors()->register_sensor(std::make_shared<sim::SimIMU>(&ctx, "SIM_IMU0"));
        vm.get_sensors()->register_sensor(std::make_shared<sim::SimBaro>(&ctx));
        vm.get_sensors()->register_sensor(std::make_shared<sim::SimCompass>(&ctx));
        register_vehicle_params(params, vm);
        settings = std::make_unique<Settings>(&params, storage);
        loaded = settings->load();
        signing.load(storage);
        GcsLink::Config cfg;
        cfg.trusted_link = trusted_link;
        link = std::make_unique<GcsLink>(&gcs.radio, &vm, &params, cfg);
        link->set_signing(&signing);
        link->set_settings(settings.get());
    }
    void run_ms(uint32_t ms, bool rc = true) {
        for (uint32_t t = 0; t < ms; t += 4) {
            if (rc) vm.set_user_inputs(0.0f, 0.0f, 0.0f, 0.0f);
            vm.step(0.004f);
            link->update();
            hal.time().advance_time_ms(4);
        }
        gcs.drain();
    }
};

} // namespace rig
