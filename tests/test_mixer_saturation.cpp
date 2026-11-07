/**
 * Mixer desaturation, NaN safety and multi-frame DroneCAN ESC output.
 */
#include "control/actuator_matrix_mixer.hpp"
#include "drivers/can/dronecan_codec.hpp"
#include "hal/posix/posix_hal.hpp"
#include "tests/test_common.hpp"
#include <cmath>
#include <cstdio>

using namespace autopilot;
using control::ActuatorMatrixMixer;
using control::ControlDemands;
using control::MixerPreset;

// Quad-X channel order: 0 FR, 1 RL, 2 FL, 3 RR
static void mix(const ActuatorMatrixMixer& m, const ControlDemands& d, float* out) {
    m.get_normalized_outputs(d, out, hal::PWMOutput::MAX_CHANNELS);
}

static bool in_range(const float* o, int n) {
    for (int i = 0; i < n; ++i) if (!(o[i] >= 0.0f && o[i] <= 1.0f)) return false;
    return true;
}

int main() {
    std::printf("== Mixer saturation ==\n");
    ActuatorMatrixMixer quad(MixerPreset::UAV_QUAD_X);
    float o[hal::PWMOutput::MAX_CHANNELS];

    // 1. Unsaturated: plain mix, untouched.
    {
        ControlDemands d; d.throttle = 0.5f; d.roll = 0.1f; d.yaw = 0.1f;
        mix(quad, d, o);
        CHECK_NEAR(o[0], 0.5f, 1e-5); CHECK_NEAR(o[1], 0.7f, 1e-5);
        CHECK_NEAR(o[2], 0.5f, 1e-5); CHECK_NEAR(o[3], 0.3f, 1e-5);
    }

    // 2. Roll + yaw too big together: roll kept in full, yaw reduced to what fits.
    {
        ControlDemands d; d.throttle = 0.5f; d.roll = 0.3f; d.yaw = 0.5f;
        mix(quad, d, o);
        CHECK(in_range(o, 4));
        const float roll_diff = 0.5f * ((o[1] + o[2]) - (o[0] + o[3]));
        const float yaw_diff = 0.5f * ((o[0] + o[1]) - (o[2] + o[3]));
        CHECK_NEAR(roll_diff, 0.6f, 0.01);        // full roll authority (2 * 0.3)
        CHECK_NEAR(yaw_diff, 0.4f, 0.01);         // yaw scaled from 1.0 to 0.4 (spread = 0.6 + yaw = 1)
        CHECK_NEAR((o[0] + o[1] + o[2] + o[3]) / 4.0f, 0.5f, 0.01); // collective preserved
        // (The old mixer shifted by the pre-shift minimum and clipped: roll and yaw both distorted.)
    }

    // 3. Roll alone beyond the motor range: scaled, sign kept, yaw dropped.
    {
        ControlDemands d; d.throttle = 0.5f; d.roll = 0.8f; d.yaw = 0.8f;
        mix(quad, d, o);
        CHECK(in_range(o, 4));
        CHECK_NEAR(o[1], 1.0f, 1e-4); CHECK_NEAR(o[2], 1.0f, 1e-4); // left motors full
        CHECK_NEAR(o[0], 0.0f, 1e-4); CHECK_NEAR(o[3], 0.0f, 1e-4); // right motors off
    }

    // 4. High throttle + torque: collective gives way, differential kept.
    {
        ControlDemands d; d.throttle = 0.95f; d.pitch = 0.2f;
        mix(quad, d, o);
        CHECK(in_range(o, 4));
        const float pitch_diff = 0.5f * ((o[0] + o[2]) - (o[1] + o[3]));
        CHECK_NEAR(pitch_diff, 0.4f, 1e-4);
        CHECK_NEAR(o[0], 1.0f, 1e-4);
    }

    // 5. Low throttle + torque: collective raised so the differential still exists.
    {
        ControlDemands d; d.throttle = 0.05f; d.roll = 0.2f;
        mix(quad, d, o);
        CHECK(in_range(o, 4));
        CHECK_NEAR(0.5f * ((o[1] + o[2]) - (o[0] + o[3])), 0.4f, 1e-4);
    }

    // 6. Random sweep: always in range, and roll/pitch direction never inverted.
    {
        uint32_t s = 7;
        auto rnd = [&]() { s = s * 1664525u + 1013904223u; return (static_cast<float>(s >> 8) / 16777216.0f) * 2.0f - 1.0f; };
        bool ok = true;
        ActuatorMatrixMixer hexa(MixerPreset::UAV_HEXA_X);
        for (int i = 0; i < 5000; ++i) {
            ControlDemands d; d.roll = rnd(); d.pitch = rnd(); d.yaw = rnd(); d.throttle = 0.5f * (rnd() + 1.0f);
            for (auto* m : {&quad, &hexa}) {
                mix(*m, d, o);
                const int n = m->get_num_channels();
                if (!in_range(o, n)) ok = false;
                float r = 0.0f, p = 0.0f;
                for (int k = 0; k < n; ++k) {
                    r += o[k] * m->get_actuator_config(k).roll;
                    p += o[k] * m->get_actuator_config(k).pitch;
                }
                if (r * d.roll < -1e-4f || p * d.pitch < -1e-4f) ok = false;
            }
        }
        CHECK(ok);
    }

    // 7. NaN / Inf demands never reach an ESC.
    {
        hal::PosixPWMOutput pwm;
        ControlDemands d; d.throttle = 0.6f; d.roll = std::nanf("");
        quad.output_to_actuators(d, &pwm, true);
        for (int i = 0; i < 4; ++i) CHECK(pwm.get_channel(i) == 1000);
        ActuatorMatrixMixer rov(MixerPreset::ROV_6_VECTORED);
        ControlDemands r; r.forward = INFINITY;
        rov.output_to_actuators(r, &pwm, true);
        for (int i = 0; i < 6; ++i) CHECK(pwm.get_channel(i) == 1500);
    }

    // 8. DroneCAN ESC RawCommand for 8 motors: multi-frame transfer carrying all 8.
    {
        ActuatorMatrixMixer octo(MixerPreset::UAV_OCTA_X);
        hal::PosixCANDevice can;
        ControlDemands d; d.throttle = 0.5f; d.roll = 0.1f;
        uint8_t tid = 3;
        CHECK(octo.send_dronecan_esc_commands(d, &can, tid, true));
        CHECK(tid == 4);
        const auto& frames = can.get_transmitted_frames();
        CHECK(frames.size() > 1);
        drivers::dronecan::Reassembler rx;
        drivers::dronecan::Transfer t;
        bool got = false;
        for (const auto& f : frames) got = rx.accept(f, 0, t) || got;
        CHECK(got);
        int16_t cmd[20]; uint8_t n = 0;
        CHECK(got && drivers::dronecan::decode_esc_raw_command(t.payload, t.len, cmd, n));
        CHECK(n == 8);
        float ref[16];
        octo.get_normalized_outputs(d, ref, 16);
        bool match = n == 8;
        for (int i = 0; i < n && i < 8; ++i) match = match && std::abs(cmd[i] - std::lround(ref[i] * 8191.0f)) <= 1;
        CHECK(match);
    }

    return test::finish("test_mixer_saturation");
}
