/**
 * MAVLink 2 signing tests.
 *   1. SHA-256 against FIPS / hashlib vectors.
 *   2. Signer + verifier against frames signed by pymavlink (tests/mavlink_golden.hpp):
 *      byte-identical signatures, replay / stale / tampered / wrong-key rejection.
 *   3. GcsLink with signing: unsigned and forged commands ignored, SETUP_SIGNING
 *      rules (trusted link, key rotation, disarmed only, turning it off),
 *      key + clock saved across a reboot.
 */
#include "comms/mavlink/sha256.hpp"
#include "hal/posix/file_storage.hpp"
#include "tests/gcs_rig.hpp"
#include "tests/mavlink_golden.hpp"
#include "tests/test_common.hpp"

#include <cstdio>
#include <string>

using namespace rig;

namespace {

std::string hex(const uint8_t* d, size_t n) {
    std::string s;
    char b[3];
    for (size_t i = 0; i < n; ++i) { std::snprintf(b, sizeof(b), "%02x", d[i]); s += b; }
    return s;
}

std::string sha_hex(const std::string& in) {
    uint8_t d[32];
    Sha256::hash(reinterpret_cast<const uint8_t*>(in.data()), in.size(), d);
    return hex(d, 32);
}

const uint8_t* test_key() {
    static uint8_t k[32];
    for (int i = 0; i < 32; ++i) k[i] = static_cast<uint8_t>(i);
    return k;
}
constexpr uint64_t TS_TX = 1000000, TS_RX = 500000000;

std::vector<uint8_t> vec(const uint8_t* d, size_t n) { return std::vector<uint8_t>(d, d + n); }

Signing::Result verify_bytes(Signing& s, const std::vector<uint8_t>& f) {
    Parser p;
    for (uint8_t b : f) {
        if (p.parse(b)) return s.verify(p.frame(), p.frame_len(), p.msg().sysid, p.msg().compid);
    }
    return Signing::Result::UNSIGNED;   // didn't even parse
}

void test_sha256() {
    std::printf("-- SHA-256 --\n");
    CHECK(sha_hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(sha_hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(sha_hex(std::string(1000, 'a')) == "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
    // incremental, crossing block boundaries in odd pieces
    Sha256 h;
    const std::string a(1000, 'a');
    for (size_t i = 0; i < a.size(); i += 7) h.update(reinterpret_cast<const uint8_t*>(a.data()) + i, std::min<size_t>(7, a.size() - i));
    uint8_t d[32];
    h.final(d);
    CHECK(hex(d, 32) == "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3");
    // MAVProxy-style passphrase key
    uint8_t k[32];
    Signing::key_from_passphrase("autopilot", k);
    CHECK(hex(k, 32) == "06a8d99fc6930dbf638ca0330c53a435b63c9541957cf9b7a237c336916c5f81");
}

void test_vs_pymavlink() {
    std::printf("-- signing vs pymavlink --\n");
    // Our signer reproduces pymavlink's signed frame byte for byte.
    Signing tx;
    tx.set_key(test_key(), TS_TX);
    CHECK(tx.enabled());
    Heartbeat hb;
    hb.type = MAV_TYPE_QUADROTOR; hb.autopilot = MAV_AUTOPILOT_GENERIC;
    hb.base_mode = 1 | 64 | 16 | 128; hb.custom_mode = 2; hb.system_status = MAV_STATE_ACTIVE;
    uint8_t f[MAX_FRAME];
    size_t n = pack(f, 0, 1, 1, hb, INCOMPAT_FLAG_SIGNED);
    n = tx.sign(f, n);
    CHECK(n == sizeof(golden::SIGNED_TX_HEARTBEAT) && std::memcmp(f, golden::SIGNED_TX_HEARTBEAT, n) == 0);
    CHECK(tx.timestamp() == TS_TX + 1);   // each frame gets a new timestamp

    // SETUP_SIGNING codec
    SetupSigning ss;
    ss.target_system = 1; ss.target_component = 1; ss.initial_timestamp = TS_RX;
    std::memcpy(ss.secret_key, test_key(), 32);
    n = pack(f, 46, 255, 190, ss);
    CHECK(n == sizeof(golden::SETUP_SIGNING) && std::memcmp(f, golden::SETUP_SIGNING, n) == 0);
    Parser p;
    bool got = false;
    for (size_t i = 0; i < sizeof(golden::SETUP_SIGNING); ++i) got |= p.parse(golden::SETUP_SIGNING[i]);
    CHECK(got && p.msg().msgid == SetupSigning::ID);
    const auto ssd = SetupSigning::decode(p.msg().payload);
    CHECK(ssd.initial_timestamp == TS_RX && std::memcmp(ssd.secret_key, test_key(), 32) == 0);

    // Our verifier accepts pymavlink-signed frames...
    Signing rx;
    rx.set_key(test_key(), 0);
    const auto hb_rx = vec(golden::SIGNED_RX_HEARTBEAT, sizeof(golden::SIGNED_RX_HEARTBEAT));
    const auto arm_rx = vec(golden::SIGNED_RX_ARM, sizeof(golden::SIGNED_RX_ARM));
    CHECK(verify_bytes(rx, hb_rx) == Signing::Result::OK);
    CHECK(verify_bytes(rx, arm_rx) == Signing::Result::OK);
    CHECK(rx.timestamp() >= TS_RX);                  // clock pulled forward to the GCS's
    // ...exactly once (replay), and never in the wrong order on one stream
    CHECK(verify_bytes(rx, arm_rx) == Signing::Result::REPLAY);
    CHECK(verify_bytes(rx, hb_rx) == Signing::Result::REPLAY);

    // tampered payload (param1 = 1.0 -> 0.0 would be "disarm"): CRC fixed up, signature can't be
    auto bad = arm_rx;
    bad[12] ^= 0x80;
    uint16_t crc = crc_calculate(bad.data() + 1, 9 + bad[1]);
    crc = crc_accumulate(CommandLong::CRC_EXTRA, crc);
    bad[10 + bad[1]] = static_cast<uint8_t>(crc & 0xFF);
    bad[11 + bad[1]] = static_cast<uint8_t>(crc >> 8);
    Signing rx2;
    rx2.set_key(test_key(), 0);
    CHECK(verify_bytes(rx2, bad) == Signing::Result::BAD_SIGNATURE);
    CHECK(verify_bytes(rx2, arm_rx) == Signing::Result::OK);   // the forgery didn't poison the stream

    // wrong key
    uint8_t other[32];
    Signing::key_from_passphrase("not the key", other);
    Signing rx3;
    rx3.set_key(other, 0);
    CHECK(verify_bytes(rx3, arm_rx) == Signing::Result::BAD_SIGNATURE);

    // a new stream more than a minute behind our clock is refused (old capture)
    Signing rx4;
    rx4.set_key(test_key(), TS_RX + 2 * Signing::ONE_MINUTE);
    CHECK(verify_bytes(rx4, arm_rx) == Signing::Result::TOO_OLD);
    Signing rx5;
    rx5.set_key(test_key(), TS_RX + Signing::ONE_MINUTE / 2);  // 30 s behind: still fine
    CHECK(verify_bytes(rx5, arm_rx) == Signing::Result::OK);

    // unsigned frame while signing is on
    std::vector<uint8_t> uns(f, f + pack(f, 3, 255, 190, hb));
    CHECK(verify_bytes(rx5, uns) == Signing::Result::UNSIGNED);

    // all-zero key turns signing off
    uint8_t zero[32] = {};
    rx5.set_key(zero, 0);
    CHECK(!rx5.enabled());
}

void test_link_signing() {
    std::printf("-- GcsLink with signing --\n");
    const char* path = "test_signing.eeprom";
    std::remove(path);
    hal::FileStorage storage(path);
    CHECK(storage.open());
    uint8_t key[32];
    Signing::key_from_passphrase("eclub-autopilot", key);

    {
        // Radio link (untrusted): an unsigned SETUP_SIGNING must not install a key.
        Rig r(&storage, /*trusted_link=*/false);
        r.run_ms(100);
        r.gcs.setup_signing(key, TS_RX);
        r.run_ms(100);
        CHECK(!r.link->signing_enabled());
        CHECK(r.gcs.has_text("Signing setup refused"));
    }
    {
        // USB / SITL (trusted): it does.
        Rig r(&storage, /*trusted_link=*/true);
        CHECK(!r.signing.enabled());
        r.run_ms(100);
        r.gcs.setup_signing(key, TS_RX);
        r.gcs.inbox.clear();
        r.run_ms(1100);
        CHECK(r.link->signing_enabled());
        CHECK(r.gcs.has_text("Signing enabled"));
        CHECK(r.settings->commits() >= 1);           // key saved straight away (disarmed)
        // Everything the vehicle sends from now on is signed, and verifies with the key.
        Signing gcs_check;
        gcs_check.set_key(key, 0);
        size_t signed_ok = 0, total = 0;
        for (const auto& fr : r.gcs.inbox) {
            if (fr.msg.msgid == SetupSigning::ID) continue;
            ++total;
            Parser p;
            for (uint8_t b : fr.raw) {
                if (p.parse(b) && gcs_check.verify(p.frame(), p.frame_len(), p.msg().sysid, p.msg().compid) ==
                                      Signing::Result::OK) ++signed_ok;
            }
        }
        std::printf("  vehicle frames after key setup: %zu, signed and verified: %zu\n", total, signed_ok);
        CHECK(total > 5 && signed_ok + 1 >= total);   // the frame right at the switch may be unsigned

        // An unsigned GCS is now ignored: no ACK, a warning says why.
        r.gcs.inbox.clear();
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
        r.run_ms(200);
        CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == -1);
        CHECK(!r.vm.is_armed());
        CHECK(r.link->stats().rx_rejected_unsigned >= 1);
        CHECK(r.gcs.has_text("unsigned MAVLink ignored"));

        // Signed with the wrong key: ignored too.
        uint8_t wrong[32];
        Signing::key_from_passphrase("guess", wrong);
        r.gcs.signing.set_key(wrong, TS_RX + 100);
        r.gcs.inbox.clear();
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
        r.run_ms(200);
        CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == -1);
        CHECK(r.link->stats().rx_rejected_signature >= 1);

        // The right key: parameters and commands work.
        r.gcs.signing.disable();
        r.gcs.signing.set_key(key, TS_RX + 200);
        r.gcs.inbox.clear();
        r.gcs.param_set("MC_ROLLRATE_P", 0.21f);
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
        r.run_ms(200);
        CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED);
        CHECK(r.vm.is_armed());
        CHECK_NEAR(r.params.value(static_cast<size_t>(r.params.find("MC_ROLLRATE_P"))), 0.21, 1e-6);
        const auto arm_frame = r.gcs.last_sent;

        // Key changes are refused in flight, even signed ones.
        uint8_t key2[32];
        Signing::key_from_passphrase("second key", key2);
        r.gcs.inbox.clear();
        r.gcs.setup_signing(key2, 0);
        r.run_ms(100);
        CHECK(r.gcs.has_text("disarm first"));

        // Disarm (signed), then replay the captured "arm" frame: rejected.
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 0, ARM_DISARM_FORCE_MAGIC);
        r.run_ms(200);
        CHECK(!r.vm.is_armed());
        const uint32_t before = r.link->stats().rx_rejected_signature;
        r.gcs.send_raw(arm_frame);
        r.run_ms(200);
        CHECK(!r.vm.is_armed());
        CHECK(r.link->stats().rx_rejected_signature == before + 1);
        r.run_ms(1500);   // let the disarm-time clock save be committed
    }
    {
        // Reboot: the key comes back from storage, so unsigned commands stay locked out.
        Rig r(&storage, /*trusted_link=*/false);
        CHECK(r.signing.enabled());
        CHECK(r.signing.timestamp() >= TS_RX);       // clock saved at disarm, never goes backwards
        r.gcs.command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
        r.run_ms(200);
        CHECK(r.gcs.ack_result(MAV_CMD_COMPONENT_ARM_DISARM) == -1);
        // The tuning set over the signed link was saved too.
        CHECK_NEAR(r.params.value(static_cast<size_t>(r.params.find("MC_ROLLRATE_P"))), 0.21, 1e-6);

        // Key rotation over the radio, signed with the current key: accepted.
        uint8_t key2[32];
        Signing::key_from_passphrase("second key", key2);
        r.gcs.signing.set_key(key, r.signing.timestamp() + 10);
        r.gcs.inbox.clear();
        r.gcs.setup_signing(key2, 0);
        r.run_ms(100);
        CHECK(r.gcs.has_text("Signing enabled"));
        // Old key no longer works.
        r.gcs.inbox.clear();
        r.gcs.command(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES);
        r.run_ms(100);
        CHECK(r.gcs.ack_result(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) == -1);
        // New key does; an all-zero key (signed) turns signing off.
        r.gcs.signing.set_key(key2, r.signing.timestamp() + 10);
        r.gcs.command(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES);
        r.run_ms(100);
        CHECK(r.gcs.ack_result(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) == MAV_RESULT_ACCEPTED);
        uint8_t zero[32] = {};
        r.gcs.setup_signing(zero, 0);
        r.run_ms(1500);
        CHECK(!r.link->signing_enabled());
        CHECK(r.gcs.has_text("Signing disabled"));
    }
    {
        Rig r(&storage, false);
        CHECK(!r.signing.enabled());                 // "off" was saved too
        r.gcs.command(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES);
        r.run_ms(100);
        CHECK(r.gcs.ack_result(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) == MAV_RESULT_ACCEPTED);
    }
    std::remove(path);
}

} // namespace

int main() {
    test_sha256();
    test_vs_pymavlink();
    test_link_signing();
    return test::finish("test_signing");
}
