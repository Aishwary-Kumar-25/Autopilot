#!/usr/bin/env python3
"""
End-to-end check of saved settings and MAVLink 2 signing, with pymavlink as
the ground station talking to the real SITL binary over UDP.

  1. Saved parameters: set MC_ROLLRATE_P + FS_RC_TIMEOUT, stop the SITL, start
     it again: the values are still there.
  2. Signing: start with --signing-passphrase. An unsigned GCS gets no answers
     (and a STATUSTEXT saying why); every frame from the vehicle is signed and
     verifies with the key; a GCS with the key can read and set parameters.
  3. Replay: re-sending a captured signed PARAM_SET does nothing.
  4. The key survives a restart; a signed SETUP_SIGNING with an all-zero key
     turns signing off, and that is saved too.

    pip install pymavlink
    python3 tools/sitl_signing_check.py [--sitl build/autopilot_sitl] [--port 14561]

Exit code 0 = all checks passed.
"""
import argparse
import hashlib
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

FAIL = []
PASSPHRASE = "eclub-autopilot"
KEY = hashlib.sha256(PASSPHRASE.encode()).digest()   # what MAVProxy's "signing setup" uses


def check(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        FAIL.append(what)


class Sitl:
    def __init__(self, exe, port, storage, *extra):
        self.p = subprocess.Popen([exe, "--gcs", "127.0.0.1:%d" % port, "--storage", storage, "--quiet",
                                   "--duration", "60", *extra],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def stop(self):
        self.p.send_signal(signal.SIGINT)      # clean shutdown = power off while disarmed
        try:
            out, _ = self.p.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            self.p.kill()
            out, _ = self.p.communicate()
        return out


def connect(port, key=None):
    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % port, source_system=255, source_component=190)
    if key is not None:
        gcs.setup_signing(key, sign_outgoing=True, initial_timestamp=None)
    return gcs


def is_signed(m):
    """Signed-frame flag straight from the wire (pymavlink only sets get_signed() when it checked a key)."""
    return m is not None and (m.get_msgbuf()[2] & mavutil.mavlink.MAVLINK_IFLAG_SIGNED) != 0


def pump(gcs, seconds, want=None, heartbeat=True):
    """Receive for `seconds`, sending a GCS heartbeat each second. Returns the messages (and the first `want`)."""
    end = time.time() + seconds
    got, first, next_hb = [], None, 0.0
    while time.time() < end:
        if heartbeat and time.time() >= next_hb:
            gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            next_hb = time.time() + 1.0
        m = gcs.recv_match(blocking=True, timeout=0.1)
        if m is None or m.get_type() == "BAD_DATA":
            continue
        got.append(m)
        if want and first is None and want(m):
            first = m
            break
    return got, first


def read_param(gcs, name, timeout=3.0):
    gcs.mav.param_request_read_send(1, 1, name.encode(), -1)
    _, m = pump(gcs, timeout, lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == name)
    if m is None:
        return None
    if m.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", struct.pack("<f", m.param_value))[0]
    return m.param_value


def set_param(gcs, name, value, int32=False):
    if int32:
        v = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        gcs.mav.param_set_send(1, 1, name.encode(), v, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    else:
        gcs.mav.param_set_send(1, 1, name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _, m = pump(gcs, 3.0, lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == name)
    return m is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sitl", default=os.path.join(os.path.dirname(__file__), "..", "build", "autopilot_sitl"))
    ap.add_argument("--port", type=int, default=14561)
    args = ap.parse_args()
    storage = os.path.join(tempfile.mkdtemp(), "sitl.eeprom")

    print("-- 1. parameters survive a restart --")
    gcs = connect(args.port)
    s = Sitl(args.sitl, args.port, storage, "--wipe")
    check(gcs.wait_heartbeat(timeout=10) is not None, "vehicle heartbeat")
    check(set_param(gcs, "MC_ROLLRATE_P", 0.22), "PARAM_SET MC_ROLLRATE_P = 0.22 answered")
    check(set_param(gcs, "FS_RC_TIMEOUT", 750, int32=True), "PARAM_SET FS_RC_TIMEOUT = 750 answered")
    pump(gcs, 1.5)   # the save waits for a quiet second
    out = s.stop()
    s = Sitl(args.sitl, args.port, storage)
    check(gcs.wait_heartbeat(timeout=10) is not None, "heartbeat after restart")
    v = read_param(gcs, "MC_ROLLRATE_P")
    check(v is not None and abs(v - 0.22) < 1e-6, "MC_ROLLRATE_P still 0.22 after restart (got %s)" % v)
    v = read_param(gcs, "FS_RC_TIMEOUT")
    check(v == 750, "FS_RC_TIMEOUT still 750 after restart (got %s)" % v)
    out = s.stop()
    check("2 saved values loaded" in out, "SITL reports 2 saved values loaded")
    gcs.close()

    print("-- 2. signing on: unsigned GCS is ignored --")
    gcs = connect(args.port)                      # no key
    s = Sitl(args.sitl, args.port, storage, "--signing-passphrase", PASSPHRASE)
    hb = gcs.wait_heartbeat(timeout=10)
    check(hb is not None, "vehicle heartbeat (unsigned GCS can still watch)")
    check(is_signed(hb), "vehicle frames are signed")
    msgs, _ = pump(gcs, 2.0)
    texts = [m.text for m in msgs if m.get_type() == "STATUSTEXT"]
    check(read_param(gcs, "MC_ROLLRATE_P", 2.0) is None, "unsigned PARAM_REQUEST_READ gets no answer")
    gcs.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msgs, ack = pump(gcs, 2.0, lambda m: m.get_type() == "COMMAND_ACK")
    texts += [m.text for m in msgs if m.get_type() == "STATUSTEXT"]
    check(ack is None, "unsigned ARM command gets no ACK")
    check(any("unsigned MAVLink ignored" in t for t in texts), "vehicle says why (STATUSTEXT)")
    gcs.close()

    print("-- 3. GCS with the key --")
    gcs = connect(args.port, KEY)                 # pymavlink now also REJECTS unsigned/bad frames from the vehicle
    m = gcs.wait_heartbeat(timeout=10)
    check(m is not None and m.get_signed(), "signed heartbeat verified by pymavlink")
    v = read_param(gcs, "MC_ROLLRATE_P")
    check(v is not None and abs(v - 0.22) < 1e-6, "signed PARAM_REQUEST_READ answered (%s)" % v)
    check(set_param(gcs, "MC_PITCHRATE_P", 0.30), "signed PARAM_SET accepted")
    # Capture a signed PARAM_SET, then move on and replay it.
    replay = gcs.mav.param_set_encode(1, 1, b"MC_PITCHRATE_P", 0.31, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    buf = replay.pack(gcs.mav)
    gcs.write(buf)
    pump(gcs, 0.5)
    check(abs((read_param(gcs, "MC_PITCHRATE_P") or 0) - 0.31) < 1e-6, "captured frame took effect once (0.31)")
    check(set_param(gcs, "MC_PITCHRATE_P", 0.33), "set MC_PITCHRATE_P = 0.33")
    gcs.write(buf)                                # replay attack
    pump(gcs, 0.5)
    v = read_param(gcs, "MC_PITCHRATE_P")
    check(v is not None and abs(v - 0.33) < 1e-6, "replayed frame ignored (still 0.33, got %s)" % v)
    pump(gcs, 1.5)
    s.stop()
    gcs.close()

    print("-- 4. key saved; turning signing off --")
    gcs = connect(args.port)                      # unsigned again
    s = Sitl(args.sitl, args.port, storage)       # no passphrase on the command line
    hb = gcs.wait_heartbeat(timeout=10)
    check(is_signed(hb), "after restart the vehicle still signs (key was saved)")
    check(read_param(gcs, "MC_ROLLRATE_P", 2.0) is None, "and still ignores unsigned requests")
    gcs.close()
    gcs = connect(args.port, KEY)
    gcs.wait_heartbeat(timeout=10)
    gcs.mav.setup_signing_send(1, 1, [0] * 32, 0)   # signed, all-zero key = off
    # The vehicle's reply is already unsigned, so let this GCS accept unsigned frames now.
    gcs.mav.signing.allow_unsigned_callback = lambda mav, msg_id: True
    msgs, t = pump(gcs, 3.0, lambda m: m.get_type() == "STATUSTEXT" and "Signing disabled" in m.text)
    check(t is not None, "signed SETUP_SIGNING(zero key): 'Signing disabled'")
    pump(gcs, 1.0)
    s.stop()
    gcs.close()
    gcs = connect(args.port)
    s = Sitl(args.sitl, args.port, storage)
    hb = gcs.wait_heartbeat(timeout=10)
    check(hb is not None and not is_signed(hb), "after restart: unsigned again")
    v = read_param(gcs, "MC_PITCHRATE_P")
    check(v is not None and abs(v - 0.33) < 1e-6, "tuning set over the signed link was saved (%s)" % v)
    s.stop()
    gcs.close()

    print("\n%s" % ("ALL CHECKS PASSED" if not FAIL else "%d CHECK(S) FAILED" % len(FAIL)))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
