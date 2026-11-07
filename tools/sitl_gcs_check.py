#!/usr/bin/env python3
"""
End-to-end check of the MAVLink link with pymavlink as the ground station,
exactly the way QGroundControl talks to the vehicle (UDP, MAVLink 2).

Starts build/autopilot_sitl pointed at a local UDP port, then as the GCS:
  heartbeat discovery -> AUTOPILOT_VERSION -> full parameter download ->
  PARAM_SET -> ALT_HOLD via DO_SET_MODE -> joystick (MANUAL_CONTROL) ->
  arm -> climb -> hold -> descend -> land -> disarm,
checking telemetry (ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS, STATUSTEXT) on the way.

    pip install pymavlink
    python3 tools/sitl_gcs_check.py [--sitl build/autopilot_sitl] [--port 14560]

Exit code 0 = all checks passed.
"""
import argparse
import os
import struct
import subprocess
import sys
import threading
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

FAIL = []


def check(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        FAIL.append(what)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sitl", default=os.path.join(os.path.dirname(__file__), "..", "build", "autopilot_sitl"))
    ap.add_argument("--port", type=int, default=14560, help="UDP port this script listens on (QGC uses 14550)")
    args = ap.parse_args()

    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % args.port, source_system=255, source_component=190)
    sitl = subprocess.Popen([args.sitl, "--gcs", "127.0.0.1:%d" % args.port, "--duration", "60", "--quiet",
                             "--storage", "none"],  # always start from defaults
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stop = threading.Event()
    stick = {"x": 0, "y": 0, "z": 0, "r": 0}
    texts = []
    state = {}

    try:
        hb = gcs.wait_heartbeat(timeout=10)
        check(hb is not None, "vehicle heartbeat received")
        if hb is None:
            return
        check(hb.get_srcSystem() == 1 and hb.type == mavutil.mavlink.MAV_TYPE_QUADROTOR,
              "sysid 1, MAV_TYPE_QUADROTOR (got type %d)" % hb.type)
        check(gcs.mavlink20(), "link is MAVLink 2")

        # Background: GCS heartbeat 1 Hz + joystick 20 Hz + collect telemetry.
        def gcs_tx():
            n = 0
            while not stop.is_set():
                if n % 20 == 0:
                    gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                gcs.mav.manual_control_send(1, stick["x"], stick["y"], stick["z"], stick["r"], 0)
                n += 1
                time.sleep(0.05)

        lock = threading.Lock()
        inbox = []

        def gcs_rx():
            while not stop.is_set():
                m = gcs.recv_match(blocking=True, timeout=0.2)
                if m is None:
                    continue
                t = m.get_type()
                with lock:
                    state[t] = m
                    inbox.append(m)
                    if t == "STATUSTEXT":
                        texts.append(m.text)
                        print("  [vehicle] %s" % m.text)

        def wait_for(pred, timeout):
            end = time.time() + timeout
            while time.time() < end:
                with lock:
                    for m in inbox:
                        if pred(m):
                            inbox.remove(m)
                            return m
                time.sleep(0.02)
            return None

        rx = threading.Thread(target=gcs_rx, daemon=True)
        rx.start()

        # 1. AUTOPILOT_VERSION after the vehicle sees our heartbeat
        gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        av = wait_for(lambda m: m.get_type() == "AUTOPILOT_VERSION", 5)
        check(av is not None and (av.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2) != 0,
              "AUTOPILOT_VERSION with MAVLINK2 capability")
        tx = threading.Thread(target=gcs_tx, daemon=True)
        tx.start()

        # 2. full parameter download
        gcs.mav.param_request_list_send(1, 1)
        params, count, end = {}, None, time.time() + 5
        while time.time() < end and (count is None or len(params) < count):
            m = wait_for(lambda m: m.get_type() == "PARAM_VALUE", 1)
            if m:
                params[m.param_id] = m
                count = m.param_count
        check(count is not None and len(params) == count, "parameter list complete (%d of %s)" % (len(params), count))
        check("MC_ROLLRATE_P" in params and abs(params["MC_ROLLRATE_P"].param_value - 0.15) < 1e-6,
              "MC_ROLLRATE_P = 0.15")
        if "FS_RC_TIMEOUT" in params:
            raw = struct.unpack("<i", struct.pack("<f", params["FS_RC_TIMEOUT"].param_value))[0]
            check(params["FS_RC_TIMEOUT"].param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32 and raw == 500,
                  "FS_RC_TIMEOUT = 500 (INT32, byte-wise)")

        # 3. PARAM_SET round trip
        gcs.mav.param_set_send(1, 1, b"MC_ROLLRATE_P", 0.16, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m = wait_for(lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == "MC_ROLLRATE_P", 3)
        check(m is not None and abs(m.param_value - 0.16) < 1e-6, "PARAM_SET MC_ROLLRATE_P -> 0.16 confirmed")

        def command(cmd, *p):
            p = list(p) + [0] * (7 - len(p))
            gcs.mav.command_long_send(1, 1, cmd, 0, *p)
            a = wait_for(lambda m: m.get_type() == "COMMAND_ACK" and m.command == cmd, 3)
            return a.result if a else None

        # 4. mode + arm with the joystick at zero throttle
        r = command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2)
        check(r == mavutil.mavlink.MAV_RESULT_ACCEPTED, "DO_SET_MODE ALT_HOLD accepted")
        time.sleep(1.5)
        r = command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        check(r == mavutil.mavlink.MAV_RESULT_ACCEPTED, "ARM accepted (result %s)" % r)
        time.sleep(1.2)
        hbm = state.get("HEARTBEAT")
        check(hbm is not None and (hbm.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and hbm.custom_mode == 2,
              "heartbeat shows ARMED in custom mode 2")

        # 5. climb, hold, descend
        stick["z"] = 850
        time.sleep(3.0)
        stick["z"] = 500
        time.sleep(1.0)
        gp1 = state.get("GLOBAL_POSITION_INT")
        time.sleep(3.0)
        gp2 = state.get("GLOBAL_POSITION_INT")
        att = state.get("ATTITUDE")
        alt1 = gp1.relative_alt / 1000.0 if gp1 else 0
        alt2 = gp2.relative_alt / 1000.0 if gp2 else 0
        print("  climb + hold: relative_alt %.2f m -> %.2f m, lat %.7f lon %.7f" %
              (alt1, alt2, (gp2.lat / 1e7) if gp2 else 0, (gp2.lon / 1e7) if gp2 else 0))
        check(alt2 > 1.5, "climbed above 1.5 m (%.2f m)" % alt2)
        check(abs(alt2 - alt1) < 0.6, "ALT_HOLD holds altitude (drift %.2f m)" % (alt2 - alt1))
        check(att is not None and abs(att.roll) < 0.08 and abs(att.pitch) < 0.08, "ATTITUDE level in hover")
        ss = state.get("SYS_STATUS")
        check(ss is not None and ss.voltage_battery > 13000, "SYS_STATUS battery %.2f V" % (ss.voltage_battery / 1000 if ss else 0))
        so = state.get("SERVO_OUTPUT_RAW")
        check(so is not None and 1300 < so.servo1_raw < 1800, "SERVO_OUTPUT_RAW motor 1 = %s us" % (so.servo1_raw if so else None))

        r = command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        check(r == mavutil.mavlink.MAV_RESULT_DENIED, "disarm in the air refused")

        stick["z"] = 150
        end = time.time() + 20
        landed = False
        while time.time() < end:
            gp = state.get("GLOBAL_POSITION_INT")
            if gp and gp.relative_alt < 150 and abs(gp.vz) < 20:
                landed = True
                break
            time.sleep(0.2)
        check(landed, "descended and landed")
        stick["z"] = 0
        # The land detector needs ~1 s of "on the ground, low thrust, not moving".
        t0, r = time.time(), None
        while time.time() - t0 < 10 and r != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            time.sleep(0.5)
            r = command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        check(r == mavutil.mavlink.MAV_RESULT_ACCEPTED,
              "disarm on the ground accepted (after %.1f s at zero throttle)" % (time.time() - t0))
        time.sleep(1.2)
        hbm = state.get("HEARTBEAT")
        check(hbm is not None and not (hbm.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED), "heartbeat shows disarmed")
        check(any("Armed" == t for t in texts) and any("Disarmed" == t for t in texts), "Armed/Disarmed STATUSTEXT")
    finally:
        stop.set()
        sitl.terminate()
        try:
            sitl.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sitl.kill()

    print("\n%s: %d failure(s)" % ("PASS" if not FAIL else "FAIL", len(FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main() or (1 if FAIL else 0))
