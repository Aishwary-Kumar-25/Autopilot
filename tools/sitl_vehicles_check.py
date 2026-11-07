#!/usr/bin/env python3
"""
End-to-end: ONE running autopilot_sitl, switched between every vehicle type
from the ground station, the way you would do it in QGroundControl
(Vehicle Setup > Parameters > VEHICLE_TYPE, while disarmed).

For each of rover (1), boat (3), sub (2) and back to quad (0):
  PARAM_SET VEHICLE_TYPE -> heartbeat MAV_TYPE changes -> arm -> drive it with the
  joystick (MANUAL_CONTROL) -> check it moved on the map (GLOBAL_POSITION_INT) or
  dived (VFR_HUD / ATTITUDE) -> disarm.

    pip install pymavlink
    python3 tools/sitl_vehicles_check.py [--sitl build/autopilot_sitl] [--port 14561]

Exit code 0 = all checks passed.
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import threading
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

M = mavutil.mavlink
FAIL = []


def check(cond, what):
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        FAIL.append(what)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sitl", default=os.path.join(os.path.dirname(__file__), "..", "build", "autopilot_sitl"))
    ap.add_argument("--port", type=int, default=14561)
    args = ap.parse_args()

    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % args.port, source_system=255, source_component=190)
    sitl = subprocess.Popen([args.sitl, "--gcs", "127.0.0.1:%d" % args.port, "--duration", "120", "--quiet", "--storage", "none"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    stop = threading.Event()
    stick = {"x": 0, "y": 0, "z": 0, "r": 0}
    state, inbox, lock = {}, [], threading.Lock()

    def gcs_tx():
        n = 0
        while not stop.is_set():
            if n % 20 == 0:
                gcs.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            gcs.mav.manual_control_send(1, stick["x"], stick["y"], stick["z"], stick["r"], 0)
            n += 1
            time.sleep(0.05)

    def gcs_rx():
        while not stop.is_set():
            m = gcs.recv_match(blocking=True, timeout=0.2)
            if m is None:
                continue
            with lock:
                state[m.get_type()] = m
                inbox.append(m)
            if m.get_type() == "STATUSTEXT":
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

    def command(cmd, *p):
        p = list(p) + [0] * (7 - len(p))
        gcs.mav.command_long_send(1, 1, cmd, 0, *p)
        a = wait_for(lambda m: m.get_type() == "COMMAND_ACK" and m.command == cmd, 3)
        return a.result if a else None

    def as_wire(v):   # INT32 params travel byte-wise in the float field (as QGC sends them)
        return struct.unpack("<f", struct.pack("<i", int(v)))[0]

    def from_wire(f):
        return struct.unpack("<i", struct.pack("<f", f))[0]

    def set_param(name, value):
        gcs.mav.param_set_send(1, 1, name.encode(), as_wire(value), M.MAV_PARAM_TYPE_INT32)
        m = wait_for(lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == name, 3)
        return m is not None and from_wire(m.param_value) == int(value)

    def heartbeat_type(want, timeout=3):
        m = wait_for(lambda m: m.get_type() == "HEARTBEAT" and m.get_srcSystem() == 1 and m.type == want, timeout)
        return m is not None

    def position():
        gp = state.get("GLOBAL_POSITION_INT")
        if not gp:
            return None
        return gp.lat / 1e7, gp.lon / 1e7, gp.relative_alt / 1000.0

    def metres(a, b):
        dn = (b[0] - a[0]) * 111320.0
        de = (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0]))
        return math.hypot(dn, de)

    try:
        hb = gcs.wait_heartbeat(timeout=10)
        check(hb is not None, "vehicle heartbeat received")
        if hb is None:
            return
        threading.Thread(target=gcs_rx, daemon=True).start()
        threading.Thread(target=gcs_tx, daemon=True).start()
        time.sleep(1.0)

        def arm():
            for _ in range(5):                      # sensors / RC may need a moment
                if command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED:
                    return True
                time.sleep(0.5)
            return False

        # ---------------- rover ----------------
        print("rover:")
        check(set_param("VEHICLE_TYPE", 1), "VEHICLE_TYPE = 1 accepted")
        check(heartbeat_type(M.MAV_TYPE_GROUND_ROVER), "heartbeat MAV_TYPE_GROUND_ROVER")
        check(command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1) == M.MAV_RESULT_ACCEPTED,
              "HEADING_HOLD")
        time.sleep(2.0)
        check(arm(), "rover armed")
        p0 = position()
        stick["x"] = 500                            # half throttle forward
        time.sleep(5.0)
        stick["x"] = 0
        p1 = position()
        d = metres(p0, p1) if p0 and p1 else 0
        hud = state.get("VFR_HUD")
        print("  drove %.1f m (groundspeed %.1f m/s)" % (d, hud.groundspeed if hud else -1))
        check(d > 5.0, "rover moved on the map (%.1f m)" % d)
        time.sleep(2.0)
        check(command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0) == M.MAV_RESULT_ACCEPTED, "rover disarmed")

        # ---------------- boat ----------------
        print("boat:")
        check(set_param("VEHICLE_TYPE", 3), "VEHICLE_TYPE = 3 accepted")
        check(heartbeat_type(M.MAV_TYPE_SURFACE_BOAT), "heartbeat MAV_TYPE_SURFACE_BOAT")
        time.sleep(1.0)
        check(arm(), "boat armed")
        p0 = position()
        stick["x"] = 600
        stick["r"] = 0
        time.sleep(5.0)
        stick["x"] = 0
        p1 = position()
        d = metres(p0, p1) if p0 and p1 else 0
        check(d > 4.0, "boat moved on the map (%.1f m)" % d)
        time.sleep(2.0)
        check(command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0) == M.MAV_RESULT_ACCEPTED, "boat disarmed")

        # ---------------- sub ----------------
        print("sub:")
        check(set_param("VEHICLE_TYPE", 2), "VEHICLE_TYPE = 2 accepted")
        check(heartbeat_type(M.MAV_TYPE_SUBMARINE), "heartbeat MAV_TYPE_SUBMARINE")
        check(command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2) == M.MAV_RESULT_ACCEPTED,
              "DEPTH_HOLD")
        stick["z"] = 500                            # centred heave stick = hold depth
        time.sleep(2.0)
        check(arm(), "sub armed")
        stick["z"] = 150                            # dive
        time.sleep(5.0)
        stick["z"] = 500                            # hold
        time.sleep(4.0)
        hud1 = state.get("VFR_HUD")
        time.sleep(4.0)
        hud2 = state.get("VFR_HUD")
        a1 = hud1.alt if hud1 else 0
        a2 = hud2.alt if hud2 else 0
        print("  depth %.2f m -> %.2f m" % (-a1, -a2))
        check(a2 < -1.0, "sub dived below 1 m (%.2f m)" % -a2)
        check(abs(a2 - a1) < 0.3, "depth hold (moved %.2f m)" % (a2 - a1))
        stick["z"] = 900                            # surface
        time.sleep(6.0)
        stick["z"] = 500
        check(command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0) == M.MAV_RESULT_ACCEPTED, "sub disarmed")

        # ---------------- quad again ----------------
        print("quad:")
        check(set_param("VEHICLE_TYPE", 0), "VEHICLE_TYPE = 0 accepted")
        check(heartbeat_type(M.MAV_TYPE_QUADROTOR), "heartbeat MAV_TYPE_QUADROTOR")
        check(command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2) == M.MAV_RESULT_ACCEPTED,
              "ALT_HOLD")
        stick["z"] = 500
        time.sleep(2.0)
        check(arm(), "quad armed")
        stick["z"] = 850
        time.sleep(3.0)
        stick["z"] = 500
        time.sleep(2.0)
        gp = state.get("GLOBAL_POSITION_INT")
        alt = gp.relative_alt / 1000.0 if gp else 0
        check(alt > 1.0, "quad climbed (%.2f m above home)" % alt)
        r = command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        check(r == M.MAV_RESULT_DENIED, "disarm in the air refused")
        set_ok = set_param("VEHICLE_TYPE", 1)
        check(not set_ok, "VEHICLE_TYPE change refused while armed")
    finally:
        stop.set()
        sitl.terminate()
        try:
            out, _ = sitl.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            sitl.kill()
            out = ""
        for line in (out or "").splitlines():
            if "simulator:" in line:
                print("  [sitl] " + line.strip())

    print("\nPASS" if not FAIL else "\nFAILED: %d" % len(FAIL))
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
