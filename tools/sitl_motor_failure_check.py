#!/usr/bin/env python3
"""
Motor failure in the real-time SITL, seen from the ground station (pymavlink,
MAVLink 2 over UDP, the way QGroundControl sees it).

Starts build/autopilot_sitl with a motor set to die in flight, then as the GCS:
ALT_HOLD -> arm -> climb -> hover -> (motor dies) -> checks what the GCS sees.

  hexa (default):  STATUSTEXT "Motor 3 failed", no failsafe, stays in the air
                   at the same altitude and level, motor 3's PWM at idle; then
                   the pilot lands it on the stick and disarms.
  quad (--frame 4): STATUSTEXT "Motor 3 failed" + "No yaw control left",
                   "Failsafe LAND: MOTOR", it spins down, lands and disarms by itself.

    pip install pymavlink
    python3 tools/sitl_motor_failure_check.py [--frame 6|4] [--sitl build/autopilot_sitl]

Exit code 0 = all checks passed.
"""
import argparse
import os
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
    ap.add_argument("--port", type=int, default=14562)
    ap.add_argument("--frame", type=int, default=6)
    ap.add_argument("--motor", type=int, default=3, help="1-based motor to kill")
    args = ap.parse_args()
    hexa = args.frame >= 6
    fail_after = 7.0   # seconds after arming

    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:%d" % args.port, source_system=255, source_component=190)
    sitl = subprocess.Popen([args.sitl, "--gcs", "127.0.0.1:%d" % args.port, "--duration", "60", "--storage", "none",
                             "--frame", str(args.frame), "--fail-motor", "%d@%g" % (args.motor, fail_after)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    console = []
    threading.Thread(target=lambda: [console.append(l.rstrip()) for l in sitl.stdout], daemon=True).start()
    stop = threading.Event()
    stick = {"x": 0, "y": 0, "z": 0, "r": 0}
    texts, state = [], {}
    lock = threading.Lock()
    inbox = []

    try:
        hb = gcs.wait_heartbeat(timeout=10)
        check(hb is not None, "vehicle heartbeat received")
        if hb is None:
            return
        want = mavutil.mavlink.MAV_TYPE_HEXAROTOR if args.frame == 6 else (
            mavutil.mavlink.MAV_TYPE_QUADROTOR if args.frame == 4 else hb.type)
        check(hb.type == want, "MAV_TYPE for a %d-rotor frame (got %d)" % (args.frame, hb.type))

        def gcs_tx():
            n = 0
            while not stop.is_set():
                if n % 20 == 0:
                    gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                gcs.mav.manual_control_send(1, stick["x"], stick["y"], stick["z"], stick["r"], 0)
                n += 1
                time.sleep(0.05)

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

        def command(cmd, *p):
            p = list(p) + [0] * (7 - len(p))
            gcs.mav.command_long_send(1, 1, cmd, 0, *p)
            a = wait_for(lambda m: m.get_type() == "COMMAND_ACK" and m.command == cmd, 3)
            return a.result if a else None

        def alt():
            gp = state.get("GLOBAL_POSITION_INT")
            return gp.relative_alt / 1000.0 if gp else 0.0

        threading.Thread(target=gcs_rx, daemon=True).start()
        threading.Thread(target=gcs_tx, daemon=True).start()
        time.sleep(1.0)

        check(command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 2)
              == mavutil.mavlink.MAV_RESULT_ACCEPTED, "ALT_HOLD")
        time.sleep(1.5)
        check(command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) == mavutil.mavlink.MAV_RESULT_ACCEPTED, "armed")
        t_arm = time.time()
        stick["z"] = 850
        time.sleep(3.0)
        stick["z"] = 500
        while time.time() - t_arm < fail_after - 0.3:
            time.sleep(0.1)
        alt_before = alt()
        print("  hovering at %.2f m, motor %d dies in a moment" % (alt_before, args.motor))
        check(alt_before > 1.5, "hovering above 1.5 m before the failure (%.2f m)" % alt_before)

        failed = wait_for(lambda m: m.get_type() == "STATUSTEXT" and ("Motor %d failed" % args.motor) in m.text, 3)
        check(failed is not None, "GCS told 'Motor %d failed'" % args.motor)

        if hexa:
            lo, hi, tilt = 1e9, 0.0, 0.0
            end = time.time() + 8.0
            while time.time() < end:
                a = alt()
                lo, hi = min(lo, a), max(hi, a)
                att = state.get("ATTITUDE")
                if att:
                    tilt = max(tilt, abs(att.roll), abs(att.pitch))
                time.sleep(0.1)
            print("  8 s on %d motors: alt %.2f..%.2f m, max tilt %.1f deg" % (args.frame - 1, lo, hi, tilt * 57.3))
            check(lo > alt_before - 0.7 and hi < alt_before + 0.7, "holds altitude on %d motors" % (args.frame - 1))
            check(tilt < 0.2, "stays level (max tilt %.1f deg)" % (tilt * 57.3))
            check(not any(t.startswith("Failsafe") for t in texts), "no failsafe: the pilot keeps control")
            so = state.get("SERVO_OUTPUT_RAW")
            raw = getattr(so, "servo%d_raw" % args.motor) if so else None
            check(raw == 1000, "motor %d output at idle (%s us)" % (args.motor, raw))
            hbm = state.get("HEARTBEAT")
            check(hbm is not None and (hbm.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED), "still armed")
            # The pilot brings it down.
            stick["z"] = 150
            end = time.time() + 20
            while time.time() < end and alt() > 0.15:
                time.sleep(0.2)
            check(alt() < 0.3, "pilot landed it on 5 motors")
            stick["z"] = 0
            t0, r = time.time(), None
            while time.time() - t0 < 10 and r != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                time.sleep(0.5)
                r = command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
            check(r == mavutil.mavlink.MAV_RESULT_ACCEPTED, "disarmed on the ground")
        else:
            check(wait_for(lambda m: m.get_type() == "STATUSTEXT" and "No yaw control" in m.text, 2) is not None,
                  "GCS told yaw is abandoned")
            check(wait_for(lambda m: m.get_type() == "STATUSTEXT" and "Failsafe LAND" in m.text and "MOTOR" in m.text,
                           2) is not None, "Failsafe LAND: MOTOR")
            end = time.time() + 20
            disarmed = False
            while time.time() < end:
                hbm = state.get("HEARTBEAT")
                if hbm is not None and not (hbm.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    disarmed = True
                    break
                time.sleep(0.2)
            check(disarmed and alt() < 0.3, "landed and disarmed by itself (alt %.2f m)" % alt())
    finally:
        stop.set()
        sitl.terminate()
        try:
            sitl.wait(timeout=3)
        except subprocess.TimeoutExpired:
            sitl.kill()
        for line in console:
            if "SIM:" in line or "FC:" in line:
                print("  [sitl] " + line.strip())

    print("\n%s: %d failure(s)" % ("PASS" if not FAIL else "FAIL", len(FAIL)))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main() or (1 if FAIL else 0))
