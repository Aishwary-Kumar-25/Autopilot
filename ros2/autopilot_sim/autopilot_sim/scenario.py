"""Scripted test scenarios for the autopilot SITL. Plain Python, no ROS.

A scenario is a YAML (or JSON) file:

    name: gps_loss_alt_hold
    description: GPS dies in a hover; the quad must keep flying on baro + IMU.
    vehicle: quad                    # quad | rover | boat | sub
    mode: alt_hold                   # passed to --mode
    sitl_args: [--frame, 4]          # any extra autopilot_sitl options
    duration: 40                     # simulated seconds
    vars:                            # numbers used as {name} in events; a [lo, hi] range is
      fault_t: [15, 25]              #   its midpoint in a normal run and uniform-random in
      wind_n: [-3, 3]                #   a randomized (data augmentation) run
      motor: {choice: [1, 2, 3, 4]}  # pick one (first in a normal run)
    setup: [wind {wind_n} 0 0]       # commands at t = 0
    events:
      - {t: 1, cmd: sticks 0 0 0 0.5}
      - {t: 1.5, cmd: arm}
      - {t: "{fault_t}", cmd: fault gps dropout}
    checks:
      - {check: reached_alt, min: 4, by: 10}
      - {check: alt_between, from: "{fault_t}", to: 40, min: 2, max: 12}
    known_issue: optional text - the scenario shows a firmware weakness that is not
                 fixed yet; it is reported as XFAIL (XPASS once fixed) and doesn't fail the suite

Every check is evaluated on the recorded telemetry; see CHECKS below for the list.
"""
import copy
import csv
import json
import math
import os
import random
import re

_VAR = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")


# ---------------------------------------------------------------- loading
def load(path):
    with open(path) as f:
        text = f.read()
    if path.endswith(".json"):
        scn = json.loads(text)
    else:
        import yaml  # PyYAML: ships with ROS 2; otherwise pip install pyyaml
        scn = yaml.safe_load(text)
    scn.setdefault("name", os.path.splitext(os.path.basename(path))[0])
    scn.setdefault("vehicle", "quad")
    scn.setdefault("duration", 30)
    scn.setdefault("events", [])
    scn.setdefault("checks", [])
    scn.setdefault("setup", [])
    scn.setdefault("vars", {})
    scn.setdefault("sitl_args", [])
    scn["_path"] = path
    return scn


def resolve_vars(scn, rng=None):
    """Pick a value for every var: midpoint / first choice, or random when rng is given."""
    out = {}
    for k, v in (scn.get("vars") or {}).items():
        if isinstance(v, dict) and "choice" in v:
            out[k] = rng.choice(v["choice"]) if rng else v["choice"][0]
        elif isinstance(v, (list, tuple)) and len(v) == 2:
            lo, hi = float(v[0]), float(v[1])
            out[k] = rng.uniform(lo, hi) if rng else (lo + hi) / 2.0
        else:
            out[k] = v
    return out


def _fmt_value(v):
    if isinstance(v, float):
        return ("%.4f" % v).rstrip("0").rstrip(".")
    return str(v)


def substitute(value, var_values):
    """Replace {name} in strings; a string that is exactly '{name}' becomes the number itself."""
    if isinstance(value, str):
        m = _VAR.fullmatch(value.strip())
        if m and m.group(1) in var_values:
            return var_values[m.group(1)]

        def rep(mm):
            name = mm.group(1)
            if name not in var_values:
                raise KeyError("scenario uses {%s} but has no such var" % name)
            return _fmt_value(var_values[name])
        return _VAR.sub(rep, value)
    if isinstance(value, list):
        return [substitute(v, var_values) for v in value]
    if isinstance(value, dict):
        return {k: substitute(v, var_values) for k, v in value.items()}
    return value


def _evaluate_expr(v):
    """Allow simple arithmetic like '{fault_t} + 5' after substitution."""
    if isinstance(v, str) and re.fullmatch(r"[0-9eE.+\-*/() ]+", v):
        return float(eval(v, {"__builtins__": {}}, {}))  # digits and operators only
    return v


def instantiate(scn, var_values):
    """Scenario with every {var} filled in."""
    s = copy.deepcopy(scn)
    for key in ("setup", "events", "checks", "sitl_args", "duration"):
        s[key] = substitute(s[key], var_values)
    for e in s["events"]:
        e["t"] = float(_evaluate_expr(e["t"]))
    for c in s["checks"]:
        for k, v in list(c.items()):
            c[k] = _evaluate_expr(v)
    s["duration"] = float(_evaluate_expr(s["duration"]))
    s["vars_resolved"] = var_values
    return s


# ---------------------------------------------------------------- telemetry helpers
def alt(m):
    return -m["truth"]["pos"][2]


def tilt_err_deg(m):
    e = m["est"]["rpy"]
    t = m["truth"]["rpy"]
    return max(abs(e[0] - t[0]), abs(e[1] - t[1]))


def yaw_err_deg(m):
    d = (m["est"]["rpy"][2] - m["truth"]["rpy"][2] + 180.0) % 360.0 - 180.0
    return abs(d)


def tilt_deg(m):
    r, p = math.radians(m["truth"]["rpy"][0]), math.radians(m["truth"]["rpy"][1])
    return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(r) * math.cos(p)))))


def hspeed(m):
    v = m["truth"]["vel"]
    return math.hypot(v[0], v[1])


def _window(tel, c, default_to=None):
    t0 = float(c.get("from", 0.0))
    t1 = float(c.get("to", default_to if default_to is not None else 1e9))
    return [m for m in tel if t0 <= m["t"] <= t1]


# ---------------------------------------------------------------- checks
def _c_reached_alt(tel, c):
    by = float(c.get("by", 1e9))
    best = max((alt(m) for m in tel if m["t"] <= by), default=float("nan"))
    return best >= float(c["min"]), "max altitude %.2f m by t=%g (need >= %g)" % (best, by, c["min"])


def _c_alt_between(tel, c):
    w = _window(tel, c)
    if not w:
        return False, "no data in window"
    lo, hi = min(alt(m) for m in w), max(alt(m) for m in w)
    ok = lo >= float(c.get("min", -1e9)) and hi <= float(c.get("max", 1e9))
    return ok, "altitude %.2f..%.2f m (allowed %s..%s)" % (lo, hi, c.get("min", "-"), c.get("max", "-"))


def _c_tilt_error_below(tel, c):
    w = [m for m in _window(tel, c) if m["armed"]]
    worst = max((tilt_err_deg(m) for m in w), default=0.0)
    return worst <= float(c["deg"]), "worst roll/pitch estimate error %.2f deg (limit %g)" % (worst, c["deg"])


def _c_yaw_error_below(tel, c):
    w = [m for m in _window(tel, c) if m["armed"]]
    worst = max((yaw_err_deg(m) for m in w), default=0.0)
    return worst <= float(c["deg"]), "worst heading estimate error %.2f deg (limit %g)" % (worst, c["deg"])


def _c_alt_error_below(tel, c):
    w = [m for m in _window(tel, c) if m["armed"]]
    worst = max((abs(m["est"]["alt"] - alt(m)) for m in w), default=0.0)
    return worst <= float(c["m"]), "worst altitude estimate error %.2f m (limit %g)" % (worst, c["m"])


def _c_max_tilt_below(tel, c):
    w = _window(tel, c)
    worst = max((tilt_deg(m) for m in w), default=0.0)
    return worst <= float(c["deg"]), "max true tilt %.1f deg (limit %g)" % (worst, c["deg"])


def _c_failsafe(tel, c):
    want = str(c.get("action", "NONE")).upper()
    w = _window(tel, c, default_to=c.get("by"))
    seen = sorted({m.get("failsafe", {}).get("action", "NONE") for m in w})
    if want == "NONE":
        ok = seen in ([], ["NONE"])
        return ok, "failsafe actions seen: %s" % ",".join(seen or ["NONE"])
    hits = [m for m in w if m.get("failsafe", {}).get("action") == want]
    if c.get("reason"):
        hits = [m for m in hits if c["reason"] in m["failsafe"].get("reasons", "")]
    first = hits[0]["t"] if hits else None
    return bool(hits), ("%s from t=%.2f (%s)" % (want, first, hits[0]["failsafe"]["reasons"]) if hits
                        else "never saw %s (seen: %s)" % (want, ",".join(seen)))


def _c_armed(tel, c):
    at = float(c["at"])
    m = min(tel, key=lambda x: abs(x["t"] - at)) if tel else None
    want = bool(c.get("value", True))
    return (m is not None and m["armed"] == want), "armed=%s at t=%.2f (want %s)" % (
        m and m["armed"], m["t"] if m else -1, want)


def _c_disarmed_by(tel, c):
    by = float(c["by"])
    after_arm = False
    for m in tel:
        if m["armed"]:
            after_arm = True
        elif after_arm and m["t"] <= by:
            return True, "disarmed at t=%.2f" % m["t"]
    return False, "still armed at t=%g" % by


def _c_motor_failed(tel, c):
    by = float(c.get("by", 1e9))
    want = int(c.get("motor", 0))
    for m in tel:
        if m["t"] > by:
            break
        if m.get("motor_failed", 0) and (want == 0 or m["motor_failed"] == want):
            return True, "FC declared motor %d failed at t=%.2f" % (m["motor_failed"], m["t"])
    return False, "FC did not declare motor %s failed by t=%g" % (want or "any", by)


def _c_active_sensor(tel, c):
    by = float(c.get("by", 1e9))
    kind, name = c["sensor"], c["name"]
    for m in tel:
        if m["t"] > by:
            break
        if m["active"].get(kind) == name and m["t"] >= float(c.get("from", 0.0)):
            return True, "active %s = %s at t=%.2f" % (kind, name, m["t"])
    last = tel[-1]["active"].get(kind) if tel else "?"
    return False, "active %s never became %s (last %s)" % (kind, name, last)


def _c_moved(tel, c):
    w = _window(tel, c)
    if len(w) < 2:
        return False, "no data in window"
    a, b = w[0]["truth"]["pos"], w[-1]["truth"]["pos"]
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    ok = d >= float(c.get("min_m", 0.0)) and d <= float(c.get("max_m", 1e9))
    return ok, "moved %.2f m (allowed %s..%s)" % (d, c.get("min_m", "-"), c.get("max_m", "-"))


def _c_touchdown_speed_below(tel, c):
    worst = 0.0
    for m in _window(tel, c):
        if alt(m) < 0.5:
            worst = max(worst, m["truth"]["vel"][2])
    return worst <= float(c["mps"]), "fastest descent near the ground %.2f m/s (limit %g)" % (worst, c["mps"])


def _c_depth_between(tel, c):
    w = _window(tel, c)
    if not w:
        return False, "no data in window"
    d = [m["truth"]["pos"][2] for m in w]
    ok = min(d) >= float(c.get("min", -1e9)) and max(d) <= float(c.get("max", 1e9))
    return ok, "depth %.2f..%.2f m (allowed %s..%s)" % (min(d), max(d), c.get("min", "-"), c.get("max", "-"))


def _c_heading_change(tel, c):
    w = _window(tel, c)
    if len(w) < 2:
        return False, "no data in window"
    d = 0.0   # unwrapped: sum of the per-sample changes, so a 270 deg turn is 270, not -90
    for a, b in zip(w, w[1:]):
        d += (b["truth"]["rpy"][2] - a["truth"]["rpy"][2] + 180.0) % 360.0 - 180.0
    if c.get("direction") == "right" and d < 0 or c.get("direction") == "left" and d > 0:
        return False, "turned %.1f deg, the wrong way (wanted %s)" % (d, c["direction"])
    ok = abs(d) >= float(c.get("min_deg", 0)) and abs(d) <= float(c.get("max_deg", 1e9))
    return ok, "heading changed %.1f deg (allowed %s..%s)" % (d, c.get("min_deg", "-"), c.get("max_deg", "-"))


def _c_speed_between(tel, c):
    w = _window(tel, c)
    if not w:
        return False, "no data in window"
    s = [hspeed(m) for m in w]
    ok = min(s) >= float(c.get("min", -1)) and max(s) <= float(c.get("max", 1e9))
    return ok, "ground speed %.2f..%.2f m/s (allowed %s..%s)" % (min(s), max(s), c.get("min", "-"), c.get("max", "-"))


CHECKS = {
    "reached_alt": _c_reached_alt,               # min, by
    "alt_between": _c_alt_between,               # from, to, min, max   (true altitude, m)
    "depth_between": _c_depth_between,           # from, to, min, max   (true depth, m, down +)
    "tilt_error_below": _c_tilt_error_below,     # deg, from, to        (estimate vs truth, roll/pitch)
    "yaw_error_below": _c_yaw_error_below,       # deg, from, to
    "alt_error_below": _c_alt_error_below,       # m, from, to
    "max_tilt_below": _c_max_tilt_below,         # deg, from, to        (true tilt: did it flip?)
    "failsafe": _c_failsafe,                     # action NONE|LAND|TERMINATE, [reason], from, to/by
    "armed": _c_armed,                           # at, value
    "disarmed_by": _c_disarmed_by,               # by
    "motor_failed": _c_motor_failed,             # motor (1-based, 0 = any), by
    "active_sensor": _c_active_sensor,           # sensor imu|baro|mag|gps, name, [from], by
    "moved": _c_moved,                           # min_m, max_m, from, to (horizontal, truth)
    "speed_between": _c_speed_between,           # min, max, from, to
    "heading_change": _c_heading_change,         # min_deg, max_deg, [direction right|left], from, to
    "touchdown_speed_below": _c_touchdown_speed_below,  # mps
}


def evaluate(scn, tel):
    results = []
    for c in scn["checks"]:
        kind = c.get("check")
        fn = CHECKS.get(kind)
        if fn is None:
            results.append({"check": kind, "ok": False, "detail": "unknown check type"})
            continue
        try:
            ok, detail = fn(tel, c)
        except (KeyError, ValueError, TypeError) as e:
            ok, detail = False, "check error: %r" % (e,)
        results.append({"check": kind, "ok": bool(ok), "detail": detail, "spec": c})
    return results


# ---------------------------------------------------------------- dataset rows
def flatten(m, prefix="", out=None):
    """Nested telemetry -> flat dict (truth.pos.0, imu.0.accel.2, ...)."""
    if out is None:
        out = {}
    if isinstance(m, dict):
        for k, v in m.items():
            flatten(v, "%s%s." % (prefix, k), out)
    elif isinstance(m, list):
        for i, v in enumerate(m):
            if isinstance(v, dict) and "name" in v:
                flatten({kk: vv for kk, vv in v.items() if kk != "name"}, "%s%s." % (prefix, v["name"]), out)
            else:
                flatten(v, "%s%d." % (prefix, i), out)
    else:
        out[prefix[:-1]] = m
    return out


# ---------------------------------------------------------------- running
def schedule(client, scn):
    """Send setup commands and every timed event to the SITL (exact in simulated time)."""
    for cmd in scn["setup"]:
        client.at(0.0, cmd)
    for e in scn["events"]:
        client.at(float(e["t"]), e["cmd"])
    client.at(float(scn["duration"]), "quit")
    client.send("start")


def scenario_sitl_args(scn):
    args = ["--vehicle", str(scn["vehicle"])]
    if scn.get("mode") is not None:
        args += ["--mode", str(scn["mode"])]
    return args + [str(a) for a in scn.get("sitl_args", [])]


def run(scn, port=9100, sitl=None, speedup=0.0, seed=42, out_dir=None, run_id=None, quiet=False, wall_timeout=None):
    """Run one instantiated scenario; returns a result dict and writes CSV + JSON when out_dir is given."""
    from .sitl_client import SitlClient

    tag = run_id or scn["name"]
    log_path = os.path.join(out_dir, tag + ".sitl.log") if out_dir else os.devnull
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    client = SitlClient(port=port, sitl=sitl, sitl_args=scenario_sitl_args(scn), speedup=speedup, seed=seed,
                        log_path=log_path)
    tel, acks = [], []
    client.start()
    try:
        schedule(client, scn)
        import time
        wall_limit = wall_timeout or (scn["duration"] / speedup * 2 + 30 if speedup > 0 else scn["duration"] + 60)
        start = time.time()
        while client.running() and time.time() - start < wall_limit:
            for m in client.messages(timeout=0.2):
                if m["type"] == "telem":
                    tel.append(m)
                elif m["type"] == "ack":
                    acks.append(m)
        for m in client.messages(timeout=0.2):   # whatever was still queued
            if m["type"] == "telem":
                tel.append(m)
            elif m["type"] == "ack":
                acks.append(m)
    finally:
        client.stop()
    refused = [a for a in acks if not a["ok"]]
    checks = evaluate(scn, tel)
    for a in refused:
        checks.append({"check": "command", "ok": False, "detail": "SITL refused '%s': %s" % (a["cmd"], a["msg"])})
    if tel and tel[-1]["t"] < scn["duration"] - 1.0:
        checks.append({"check": "completed", "ok": False,
                       "detail": "telemetry stopped at t=%.1f of %.1f" % (tel[-1]["t"], scn["duration"])})
    result = {
        "scenario": scn["name"],
        "run_id": tag,
        "vehicle": scn["vehicle"],
        "seed": seed,
        "vars": scn.get("vars_resolved", {}),
        "events": scn["events"],
        "setup": scn["setup"],
        "checks": checks,
        "passed": all(c["ok"] for c in checks),
        "known_issue": scn.get("known_issue"),
        "samples": len(tel),
        "commands": [{"t": a["t"], "cmd": a["cmd"], "ok": a["ok"], "msg": a["msg"]} for a in acks],
    }
    if out_dir:
        write_dataset(os.path.join(out_dir, tag + ".csv"), tel, result)
        with open(os.path.join(out_dir, tag + ".json"), "w") as f:
            json.dump(result, f, indent=2, default=str)
    result["telemetry"] = tel
    return result


def write_dataset(path, tel, result):
    """One CSV row per telemetry sample, flat columns, plus labels (scenario, run, active faults)."""
    rows = [flatten(m) for m in tel]
    cols = []
    seen = set()
    for r in rows:
        for k in r:
            if k not in seen and k != "type":
                seen.add(k)
                cols.append(k)
    lead = ["t", "faults", "armed", "mode_name", "vehicle"]
    cols = [c for c in lead if c in seen] + [c for c in cols if c not in lead]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "run_id"] + cols)
        for r in rows:
            w.writerow([result["scenario"], result["run_id"]] + [r.get(c, "") for c in cols])
