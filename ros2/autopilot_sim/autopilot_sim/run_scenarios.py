"""Run scenario files against autopilot_sitl: pass/fail tests, or randomized runs for datasets.

    # test suite: every scenario once, nominal values, exit code 1 if any check fails
    python3 -m autopilot_sim.run_scenarios scenarios/*.yaml

    # data augmentation: 50 randomized runs of each scenario, 4 in parallel, CSV + JSON per run
    python3 -m autopilot_sim.run_scenarios scenarios/*.yaml --runs 50 --seed 1 --jobs 4 --out datasets/v1

With ROS 2 installed the same thing is `ros2 run autopilot_sim run_scenarios ...`.
"""
import argparse
import concurrent.futures
import glob
import json
import os
import random
import sys
import time

from . import scenario as S


def _scenario_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "scenarios"),
                 os.path.join(here, "..", "..", "..", "..", "share", "autopilot_sim", "scenarios")):
        if os.path.isdir(cand):
            return os.path.abspath(cand)
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("autopilot_sim"), "scenarios")
    except Exception:  # noqa: BLE001 - ROS not installed
        return None


def expand(paths):
    out = []
    for p in paths:
        if os.path.isdir(p):
            out += sorted(glob.glob(os.path.join(p, "*.yaml")) + glob.glob(os.path.join(p, "*.json")))
        elif any(ch in p for ch in "*?["):
            out += sorted(glob.glob(p))
        elif not os.path.exists(p) and _scenario_dir() and os.path.exists(os.path.join(_scenario_dir(), p)):
            out.append(os.path.join(_scenario_dir(), p))
        elif not os.path.exists(p) and _scenario_dir() and os.path.exists(os.path.join(_scenario_dir(), p + ".yaml")):
            out.append(os.path.join(_scenario_dir(), p + ".yaml"))
        else:
            out.append(p)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="*", help="scenario files, directories or names (default: all bundled)")
    ap.add_argument("--sitl", help="path to autopilot_sitl (default: $AUTOPILOT_SITL or <repo>/build)")
    ap.add_argument("--runs", type=int, default=0,
                    help="randomized runs per scenario (data augmentation); 0 = one nominal run (test mode)")
    ap.add_argument("--seed", type=int, default=1, help="random seed for randomized runs")
    ap.add_argument("--jobs", type=int, default=max(1, min(4, (os.cpu_count() or 2) - 1)), help="parallel SITLs")
    ap.add_argument("--speedup", type=float, default=0.0, help="0 = as fast as possible (default)")
    ap.add_argument("--port", type=int, default=9200, help="first bridge port; job k uses port+k")
    ap.add_argument("--out", help="write <run>.csv (one row per sample) + <run>.json (labels, checks)")
    ap.add_argument("--sitl-args", default="",
                    help="extra autopilot_sitl options for every run, e.g. \"--physics json\" (then use --jobs 1)")
    ap.add_argument("--list", action="store_true", help="list the bundled scenarios and exit")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every check, not just failures")
    args = ap.parse_args(argv)

    if args.list or not args.scenarios:
        d = _scenario_dir()
        files = sorted(glob.glob(os.path.join(d, "*.yaml"))) if d else []
        if args.list:
            for f in files:
                scn = S.load(f)
                print("%-28s %-6s %s" % (scn["name"], scn["vehicle"], scn.get("description", "").strip().split("\n")[0]))
            return 0
        args.scenarios = files

    files = expand(args.scenarios)
    if not files:
        print("no scenarios found", file=sys.stderr)
        return 2
    extra = args.sitl_args.split()
    jobs = []
    master = random.Random(args.seed)
    for f in files:
        base = S.load(f)
        base["sitl_args"] = list(base.get("sitl_args", [])) + extra
        if args.runs <= 0:
            jobs.append((S.instantiate(base, S.resolve_vars(base)), 42, base["name"]))
        else:
            for k in range(args.runs):
                rng = random.Random(master.getrandbits(32))
                jobs.append((S.instantiate(base, S.resolve_vars(base, rng)), rng.getrandbits(31),
                             "%s_%04d" % (base["name"], k)))

    print("running %d scenario run(s) on %d parallel SITL(s)%s" % (len(jobs), args.jobs,
          ", writing to " + args.out if args.out else ""))
    ports = list(range(args.port, args.port + args.jobs))
    free = list(ports)
    import threading
    lock = threading.Lock()

    def work(job):
        scn, seed, tag = job
        with lock:
            port = free.pop()
        try:
            return S.run(scn, port=port, sitl=args.sitl, speedup=args.speedup, seed=seed, out_dir=args.out,
                         run_id=tag)
        except Exception as e:  # noqa: BLE001 - report and carry on with the other runs
            return {"scenario": scn["name"], "run_id": tag, "passed": False, "samples": 0, "known_issue": None,
                    "checks": [{"check": "run", "ok": False, "detail": "error: %s" % e}]}
        finally:
            with lock:
                free.append(port)

    t0 = time.time()
    failed = xfailed = 0
    summary = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for res in ex.map(work, jobs):
            res.pop("telemetry", None)
            summary.append(res)
            known = res.get("known_issue")
            if known:   # expected to fail until the firmware is fixed: report, don't count
                mark = "XPASS" if res["passed"] else "XFAIL"
                xfailed += 0 if res["passed"] else 1
            else:
                mark = "PASS" if res["passed"] else "FAIL"
                failed += 0 if res["passed"] else 1
            print("%s  %-34s %6d samples" % (mark, res["run_id"], res["samples"]))
            if known and (args.verbose or not res["passed"]):
                print("        known issue: %s" % known)
            for c in res["checks"]:
                if args.verbose or not c["ok"]:
                    print("        %s %-22s %s" % ("ok  " if c["ok"] else "FAIL", c["check"], c["detail"]))
            sys.stdout.flush()
    print("%d/%d passed%s in %.1f s" % (len(jobs) - failed - xfailed, len(jobs),
                                         " (%d known issue%s)" % (xfailed, "s" if xfailed != 1 else "") if xfailed else "",
                                         time.time() - t0))
    if args.out:
        with open(os.path.join(args.out, "index.json"), "w") as f:
            json.dump({"runs": summary, "seed": args.seed, "randomized_runs_per_scenario": args.runs}, f,
                      indent=2, default=str)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
