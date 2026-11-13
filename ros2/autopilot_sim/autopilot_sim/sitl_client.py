"""Talk to autopilot_sitl over its simulator bridge (sim/sim_bridge.hpp). Plain Python, no ROS.

    client = SitlClient(port=9100, sitl="build/autopilot_sitl", sitl_args=["--vehicle", "quad"], speedup=0)
    client.start()                         # launches the SITL and connects
    client.send("at 1 arm")                # commands; "at <t>" runs them at simulated time t
    client.send("start")                   # simulated time starts now
    for msg in client.messages(timeout=1): # telemetry dicts ({"type": "telem", ...}) and acks
        ...
    client.stop()
"""
import json
import os
import shutil
import socket
import subprocess
import time


def find_sitl(explicit=None):
    """Locate the autopilot_sitl binary: explicit path, $AUTOPILOT_SITL, PATH, or <repo>/build."""
    candidates = []
    if explicit:
        candidates.append(explicit)
    if os.environ.get("AUTOPILOT_SITL"):
        candidates.append(os.environ["AUTOPILOT_SITL"])
    on_path = shutil.which("autopilot_sitl")
    if on_path:
        candidates.append(on_path)
    here = os.path.abspath(os.path.dirname(__file__))
    for up in range(1, 7):  # ros2/autopilot_sim/autopilot_sim -> repo root, also from an install/ tree
        root = os.path.abspath(os.path.join(here, *[".."] * up))
        candidates.append(os.path.join(root, "build", "autopilot_sitl"))
    cwd = os.getcwd()
    candidates += [os.path.join(cwd, "build", "autopilot_sitl"), os.path.join(cwd, "autopilot_sitl")]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return os.path.abspath(c)
    raise FileNotFoundError(
        "autopilot_sitl not found. Build the firmware (cmake -B build && cmake --build build) and/or "
        "set AUTOPILOT_SITL=/path/to/build/autopilot_sitl")


def _die_with_parent():
    """Linux: the SITL gets SIGTERM if this process dies, so no orphan keeps the bridge port."""
    try:
        import ctypes
        import signal
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except Exception:  # noqa: BLE001 - not Linux: nothing to do
        pass


class SitlClient:
    def __init__(self, port=9100, host="127.0.0.1", sitl=None, sitl_args=(), launch=True, speedup=1.0,
                 seed=42, bridge_rate=50, gcs="off", log_path=None):
        self.port = port
        self.host = host
        self.launch = launch
        self.sitl = sitl
        self.sitl_args = list(sitl_args)
        self.speedup = speedup
        self.seed = seed
        self.bridge_rate = bridge_rate
        self.gcs = gcs
        self.log_path = log_path
        self.proc = None
        self.sock = None
        self._log = None
        self.last_telem = None

    # ---- lifecycle ----
    def start(self, timeout=10.0):
        if self.launch:
            exe = find_sitl(self.sitl)
            cmd = [exe, "--bridge", str(self.port), "--bridge-rate", str(self.bridge_rate), "--speedup",
                   str(self.speedup), "--seed", str(self.seed), "--gcs", self.gcs, "--storage", "none",
                   "--quiet"] + self.sitl_args
            # log_path: file for the SITL's console output; None = inherit ours (shows up in ros2 launch)
            self._log = open(self.log_path, "w") if self.log_path else None
            self.proc = subprocess.Popen(cmd, stdout=self._log, stderr=subprocess.STDOUT, preexec_fn=_die_with_parent)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("0.0.0.0" if self.host not in ("127.0.0.1", "localhost") else "127.0.0.1", 0))
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc is not None and self.proc.poll() is not None:
                raise RuntimeError("autopilot_sitl exited with code %d (see %s)" % (self.proc.returncode,
                                                                                 self.log_path or "its output"))
            self.send("hello")
            for m in self.messages(timeout=0.2):
                if m.get("type") == "ack" and m.get("cmd") == "hello":
                    return self
        self.stop()
        raise TimeoutError("no answer from the SITL bridge on %s:%d" % (self.host, self.port))

    def stop(self, grace=3.0):
        if self.sock is not None:
            try:
                self.send("quit")
            except OSError:
                pass
        if self.proc is not None:
            try:
                self.proc.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=grace)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            self.proc = None
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self._log not in (None, subprocess.DEVNULL):
            self._log.close()
        self._log = None

    def running(self):
        return self.proc is None or self.proc.poll() is None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # ---- I/O ----
    def send(self, command):
        self.sock.sendto(command.encode(), (self.host, self.port))

    def at(self, t, command):
        self.send("at %.4f %s" % (t, command))

    def messages(self, timeout=0.0):
        """Yield every message received within `timeout` seconds (non-blocking when 0)."""
        end = time.time() + timeout
        while True:
            remaining = end - time.time()
            self.sock.settimeout(max(0.0, remaining) if remaining > 0 else 0.0)
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, socket.timeout):
                return
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            if msg.get("type") == "telem":
                self.last_telem = msg
            yield msg
