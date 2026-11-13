"""ROS 2 node: autopilot_sitl <-> ROS topics.

Starts (or attaches to) autopilot_sitl with its simulator bridge and republishes
everything in ROS conventions (REP 103: ENU world "map", FLU body "base_link"):

  /autopilot/truth/odom        nav_msgs/Odometry     true state of the simulated vehicle
  /autopilot/estimate/odom     nav_msgs/Odometry     what the firmware's estimator believes
  /autopilot/truth/path        nav_msgs/Path         (and /autopilot/estimate/path) for RViz
  /autopilot/imu               sensor_msgs/Imu       raw IMU0 (orientation = firmware estimate)
  /autopilot/imu1              sensor_msgs/Imu       raw IMU1
  /autopilot/gps/fix           sensor_msgs/NavSatFix
  /autopilot/gps/vel           geometry_msgs/TwistStamped (ENU)
  /autopilot/baro              sensor_msgs/FluidPressure
  /autopilot/baro/altitude     std_msgs/Float64
  /autopilot/mag               sensor_msgs/MagneticField (tesla, FLU)
  /autopilot/battery           sensor_msgs/BatteryState
  /autopilot/pwm               std_msgs/UInt16MultiArray  outputs to ESCs / servos (us)
  /autopilot/state             std_msgs/String       JSON: armed, mode, failsafe, faults, active sensors
  /autopilot/telemetry         std_msgs/String       the full raw telemetry record (JSON) for rosbag/datasets
  /autopilot/vehicle_marker    visualization_msgs/Marker
  /diagnostics                 diagnostic_msgs/DiagnosticArray  sensor health
  /clock                       rosgraph_msgs/Clock   simulated time (publish_clock:=true)
  TF map -> base_link (truth), map -> base_link_est (estimate)

Inputs:
  /autopilot/cmd     std_msgs/String     any bridge command: "arm", "mode 2", "fault gps dropout",
                                         "motor 3 0", "wind 3 0 0", "at 20 fault imu0 stuck", ...
  /autopilot/sticks  sensor_msgs/Joy     axes = roll, pitch, yaw, throttle[, forward, lateral]
  /autopilot/arm     std_srvs/SetBool    service
Parameters: see declare_parameter() calls below.
"""
import json
import math
import os
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, Joy, MagneticField, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String, UInt16MultiArray
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from . import frames
from . import scenario as S
from .sitl_client import SitlClient


def _stamp(t):
    msg = TimeMsg()
    msg.sec = int(math.floor(t))
    msg.nanosec = int((t - math.floor(t)) * 1e9)
    return msg


class AutopilotBridge(Node):
    def __init__(self):
        super().__init__("autopilot_bridge")
        any_type = ParameterDescriptor(dynamic_typing=True)   # launch args arrive as "0", "2", "1.5", ...
        p = lambda name, default: self.declare_parameter(name, default, any_type)  # noqa: E731
        p("sitl", "")                 # path to autopilot_sitl ("" = $AUTOPILOT_SITL or <repo>/build)
        p("launch_sitl", True)        # false: attach to an autopilot_sitl you started yourself
        p("host", "127.0.0.1")
        p("port", 9100)
        p("vehicle", "quad")          # quad | rover | boat | sub
        p("mode", "")                 # e.g. alt_hold, 1, 2 ("" = SITL default)
        p("speedup", 1.0)             # 1 = real time; >1 faster; 0 = as fast as possible
        p("bridge_rate", 50.0)        # telemetry Hz (simulated time)
        p("gcs", "127.0.0.1:14550")   # MAVLink to QGroundControl at the same time ("off" to disable)
        p("physics", "internal")      # internal | json (Gazebo / ardupilot_gazebo)
        p("extra_args", "")           # anything else for autopilot_sitl, space separated
        p("seed", 42)
        p("scenario", "")             # scenario YAML: run it, report checks, then stop
        p("dataset_dir", "")          # with a scenario: also write CSV + JSON there
        p("stamp", "telemetry")       # telemetry: stamps = simulated time; clock: this node's ROS clock
        p("publish_clock", True)      # publish /clock from simulated time (use_sim_time for others)
        p("publish_tf", True)
        p("path_length", 2000)
        p("frame_id", "map")
        p("base_frame", "base_link")

        def g(n):
            v = self.get_parameter(n).value
            return "" if v is None else v
        self.frame = g("frame_id")
        self.base = g("base_frame")
        self.stamp_mode = g("stamp")
        self.path_len = int(g("path_length"))

        self.scn = None
        args = ["--vehicle", str(g("vehicle"))]
        if str(g("mode")) != "":
            args += ["--mode", str(g("mode"))]
        if g("scenario"):
            path = str(g("scenario"))
            if not os.path.exists(path):
                share = os.path.join(os.path.dirname(__file__), "..", "scenarios")
                try:
                    from ament_index_python.packages import get_package_share_directory
                    share = os.path.join(get_package_share_directory("autopilot_sim"), "scenarios")
                except Exception:  # noqa: BLE001
                    pass
                for cand in (os.path.join(share, path), os.path.join(share, path + ".yaml")):
                    if os.path.exists(cand):
                        path = cand
                        break
            base = S.load(path)
            self.scn = S.instantiate(base, S.resolve_vars(base))
            args = S.scenario_sitl_args(self.scn)
            self.get_logger().info("scenario %s: %s" % (self.scn["name"], self.scn.get("description", "")))
        if g("physics") == "json":
            args += ["--physics", "json"]
        if g("extra_args"):
            args += str(g("extra_args")).split()

        self.client = SitlClient(port=int(g("port")), host=g("host"), sitl=str(g("sitl")) or None, sitl_args=args,
                                 launch=str(g("launch_sitl")).lower() not in ("false", "0"), speedup=float(g("speedup")), seed=int(g("seed")),
                                 bridge_rate=float(g("bridge_rate")), gcs="off" if not _truthy(g("gcs")) or str(g("gcs")).lower() == "off" else str(g("gcs")))

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        pub = self.create_publisher
        self.p_truth = pub(Odometry, "/autopilot/truth/odom", 10)
        self.p_est = pub(Odometry, "/autopilot/estimate/odom", 10)
        self.p_tpath = pub(Path, "/autopilot/truth/path", 1)
        self.p_epath = pub(Path, "/autopilot/estimate/path", 1)
        self.p_imu = pub(Imu, "/autopilot/imu", best_effort)
        self.p_imu1 = pub(Imu, "/autopilot/imu1", best_effort)
        self.p_fix = pub(NavSatFix, "/autopilot/gps/fix", 10)
        self.p_gvel = pub(TwistStamped, "/autopilot/gps/vel", 10)
        self.p_baro = pub(FluidPressure, "/autopilot/baro", 10)
        self.p_balt = pub(Float64, "/autopilot/baro/altitude", 10)
        self.p_mag = pub(MagneticField, "/autopilot/mag", 10)
        self.p_bat = pub(BatteryState, "/autopilot/battery", 10)
        self.p_pwm = pub(UInt16MultiArray, "/autopilot/pwm", 10)
        self.p_state = pub(String, "/autopilot/state", 10)
        self.p_tel = pub(String, "/autopilot/telemetry", 50)
        self.p_marker = pub(Marker, "/autopilot/vehicle_marker", 1)
        self.p_diag = pub(DiagnosticArray, "/diagnostics", 10)
        self.p_result = pub(String, "/autopilot/scenario_result", 1)
        self.p_clock = pub(Clock, "/clock", 10) if _truthy(g("publish_clock")) else None
        self.tf = TransformBroadcaster(self) if _truthy(g("publish_tf")) else None

        self.create_subscription(String, "/autopilot/cmd", self.on_cmd, 10)
        self.create_subscription(Joy, "/autopilot/sticks", self.on_joy, 10)
        self.create_service(SetBool, "/autopilot/arm", self.on_arm)

        self.tpath = Path()
        self.epath = Path()
        self.tpath.header.frame_id = self.epath.header.frame_id = self.frame
        self.n = 0
        self.last_joy = 0.0
        self.recorded = []
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.finishing = False

        self.client.start()
        if self.scn:
            S.schedule(self.client, self.scn)   # also sends "start"
        else:
            self.client.send("start")
        self.get_logger().info("connected to autopilot_sitl bridge on %s:%d" % (g("host"), int(g("port"))))
        self.rx = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx.start()

    # ------------------------------------------------------------ inputs
    def on_cmd(self, msg):
        self.client.send(msg.data.strip())

    def on_joy(self, msg):
        now = time.time()
        if now - self.last_joy < 0.02:   # 50 Hz is plenty
            return
        self.last_joy = now
        a = list(msg.axes) + [0.0] * 6
        self.client.send("sticks %.3f %.3f %.3f %.3f %.3f %.3f" % tuple(a[:6]))

    def on_arm(self, req, resp):
        self.client.send("arm" if req.data else "disarm")
        resp.success = True
        resp.message = "sent"
        return resp

    # ------------------------------------------------------------ telemetry
    def rx_loop(self):
        while rclpy.ok() and not self.done.is_set():
            if not self.client.running():
                self.finish("autopilot_sitl exited")
                return
            for m in self.client.messages(timeout=0.1):
                if m["type"] == "telem":
                    try:
                        self.publish(m)
                    except Exception as e:  # noqa: BLE001 - keep the bridge alive
                        self.get_logger().error("publish failed: %r" % (e,))
                    if self.scn is not None:
                        self.recorded.append(m)
                elif m["type"] == "ack":
                    log = self.get_logger().info if m["ok"] else self.get_logger().warn
                    if not m["cmd"].startswith("at ") or not m["ok"] or m["msg"] != "scheduled":
                        log("[%.2f] %s -> %s" % (m["t"], m["cmd"], m["msg"] or ("ok" if m["ok"] else "refused")))

    def finish(self, why):
        with self.lock:
            if self.finishing:
                return
            self.finishing = True
        try:
            self._report(why)
        finally:
            self.done.set()   # main loop exits only after the report and dataset are written

    def _report(self, why):
        if self.scn is not None and not self.recorded:
            self.get_logger().error("autopilot_sitl stopped before sending telemetry (%s)" % why)
        if self.scn is not None:
            checks = S.evaluate(self.scn, self.recorded)
            passed = all(c["ok"] for c in checks)
            for c in checks:
                (self.get_logger().info if c["ok"] else self.get_logger().error)(
                    "%s %s: %s" % ("PASS" if c["ok"] else "FAIL", c["check"], c["detail"]))
            self.get_logger().info("scenario %s %s" % (self.scn["name"], "PASSED" if passed else "FAILED"))
            result = {"scenario": self.scn["name"], "passed": passed, "checks": checks}
            self.p_result.publish(String(data=json.dumps(result, default=str)))
            ddir = self.get_parameter("dataset_dir").value
            if ddir:
                os.makedirs(ddir, exist_ok=True)
                S.write_dataset(os.path.join(ddir, self.scn["name"] + ".csv"), self.recorded,
                                {"scenario": self.scn["name"], "run_id": self.scn["name"]})
                with open(os.path.join(ddir, self.scn["name"] + ".json"), "w") as f:
                    json.dump(result, f, indent=2, default=str)
        else:
            self.get_logger().info(why)

    def stamp_of(self, m):
        return _stamp(m["t"]) if self.stamp_mode == "telemetry" else self.get_clock().now().to_msg()

    def publish(self, m):
        st = self.stamp_of(m)
        if self.p_clock is not None:
            self.p_clock.publish(Clock(clock=_stamp(m["t"])))
        self.p_tel.publish(String(data=json.dumps(m)))

        tq = frames.quat_ned_frd_to_enu_flu(m["truth"]["q"])
        eq = frames.quat_ned_frd_to_enu_flu(m["est"]["q"])
        self.p_truth.publish(self.odom(st, self.base, m["truth"]["pos"], m["truth"]["vel"], tq, m["truth"]["omega"]))
        self.p_est.publish(self.odom(st, self.base + "_est", m["est"]["pos"], m["est"]["vel"], eq, m["est"]["omega"]))
        if self.tf is not None:
            self.tf.sendTransform([self.tf_msg(st, self.base, m["truth"]["pos"], tq),
                                   self.tf_msg(st, self.base + "_est", m["est"]["pos"], eq)])

        # IMUs: raw readings in FLU; orientation = the firmware's estimate
        for i, imu in enumerate(m.get("imu", [])[:2]):
            msg = Imu()
            msg.header.stamp = st
            msg.header.frame_id = self.base
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = eq
            a = frames.frd_to_flu(imu["accel"])
            w = frames.frd_to_flu(imu["gyro"])
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (_f(v) for v in a)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (_f(v) for v in w)
            (self.p_imu if i == 0 else self.p_imu1).publish(msg)

        g = m["gps"]
        fix = NavSatFix()
        fix.header.stamp = st
        fix.header.frame_id = self.base
        fix.status.status = NavSatStatus.STATUS_FIX if g["fix"] >= 3 else NavSatStatus.STATUS_NO_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = _f(g["lat"]), _f(g["lon"]), _f(g["alt"])
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.p_fix.publish(fix)
        gv = TwistStamped()
        gv.header.stamp = st
        gv.header.frame_id = self.frame
        gv.twist.linear.x, gv.twist.linear.y, gv.twist.linear.z = (_f(v) for v in frames.ned_to_enu(g["vel"]))
        self.p_gvel.publish(gv)

        bp = FluidPressure()
        bp.header.stamp = st
        bp.header.frame_id = self.base
        bp.fluid_pressure = _f(m["baro"]["pressure"])
        self.p_baro.publish(bp)
        self.p_balt.publish(Float64(data=_f(m["baro"]["alt"])))

        mg = MagneticField()
        mg.header.stamp = st
        mg.header.frame_id = self.base
        f = frames.frd_to_flu(m["mag"])
        mg.magnetic_field.x, mg.magnetic_field.y, mg.magnetic_field.z = (_f(v) * 1e-4 for v in f)  # gauss -> T
        self.p_mag.publish(mg)

        b = BatteryState()
        b.header.stamp = st
        b.voltage = _f(m["battery"]["v"])
        b.current = -_f(m["battery"]["a"])   # ROS: negative = discharging
        b.percentage = _f(m["battery"]["pct"]) / 100.0
        b.present = True
        self.p_bat.publish(b)

        self.p_pwm.publish(UInt16MultiArray(data=[int(v) for v in m["pwm"]]))
        state = {k: m.get(k) for k in ("t", "vehicle", "armed", "mode", "mode_name", "failsafe", "faults",
                                       "motor_failed", "active")}
        self.p_state.publish(String(data=json.dumps(state)))

        self.n += 1
        if self.n % 5 == 0:   # paths / marker / diagnostics at 1/5 of the telemetry rate
            self.append_path(self.tpath, st, m["truth"]["pos"], tq)
            self.append_path(self.epath, st, m["est"]["pos"], eq)
            self.p_tpath.publish(self.tpath)
            self.p_epath.publish(self.epath)
            self.p_marker.publish(self.marker(st, m))
        if self.n % 25 == 0:
            self.p_diag.publish(self.diagnostics(st, m))

    # ------------------------------------------------------------ message builders
    def odom(self, st, child, pos_ned, vel_ned, q_enu, omega_frd):
        o = Odometry()
        o.header.stamp = st
        o.header.frame_id = self.frame
        o.child_frame_id = child
        p = frames.ned_to_enu(pos_ned)
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (_f(v) for v in p)
        o.pose.pose.orientation.w, o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z = q_enu
        v = frames.enu_vector_to_flu_body(q_enu, frames.ned_to_enu(vel_ned))   # twist is in the child frame
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (_f(x) for x in v)
        w = frames.frd_to_flu(omega_frd)
        o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = (_f(x) for x in w)
        return o

    def tf_msg(self, st, child, pos_ned, q_enu):
        t = TransformStamped()
        t.header.stamp = st
        t.header.frame_id = self.frame
        t.child_frame_id = child
        p = frames.ned_to_enu(pos_ned)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (_f(v) for v in p)
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = q_enu
        return t

    def append_path(self, path, st, pos_ned, q_enu):
        ps = PoseStamped()
        ps.header.stamp = st
        ps.header.frame_id = self.frame
        p = frames.ned_to_enu(pos_ned)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (_f(v) for v in p)
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = q_enu
        path.header.stamp = st
        path.poses.append(ps)
        if len(path.poses) > self.path_len:
            del path.poses[: len(path.poses) - self.path_len]

    def marker(self, st, m):
        mk = Marker()
        mk.header.stamp = st
        mk.header.frame_id = self.base
        mk.ns = "vehicle"
        mk.id = 0
        mk.action = Marker.ADD
        body = m.get("body", "quad")
        mk.type = Marker.CUBE
        sizes = {"quad": (0.45, 0.45, 0.08), "rover": (0.45, 0.30, 0.15), "boat": (1.0, 0.4, 0.2),
                 "sub": (0.46, 0.34, 0.25), "external": (0.5, 0.5, 0.1)}
        mk.scale.x, mk.scale.y, mk.scale.z = sizes.get(body, (0.4, 0.4, 0.1))
        fs = m.get("failsafe", {}).get("action", "NONE")
        if fs != "NONE":
            mk.color.r, mk.color.g, mk.color.b = 1.0, 0.2, 0.1
        elif m.get("faults"):
            mk.color.r, mk.color.g, mk.color.b = 1.0, 0.7, 0.0
        else:
            mk.color.r, mk.color.g, mk.color.b = 0.1, 0.6, 1.0
        mk.color.a = 0.9
        mk.pose.orientation.w = 1.0
        return mk

    def diagnostics(self, st, m):
        arr = DiagnosticArray()
        arr.header.stamp = st
        level = {"HEALTHY": DiagnosticStatus.OK, "DEGRADED": DiagnosticStatus.WARN}
        for name, h in m.get("health", {}).items():
            s = DiagnosticStatus()
            s.name = "autopilot/sensors/" + name
            s.hardware_id = name
            s.level = level.get(h, DiagnosticStatus.ERROR)
            s.message = h
            arr.status.append(s)
        fs = m.get("failsafe", {})
        s = DiagnosticStatus()
        s.name = "autopilot/failsafe"
        s.level = DiagnosticStatus.OK if fs.get("action", "NONE") == "NONE" else DiagnosticStatus.ERROR
        s.message = fs.get("action", "NONE") + (" (%s)" % fs["reasons"] if fs.get("reasons") else "")
        s.values = [KeyValue(key="faults", value=m.get("faults", "")),
                    KeyValue(key="active_imu", value=m["active"].get("imu", ""))]
        arr.status.append(s)
        return arr


def _truthy(v):
    return str(v).lower() not in ("false", "0", "", "none")


def _f(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float("nan")
    return v


def main(args=None):
    rclpy.init(args=args)
    node = AutopilotBridge()
    try:
        while rclpy.ok() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.done.set()
        node.client.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
