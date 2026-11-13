"""Run the autopilot firmware (autopilot_sitl) in simulation and bridge it to ROS 2.

    ros2 launch autopilot_sim sim.launch.py                         # quad, real time, RViz
    ros2 launch autopilot_sim sim.launch.py vehicle:=rover mode:=1
    ros2 launch autopilot_sim sim.launch.py scenario:=quad_gps_loss speedup:=4 record:=true
    ros2 launch autopilot_sim sim.launch.py rviz:=false gcs:=off    # headless

QGroundControl can connect at the same time (UDP 14550) unless gcs:=off.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = DeclareLaunchArgument
    cfg = LaunchConfiguration
    args = [
        arg("vehicle", default_value="quad", description="quad | rover | boat | sub"),
        arg("mode", default_value="", description="flight mode: 0/1/2 or manual/stabilize/alt_hold"),
        arg("speedup", default_value="1.0", description="1 = real time, 0 = as fast as possible"),
        arg("scenario", default_value="", description="scenario name or YAML path to run, then stop"),
        arg("dataset_dir", default_value="", description="with scenario: write CSV + JSON here"),
        arg("sitl", default_value="", description="path to autopilot_sitl (default: $AUTOPILOT_SITL)"),
        arg("gcs", default_value="127.0.0.1:14550", description="MAVLink to QGroundControl, or off"),
        arg("extra_args", default_value="", description="extra autopilot_sitl options"),
        arg("port", default_value="9100", description="simulator bridge UDP port"),
        arg("seed", default_value="42", description="sensor noise seed"),
        arg("rviz", default_value="true"),
        arg("record", default_value="false", description="ros2 bag record /autopilot/* and /clock"),
        arg("bag", default_value="autopilot_bag", description="bag output folder"),
    ]
    bridge = Node(
        package="autopilot_sim",
        executable="bridge",
        name="autopilot_bridge",
        output="screen",
        parameters=[{
            "vehicle": cfg("vehicle"),
            "mode": cfg("mode"),
            "speedup": cfg("speedup"),
            "scenario": cfg("scenario"),
            "dataset_dir": cfg("dataset_dir"),
            "sitl": cfg("sitl"),
            "gcs": cfg("gcs"),
            "extra_args": cfg("extra_args"),
            "port": cfg("port"),
            "seed": cfg("seed"),
            "publish_clock": True,
            "stamp": "telemetry",
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("autopilot_sim"), "rviz", "autopilot.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(cfg("rviz")),
        output="log",
    )
    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", cfg("bag"), "--use-sim-time", "-e", "/autopilot/.*|/clock|/tf|/diagnostics"],
        condition=IfCondition(cfg("record")),
        output="screen",
    )
    # A scenario run ends the whole launch when the bridge exits.
    stop_when_done = RegisterEventHandler(
        OnProcessExit(target_action=bridge, on_exit=[Shutdown(reason="bridge finished")]),
        condition=IfCondition(PythonExpression(["'", cfg("scenario"), "' != ''"])),
    )
    return LaunchDescription(args + [bridge, rviz, record, stop_when_done])
