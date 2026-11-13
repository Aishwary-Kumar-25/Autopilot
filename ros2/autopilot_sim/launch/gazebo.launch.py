"""Fly the autopilot firmware in Gazebo Harmonic (3D world, physics by Gazebo), bridged to ROS 2.

Needs Gazebo Harmonic + the ardupilot_gazebo plugin (see docs/ROS2_SIMULATOR.md).
Gazebo computes the physics of the iris quad; autopilot_sitl runs the real
flight code in lock-step with it over the ArduPilot JSON protocol (UDP 9002),
and the bridge node publishes everything to ROS 2 as in sim.launch.py.

    ros2 launch autopilot_sim gazebo.launch.py                       # iris_runway world, Gazebo GUI
    ros2 launch autopilot_sim gazebo.launch.py gui:=false            # headless (WSL without a GPU, CI)
    ros2 launch autopilot_sim gazebo.launch.py scenario:=quad_gps_loss
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = DeclareLaunchArgument
    cfg = LaunchConfiguration
    args = [
        arg("world", default_value="iris_runway.sdf", description="Gazebo world (from ardupilot_gazebo)"),
        arg("gui", default_value="true", description="false = gz server only (-s)"),
        arg("mode", default_value="alt_hold"),
        arg("scenario", default_value=""),
        arg("dataset_dir", default_value=""),
        arg("sitl", default_value=""),
        arg("gcs", default_value="127.0.0.1:14550"),
        arg("json_pwm", default_value="1100:1900", description="ESC range of the Gazebo model"),
        arg("rviz", default_value="false"),
        arg("record", default_value="false"),
        arg("bag", default_value="autopilot_gz_bag"),
    ]
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-v", "3", "-r", cfg("world")], output="screen",
                            condition=IfCondition(cfg("gui")))
    gz_headless = ExecuteProcess(cmd=["gz", "sim", "-v", "3", "-r", "-s", cfg("world")], output="screen",
                                 condition=IfCondition(PythonExpression(["not ", cfg("gui")])))
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="log")
    bridge = Node(
        package="autopilot_sim",
        executable="bridge",
        name="autopilot_bridge",
        output="screen",
        parameters=[{
            "vehicle": "quad",
            "mode": cfg("mode"),
            "scenario": cfg("scenario"),
            "dataset_dir": cfg("dataset_dir"),
            "sitl": cfg("sitl"),
            "gcs": cfg("gcs"),
            "physics": "json",
            "extra_args": ["--json-pwm ", cfg("json_pwm")],
            "publish_clock": False,     # Gazebo publishes /clock through ros_gz_bridge
            "stamp": "clock",
            "use_sim_time": True,
        }],
    )
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("autopilot_sim"), "rviz", "autopilot.rviz"])],
                parameters=[{"use_sim_time": True}], condition=IfCondition(cfg("rviz")), output="log")
    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", cfg("bag"), "--use-sim-time", "-e", "/autopilot/.*|/clock|/tf|/diagnostics"],
        condition=IfCondition(cfg("record")), output="screen")
    stop_when_done = RegisterEventHandler(
        OnProcessExit(target_action=bridge, on_exit=[Shutdown(reason="bridge finished")]),
        condition=IfCondition(PythonExpression(["'", cfg("scenario"), "' != ''"])))
    # Give Gazebo a few seconds to load the world before the firmware starts asking for physics.
    return LaunchDescription(args + [gz_gui, gz_headless, clock_bridge, TimerAction(period=4.0, actions=[bridge]),
                                     rviz, record, stop_when_done])
