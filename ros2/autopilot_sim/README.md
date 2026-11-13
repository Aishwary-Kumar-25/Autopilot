# autopilot_sim

ROS 2 (Jazzy) package and plain-Python tools for testing the autopilot firmware in
simulation: scripted fault scenarios with PASS/FAIL checks, randomized flight
datasets, ROS topics/TF/RViz/rosbag, and Gazebo Harmonic physics.

```bash
python3 -m autopilot_sim.run_scenarios                 # no ROS needed (pip install pyyaml)
ros2 run autopilot_sim run_scenarios --runs 20 --out ~/datasets/v1
ros2 launch autopilot_sim sim.launch.py                # live, with RViz
ros2 launch autopilot_sim gazebo.launch.py             # firmware flying the Gazebo iris
```

Needs the firmware built first (`cmake -B build && cmake --build build` at the
repo root); set `AUTOPILOT_SITL` if the binary is somewhere else.

Full guide, including Windows (WSL2) setup: [docs/ROS2_SIMULATOR.md](../../docs/ROS2_SIMULATOR.md).
