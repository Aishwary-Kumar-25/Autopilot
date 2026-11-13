# Testing the autopilot without hardware: the ROS 2 simulator

We don't have flight hardware, so the firmware is tested in simulation. The
**real flight code** (sensor manager, estimator, controllers, mixers, failsafe,
motor-failure detection) runs unchanged in `autopilot_sitl`. Only the sensors
and the vehicle are simulated. On top of that there are three layers:

1. **Scripted test scenarios.** YAML files say what happens and when ("GPS dies
   at t = 15 s") and what must hold ("altitude stays between 2 and 9 m"). The
   whole suite runs in about 3 seconds, faster than real time, with a PASS/FAIL
   report. It works with or without ROS.
2. **Data generation for data augmentation.** The same scenarios run hundreds
   of times with randomized wind, vibration, payload, fault timing and sensor
   noise. Every run is saved as a CSV (true state, firmware estimate, raw sensor
   readings, outputs, and labels for the faults that are active) plus a JSON
   file with the run's parameters and check results.
3. **ROS 2 integration.** The `autopilot_sim` package publishes everything as
   standard ROS topics (odometry, IMU, GPS, TF, diagnostics, ...), takes commands
   and joystick input from ROS, shows the flight in RViz, records rosbags, and
   can hand the physics to **Gazebo Harmonic** for a 3D world.

```
                 commands / faults / sticks                     MAVLink 2 (UDP 14550)
 ROS 2 nodes  <---------------------------->  autopilot_sitl  <---------------------> QGroundControl
 RViz, rosbag      bridge node (UDP 9100)     (real firmware)
 scenario runner   telemetry: truth +              |  PWM out / state in (ArduPilot JSON, UDP 9002)
 datasets          estimate + sensors              v
                                         built-in physics (quad, hexa, rover, boat, ROV)
                                         or Gazebo Harmonic + ardupilot_gazebo (iris)
```

Everything here was run before it was committed: the scenario suite, 140
randomized runs, the ROS 2 node inside ROS 2 Jazzy (topics, services, rosbag),
and the firmware flying the Gazebo Harmonic iris quad through the
ardupilot_gazebo plugin.

---

## 1. Setup on Windows (WSL2)

ROS 2 runs on Linux, so on a Windows laptop use WSL2 with Ubuntu 24.04 (ROS 2
Jazzy needs 24.04). Windows 11 shows Linux GUI apps (RViz, Gazebo) directly.

**PowerShell (as administrator), once:**

```powershell
wsl --install -d Ubuntu-24.04
```

Restart if asked, open "Ubuntu 24.04" from the Start menu and create a user.
Everything below runs **inside Ubuntu**.

**Build tools and ROS 2 Jazzy:**

```bash
sudo apt update && sudo apt install -y build-essential cmake git curl python3-pip software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && source ~/.bashrc
```

**The firmware and the ROS package.** Clone into the Linux home folder, not
`/mnt/c`, because builds on the Windows drive are very slow:

```bash
git clone -b darsh https://github.com/Aishwary-Kumar-25/Autopilot.git ~/Autopilot
cd ~/Autopilot && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
(cd build && ctest)                                    # firmware unit + SITL tests

mkdir -p ~/ros2_ws/src && ln -s ~/Autopilot/ros2/autopilot_sim ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
echo 'export AUTOPILOT_SITL=$HOME/Autopilot/build/autopilot_sitl' >> ~/.bashrc
source ~/.bashrc
```

**Check that it works:**

```bash
ros2 run autopilot_sim run_scenarios            # all scenarios, ~3 s, prints PASS/FAIL
ros2 launch autopilot_sim sim.launch.py         # live quad + RViz
```

> Without ROS, for example on a teammate's Linux or macOS machine: after
> building the firmware, `pip install pyyaml` and run
> `cd ros2/autopilot_sim && python3 -m autopilot_sim.run_scenarios`. The
> scenario runner and dataset generator don't need ROS.

---

## 2. Run the test scenarios

```bash
ros2 run autopilot_sim run_scenarios --list             # what's there
ros2 run autopilot_sim run_scenarios                    # all, nominal values, exit code 1 on failure
ros2 run autopilot_sim run_scenarios quad_gps_loss -v   # one, every check printed
```

Example output:

```
PASS   quad_imu_failover                    1501 samples
PASS   quad_motor_failure                   1751 samples
XFAIL  quad_gps_spoof                       1751 samples
        known issue: Not fixed yet. The estimator uses GPS velocity for tilt aiding without ...
        FAIL tilt_error_below       worst roll/pitch estimate error 12.58 deg (limit 8)
13/14 passed (1 known issue) in 2.8 s
```

| Scenario | Vehicle | What happens | What must hold |
|---|---|---|---|
| `quad_hover` | quad | Take off, hover, land, disarm (baseline) | altitude window, estimate errors, soft touchdown |
| `quad_gps_loss` | quad | GPS receiver dies in a hover | keeps altitude and attitude on IMU + baro |
| `quad_gps_spoof` | quad | GPS position and velocity spoofed | tilt estimate stays right (**known issue**) |
| `quad_imu_failover` | quad | IMU0 drops out, freezes or returns NaN | switches to IMU1 within 2 s, keeps flying |
| `quad_baro_failure` | quad | Barometer dies in ALT_HOLD | no fly-away, no crash |
| `quad_compass_interference` | quad | Strong magnetic disturbance | roll/pitch unaffected |
| `quad_wind_gust` | quad | 6 m/s gust for 5 s | estimate and altitude hold up |
| `quad_rc_loss` | quad | Radio link lost | failsafe LAND, soft touchdown, disarm |
| `quad_motor_failure` | quad | One motor dies | FC detects the motor, lands without flipping |
| `hexa_motor_failure` | hexa | One of six motors dies | detected, keeps flying on five |
| `rover_drive` | rover | Drive, steer, hold heading, lose GPS | moves, turns the right way, holds the new heading |
| `boat_current` | boat | Crosses a water current | heading estimate stays right |
| `sub_depth_hold` | ROV | Dive to about 2 m and hold it (no GPS underwater) | depth window, attitude estimate |
| `vehicle_switch` | quad, then rover | One firmware switched from quad to rover at runtime | both fly or drive |

### Writing a scenario

A scenario is a YAML file in `ros2/autopilot_sim/scenarios/`. Times are
simulated seconds from the start. `{name}` is a variable: a `[lo, hi]` range
takes its midpoint in a normal run and a random value in randomized runs.

```yaml
name: quad_gps_loss
description: GPS receiver dies in a hover.
vehicle: quad                  # quad | rover | boat | sub
mode: alt_hold                 # 0/1/2 or manual | stabilize | alt_hold
sitl_args: [--frame, 6]        # optional: any autopilot_sitl option (here: hexacopter)
duration: 40
vars:
  fault_t: [10, 20]            # number range
  kind: {choice: [dropout, stuck, nan]}
setup: [wind 2 0 0]            # commands at t = 0
events:
  - {t: 1.0, cmd: sticks 0 0 0 0.5}
  - {t: 1.5, cmd: arm}
  - {t: 2.0, cmd: sticks 0 0 0 0.85}
  - {t: "{fault_t}", cmd: "fault gps {kind}"}
checks:
  - {check: alt_between, from: "{fault_t}", to: 40, min: 2, max: 9}
  - {check: active_sensor, sensor: imu, name: SIM_IMU1, by: "{fault_t} + 2"}
known_issue: optional; the scenario reports XFAIL instead of failing the suite
```

Checks: `reached_alt`, `alt_between`, `depth_between`, `tilt_error_below`,
`yaw_error_below`, `alt_error_below`, `max_tilt_below`, `failsafe`
(`action: NONE|LAND|TERMINATE`, optional `reason`), `armed`, `disarmed_by`,
`motor_failed`, `active_sensor`, `moved`, `speed_between`, `heading_change`
(optional `direction: right|left`), `touchdown_speed_below`. Each one is a short
function in `autopilot_sim/scenario.py`, so adding a check is easy.

**Stick meaning** (`sticks roll pitch yaw throttle [forward lateral]`, the same
as the firmware's `set_user_inputs`):
- quad: roll and pitch in -1..1 (pitch + = nose up, so negative flies forward),
  yaw rate, throttle 0..1 (in ALT_HOLD 0.5 = hold, above climbs, below descends)
- rover and boat: `sticks 0 0 <steer> <drive> <drive> 0`, with steer + = right
- ROV: throttle = heave -1..1 (negative dives, 0 holds depth), forward, lateral

---

## 3. Generate datasets (data augmentation)

```bash
ros2 run autopilot_sim run_scenarios --runs 50 --seed 1 --jobs 4 --out ~/datasets/v1
```

This runs every scenario 50 times with random variables (wind, IMU vibration,
payload mass, fault type and timing, spoofing offsets, water current, ...) and a
different sensor-noise seed each time. 14 scenarios × 10 runs take about 50
seconds on a laptop and give about 220 MB of CSV. Name scenarios to pick a
subset (`quad_imu_failover quad_motor_failure`).

Each run writes:
- `<scenario>_<n>.csv`: one row per telemetry sample (50 Hz). Columns:
  - `t` (s) and labels: `faults` (for example `imu0:nan`, `gps:dropout`,
    `motor3:0.00`, `rc:lost`, empty = nominal), `armed`, `mode_name`,
    `failsafe.action`, `failsafe.reasons`, `motor_failed`, `active.imu`, ...
  - Ground truth: `truth.pos.0-2` (NED m), `truth.vel.*`, `truth.q.*`,
    `truth.rpy.*` (deg), `truth.omega.*` (rad/s, FRD), `truth.accel.*`
  - Firmware estimate: `est.rpy.*`, `est.q.*`, `est.alt`, `est.vz_up`,
    `est.pos.*`, `est.vel.*`, `est.gyro_bias.*`, validity flags
  - Raw sensors: `imu.SIM_IMU0.accel.*` / `.gyro.*` (both IMUs),
    `baro.alt`, `baro.pressure`, `mag.*` (gauss), `gps.lat/lon/alt/vel.*/fix/sats`
  - Outputs: `pwm.0..n` (µs), `sticks.*`, `battery.v/a/pct`, `env.wind.*`
  - `health.<sensor>`: what the sensor manager thinks of each sensor
- `<scenario>_<n>.json`: the random values used, the events, the check results
  and every command the SITL acknowledged.
- `index.json`: a summary of all runs.

```python
import pandas as pd, glob
df = pd.concat(pd.read_csv(f) for f in glob.glob("datasets/v1/*.csv"))
X = df[[c for c in df if c.startswith(("imu.", "baro.", "mag.", "gps."))]]
y = df["faults"].fillna("") != ""        # e.g. train a fault detector
```

The `/autopilot/telemetry` topic carries the same record in ROS, so a rosbag
(`record:=true`) holds the full dataset too.

---

## 4. ROS 2 interface

```bash
ros2 launch autopilot_sim sim.launch.py                                   # quad, real time, RViz
ros2 launch autopilot_sim sim.launch.py vehicle:=rover mode:=1
ros2 launch autopilot_sim sim.launch.py scenario:=quad_motor_failure speedup:=5 record:=true
ros2 launch autopilot_sim sim.launch.py rviz:=false gcs:=off              # headless
```

Launch arguments: `vehicle`, `mode`, `speedup` (0 = as fast as possible),
`scenario`, `dataset_dir`, `gcs` (MAVLink target, default `127.0.0.1:14550`, or
`off`), `extra_args` (for example `"--frame 6"`), `port`, `seed`, `rviz`,
`record`, `bag`.

World frame `map` is ENU and the body frame `base_link` is FLU, as in REP 103.
The bridge converts from the firmware's NED/FRD.

| Topic | Type | |
|---|---|---|
| `/autopilot/truth/odom` | nav_msgs/Odometry | true state of the simulated vehicle |
| `/autopilot/estimate/odom` | nav_msgs/Odometry | what the firmware's estimator believes |
| `/autopilot/truth/path`, `/autopilot/estimate/path` | nav_msgs/Path | for RViz |
| `/autopilot/imu`, `/autopilot/imu1` | sensor_msgs/Imu | raw IMU0 / IMU1, orientation = firmware estimate |
| `/autopilot/gps/fix`, `/autopilot/gps/vel` | NavSatFix, TwistStamped | |
| `/autopilot/baro`, `/autopilot/baro/altitude` | FluidPressure, Float64 | |
| `/autopilot/mag` | sensor_msgs/MagneticField | tesla, FLU |
| `/autopilot/battery` | sensor_msgs/BatteryState | |
| `/autopilot/pwm` | std_msgs/UInt16MultiArray | ESC/servo outputs, µs |
| `/autopilot/state` | std_msgs/String (JSON) | armed, mode, failsafe, faults, active sensors |
| `/autopilot/telemetry` | std_msgs/String (JSON) | the full raw record |
| `/autopilot/vehicle_marker` | visualization_msgs/Marker | blue = OK, orange = fault injected, red = failsafe |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | sensor health, failsafe |
| `/clock` | rosgraph_msgs/Clock | simulated time |
| TF | | `map → base_link` (truth), `map → base_link_est` (estimate) |

Inputs:

```bash
ros2 topic pub --once /autopilot/cmd std_msgs/msg/String "{data: 'fault gps dropout'}"
ros2 service call /autopilot/arm std_srvs/srv/SetBool "{data: true}"
# /autopilot/sticks (sensor_msgs/Joy): axes = roll, pitch, yaw, throttle[, forward, lateral]
```

QGroundControl works at the same time: run it on Windows and it shows the
vehicle, parameters and failsafe messages. WSL2 forwards `127.0.0.1` to Windows
only with *mirrored networking* (Windows 11: add `networkingMode=mirrored` under
`[wsl2]` in `%UserProfile%\.wslconfig`, then `wsl --shutdown`). Otherwise pass
the Windows IP: `gcs:=$(ip route | awk '/default/ {print $3}'):14550`.

### Bridge commands

The same commands work from ROS (`/autopilot/cmd`), from scenario files and from
plain UDP (`nc -u 127.0.0.1 9100`). Prefix any command with `at <t>` to run it at
simulated time t.

| Command | Effect |
|---|---|
| `start` | start simulated time (the SITL waits for this, so a script can queue its `at` commands first) |
| `arm`, `disarm`, `mode <0-2 or name>` | as from a ground station |
| `vehicle quad\|rover\|boat\|sub`, `frame <n>` | switch vehicle type or frame (while disarmed) |
| `sticks r p y t [f l]`, `rc off\|on\|release` | pilot input; `rc off` simulates losing the radio |
| `fault imu0\|imu1\|imu\|baro\|mag\|gps none\|dropout\|stuck\|nan\|gyro_offset` | sensor faults |
| `gps_offset n e d`, `gps_vel_offset n e d` | GPS error or spoofing (m, m/s) |
| `mag_interference x y z`, `baro_offset m` | compass disturbance (gauss), baro drift |
| `gyro_bias imu x y z`, `accel_noise imu sd` | residual gyro bias (rad/s), vibration (m/s²) |
| `gps_noise pos vel`, `baro_noise m` | sensor noise levels |
| `motor <n> <health>` | motor n (1-based): 0 = dead, 0.5 = damaged prop |
| `wind n e d`, `current n e`, `mass kg`, `max_thrust N` | environment and airframe |
| `battery_used mAh` | drain the simulated battery |
| `quit` | stop the SITL |

The `autopilot_sitl` options for this are `--bridge PORT`, `--bridge-rate HZ`,
`--speedup X`, `--seed N`, `--gcs off`, `--physics json[:HOST:PORT]` and
`--json-pwm MIN:MAX`. See `autopilot_sitl --help`.

---

## 5. Gazebo Harmonic (3D physics)

With `--physics json`, `autopilot_sitl` does not simulate the vehicle itself. It
exchanges one packet per physics step with an external simulator using the
**ArduPilot JSON SITL protocol**. Gazebo's `ardupilot_gazebo` plugin speaks that
protocol, so our firmware flies Gazebo's iris quad unchanged. The sensors are
still generated (with faults) from Gazebo's true state, so fault injection and
scenarios work the same way.

**Install, once:**

```bash
sudo apt install -y ros-jazzy-ros-gz rapidjson-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir -p build && cd build
export GZ_VERSION=harmonic && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
cat >> ~/.bashrc <<'EOF'
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
EOF
source ~/.bashrc
```

**Fly:**

```bash
ros2 launch autopilot_sim gazebo.launch.py                     # Gazebo window + firmware + ROS bridge
ros2 launch autopilot_sim gazebo.launch.py gui:=false          # headless (no GPU / slow laptop)
ros2 launch autopilot_sim gazebo.launch.py scenario:=quad_gps_loss
```

To do it by hand: `gz sim -r iris_runway.sdf` in one terminal, then
`build/autopilot_sitl --physics json --mode alt_hold` and QGroundControl, or the
scenario runner:
`ros2 run autopilot_sim run_scenarios quad_hover --sitl-args "--physics json" --jobs 1`.

Results with the Gazebo iris (headless, Gazebo Harmonic 8, ardupilot_gazebo):
`quad_hover`, `quad_imu_failover`, `quad_rc_loss` and `quad_compass_interference`
pass. Two miss limits that were set for the built-in quad: `quad_gps_loss`
(tilt estimate error 7.4° against a 6° limit) and `quad_motor_failure`
(touchdown at 3.9 m/s against 3 m/s; the iris is heavier). Those are
worth a look before raising the limits.

Notes:
- If Gazebo's window is black or crashes under WSL, try
  `export LIBGL_ALWAYS_SOFTWARE=1`, or use `gui:=false` and watch in RViz.
- Restart Gazebo between scenario runs, because the iris stays wherever the
  last run left it.
- `wind`, `mass` and `motor` commands change the built-in physics only. With
  Gazebo, a dead motor is simulated by cutting that output's PWM, which works,
  but wind and mass belong to the Gazebo world.
- Without Gazebo, `python3 -m autopilot_sim.json_physics_quad` is a 200-line
  stand-in that speaks the same protocol. Use it to test the external-physics
  path, or as a template for your own physics.

---

## 6. What the simulator has found so far

Fixed in this commit, each with a regression test in `tests/test_sim_found_bugs.cpp`:

1. **A NaN from one IMU cut all motors in flight**, even with a healthy second
   IMU (`quad_imu_failover`, `kind: nan`; the quad fell from 5 m). The gyro
   consistency check compared the good IMU against the NaN one, marked the good
   one DEGRADED in the same tick, and the failsafe saw "no usable IMU" and
   terminated. Non-finite readings are now left out of the comparison.
2. **ALT_HOLD climbed away when the baro died** (+14 m in 20 s). Without an
   altitude estimate, the fallback treated mid stick as 50 % collective. Now
   mid stick keeps the collective that was holding altitude. This is still
   open loop, so randomized runs can drift by about 1 m/s. The real fix is a
   second height source (GPS altitude) in the estimator.
3. **Rover HEADING_HOLD ignored the steering stick.** Steering now passes
   through, and the vehicle holds the new heading when the stick is released.
4. **An IMU that stopped sending could still cut the motors** (found in Gazebo,
   `quad_imu_failover`, `kind: dropout`). For the 50 ms before IMU0 timed out,
   the live IMU1 was compared with IMU0's frozen reading. Gazebo's airframe
   shakes enough that IMU1 was marked DEGRADED, the failover found no usable
   IMU and the failsafe terminated at 4 m. A stale IMU is now left out of the
   comparison.

Still open, shown by the scenarios:
- **GPS spoofing tilts the attitude estimate by 10–15°** (`quad_gps_spoof`,
  marked `known_issue`). GPS velocity is used for tilt aiding without an
  innovation check. This ties in with the GPS spoofing detection work.
- Baro loss: needs a GPS-altitude fallback in the estimator (see 2).
- There is no auto-disarm after a normal landing. Only the failsafe disarms by
  itself.

## 7. Limitations

- The built-in vehicles are simple rigid-body models. They are good for logic,
  failsafes and estimator behaviour, not for tuning gains for a specific frame.
  Gazebo is closer but still idealized.
- The sensors are generated from the true state (noise, bias, latency, faults).
  They are not a model of specific chips. The DroneCAN and I2C drivers are
  tested separately by the C++ unit tests.
- Real time on WSL is approximate. Scenario runs use simulated time, so they
  are exact at any speed.
