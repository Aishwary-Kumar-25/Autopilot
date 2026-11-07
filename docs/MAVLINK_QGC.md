# MAVLink v2 and QGroundControl

The autopilot speaks MAVLink 2 to any ground station (QGroundControl, Mission
Planner, MAVProxy, pymavlink). The protocol code is written from the spec with
no generated headers and no heap use in the codec, and every encoder is checked
byte for byte against pymavlink.

## Fly the SITL quad from QGroundControl

```bash
cmake -S . -B build && cmake --build build -j
./build/autopilot_sitl                    # sends to 127.0.0.1:14550
```

1. Start QGroundControl. It listens on UDP 14550 by default, so the vehicle
   shows up on its own as a generic quadrotor at IIT Kanpur.
2. Turn on a joystick: either a USB gamepad, or **Application Settings >
   General > Virtual Joystick**. The sticks arrive as `MANUAL_CONTROL`, which the
   autopilot treats as RC input, so RC-loss failsafe works when the sticks stop.
3. Choose a flight mode. QGC's mode menu (Manual or Stabilized) works, and
   `FLTMODE` in the parameter editor covers all three modes (0 MANUAL,
   1 STABILIZE, 2 ALT_HOLD). `--mode alt_hold` on the command line starts in
   ALT_HOLD.
4. Arm from QGC with throttle at zero, then fly.

Options: `--gcs HOST:PORT` (QGC on another PC or in WSL/VM),
`--rc-neutral` (feeds centred sticks so you can arm without a joystick),
`--sysid N`, `--duration S`, `--quiet`.

If no RC input has arrived, arming is refused with `PreArm: no RC input`, which
QGC shows as a message. The real vehicle behaves the same way.

## What the vehicle sends

| Message | Rate | Content |
|---|---|---|
| HEARTBEAT | 1 Hz | MAV_TYPE from the active profile (quad/hexa/octa/tri, rover, sub), armed flag, custom_mode, state (STANDBY/ACTIVE/CRITICAL in failsafe/EMERGENCY on terminate) |
| SYS_STATUS | 1 Hz | sensor present/health bits from the sensor manager (gyro, accel, mag, baro, GPS, AHRS, RC, pre-arm), battery V/A/%, link drop rate |
| ATTITUDE | 10 Hz | estimator roll/pitch/yaw and body rates |
| VFR_HUD | 4 Hz | ground speed, altitude, climb, heading, throttle % |
| GLOBAL_POSITION_INT | 5 Hz | only while a GPS has a fix, so the map never shows 0,0 |
| GPS_RAW_INT | 2 Hz | fix type, satellites, raw position |
| SERVO_OUTPUT_RAW | 2 Hz | the PWM output pins (µs) |
| STATUSTEXT | on event | armed/disarmed, failsafe and its cause, sensor health changes, mode/profile changes, refused commands |
| AUTOPILOT_VERSION | on request / GCS connect | capabilities: MAVLINK2, PARAM_FLOAT, PARAM_ENCODE_BYTEWISE, COMMAND_INT, MISSION_INT |

`MAV_CMD_SET_MESSAGE_INTERVAL` changes any of the streamed rates.

## What the vehicle accepts

| Message / command | Behaviour |
|---|---|
| HEARTBEAT (GCS) | marks the GCS connected (3 s timeout) and sends AUTOPILOT_VERSION |
| PARAM_REQUEST_LIST / _READ / PARAM_SET | full parameter protocol. INT32 values use byte-wise encoding (the PX4 and generic convention QGC expects). A refused or out-of-range set answers with the unchanged value plus a STATUSTEXT |
| COMMAND_LONG / COMMAND_INT | `COMPONENT_ARM_DISARM` (400), `DO_SET_MODE` (176), `SET_MESSAGE_INTERVAL` (511), `REQUEST_MESSAGE` (512), `REQUEST_AUTOPILOT_CAPABILITIES` (520). Anything else gets an ACK with `UNSUPPORTED` |
| SET_MODE | custom mode, or base-mode flags for GCSs that only know those |
| MANUAL_CONTROL | joystick to pilot input, mapped per vehicle (UAV: pitch/roll/thrust/yaw; rover: throttle/steer; sub: surge/sway/heave/yaw) |
| MISSION_REQUEST_LIST | answers "0 items", so QGC's plan sync finishes cleanly |
| TIMESYNC, PING | answered |

Safety rules:

- Disarming a multicopter that isn't landed is refused unless the GCS forces
  it (param2 = 21196, which is QGC's "force disarm").
- Changing the vehicle profile or frame (`VEHICLE_TYPE`, `UAV_ROTORS`,
  `ROV_THRUSTERS`) is refused while armed.
- Commands addressed to another system id are ignored.

## Parameters

| Name | Type | Meaning |
|---|---|---|
| VEHICLE_TYPE | int | 0 UAV, 1 UGV, 2 ROV. Switches the runtime profile (disarmed only) |
| UAV_ROTORS / ROV_THRUSTERS | int | frame size (disarmed only) |
| FLTMODE | int | flight mode 0..2 (names per vehicle, see `comms/vehicle_params.hpp`) |
| FS_RC_TIMEOUT | int ms | RC-loss failsafe delay |
| FS_BATT_LOW_V | float V | battery failsafe threshold (0 = off) |
| MC_HOVER_THR | float | initial hover throttle (it keeps learning in flight) |
| MC_ROLL_P, MC_PITCH_P | float | angle loop P |
| MC_ROLLRATE_P/I/D, MC_PITCHRATE_P/I/D, MC_YAWRATE_P/I | float | rate loops |
| MC_ALT_P, MC_CLIMB_P/I/D | float | altitude hold |

Tuning survives a vehicle-profile switch, because stored values are re-applied
to the new vehicle object. Parameters are **not saved across reboots yet**;
that needs a flash/EEPROM backend on the board.

## Code map

```
comms/mavlink/mavlink_protocol.*   framing, CRC, v1+v2 parser, LE writer/reader
comms/mavlink/mavlink_messages.hpp message structs (wire order), enums
comms/params.*                     ParamStore: typed named params with setters
comms/vehicle_params.*             the flight-stack parameter set + flight modes
comms/gcs_link.*                   GcsLink: streams, commands, params, events
hal/posix/udp_link.*               UDP transport (a hal::UARTDevice)
sim/sitl_main.cpp                  real-time SITL app (quad + GPS + battery sim)
tests/test_mavlink.cpp             golden frames + link behaviour (ctest)
tools/gen_mavlink_golden.py        regenerates tests/mavlink_golden.hpp
tools/sitl_gcs_check.py            end-to-end: pymavlink as GCS flies the SITL
```

On hardware, give `GcsLink` the telemetry UART (`hal->get_uart(n)`, USART1 in
`docs/HARDWARE_STM32_HAL.md`) and call `update()` from the main loop at
50–400 Hz. It sends one frame per `write()`, and reception is bounded per call.

## Testing

```bash
ctest --test-dir build                 # includes test_mavlink (108 checks)
pip install pymavlink
python3 tools/sitl_gcs_check.py        # pymavlink as GCS: params, arm, climb, hold, land, disarm
python3 tools/gen_mavlink_golden.py > tests/mavlink_golden.hpp   # after changing an encoder
```

## Not done yet

- MAVLink signing. Signed frames are parsed but **not verified**, and that
  verification is the next security step (Arv's item).
- Missions and guided/auto modes, which need a navigation controller first.
- Parameter persistence.
- PARAM_ERROR replies.
- Sending COMMAND_ACK progress for long-running commands.
- Calibration commands (`PREFLIGHT_CALIBRATION`), which currently get an ACK
  with `UNSUPPORTED`.
