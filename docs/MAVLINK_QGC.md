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

## Drive a rover, a boat or a sub with the same firmware

The same `autopilot_sitl` binary simulates every vehicle type, and you can
switch between them while it runs, without restarting anything:

```bash
./build/autopilot_sitl --vehicle rover    # or boat, sub, quad (default)
./build/autopilot_sitl --vehicle boat --current 0,0.3   # 0.3 m/s current toward the east
```

To switch from QGC: disarm, open **Vehicle Setup > Parameters**, set
`VEHICLE_TYPE` (0 quad, 1 rover, 2 sub/ROV, 3 boat) and save. The firmware swaps
its controller and mixer, the heartbeat's MAV_TYPE changes (so QGC shows the new
vehicle icon), and the simulator replaces the physics body at the same spot and
heading. The sensors, the sensor manager and the estimator are the same objects
before and after: only the vehicle-specific part changes. The change is refused
while armed. `ROV_THRUSTERS` (3, 4, 6 or 8) and `UAV_ROTORS` change the frame the
same way.

| VEHICLE_TYPE | Physics (`sim/`) | Outputs | Modes (FLTMODE 0 / 1 / 2) |
|---|---|---|---|
| 0 quad | `QuadModel`: 6-DOF, 4 motors | ch1-4 ESCs | MANUAL / STABILIZE / ALT_HOLD |
| 1 rover | `RoverModel`: Ackermann, tyre grip limit | ch1 steering servo, ch2 ESC | MANUAL / HEADING_HOLD / SPEED_HOLD |
| 2 sub | `SubModel`: 6-DOF, buoyancy, righting moment, real thruster positions | ch1-6 thrusters (3/4/6/8) | MANUAL / STABILIZE / DEPTH_HOLD |
| 3 boat | `BoatModel`: surge/sway/yaw hull, water current | ch1 port, ch2 starboard thruster | MANUAL / HEADING_HOLD / SPEED_HOLD |

Joystick mapping: on the rover and boat the forward stick is throttle and the
yaw stick steers. On the sub the throttle stick is up/down (centre = hold), the
forward stick is surge, roll stick is sway, and yaw turns. The sub loses GPS when
it dives deeper than 20 cm, as a real one does, so the map position freezes and
QGC shows depth as negative altitude.

The boat runs the rover's controller. That is how ArduPilot Rover handles boats
too: steering and speed control are the same, and only the physics and the
outputs differ.

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
| EKF_DRAG_MCOEF | float 1/s | multicopter rotor-drag coefficient for tilt without GPS (learned in flight, saved after landing) |
| EKF_DRAG_BCOEF | float kg/m^2 | body ballistic coefficient (0 = no quadratic drag) |
| EKF_DRAG_AID, EKF_DRAG_CAL | int 0/1 | rotor-drag tilt aiding / in-flight calibration of EKF_DRAG_MCOEF |
| EKF_GBIAS_LEARN | int 0/1 | online gyro bias estimation |

Tuning survives a vehicle-profile switch, because stored values are re-applied
to the new vehicle object, and it survives a reboot (next section).

## Saved settings

Every parameter except `FLTMODE` is saved and loaded at the next boot,
including `VEHICLE_TYPE`, so the board comes up as the airframe it was set to.

- **Where:** `hal::Storage` (`hal/hal.hpp`). In SITL it is the file
  `autopilot_sitl.eeprom` in the current folder (`--storage FILE`,
  `--storage none`, `--wipe` for factory defaults). On STM32 it is two flash
  sectors used as alternate copies (`hal/flash_storage.*`,
  `hal/stm32/stm32_flash.*`); see "STM32 flash" below.
- **When:** a change is saved once nothing has changed for one second (QGC
  writing 20 values costs one save, not 20), and **only while disarmed**: an
  STM32 sector erase stalls the CPU for 1-2 s, which must never happen in the
  air. Changes made in flight are saved after landing and disarming.
- **What:** only values that differ from the default, stored by name with a
  CRC. New firmware that adds or renames parameters still loads the rest, a
  default changed in new firmware takes effect unless you set that value
  yourself, and a damaged block is ignored (defaults are used).
- **From the GCS:** `MAV_CMD_PREFLIGHT_STORAGE` param1 = 0 reloads the saved
  values, 1 saves now, 2 or 4 resets everything to defaults (QGC's
  *Parameters > Tools > Reset all to firmware's defaults*). Refused while armed.

### STM32 flash

By default sectors 10 and 11 of a 1 MB F405/F407/F427 are used (128 KB each at
`0x080C0000` and `0x080E0000`). **Shrink the FLASH region in the CubeMX linker
script to 768K** so no code is placed there. Other chips: set
`AP_STORAGE_SLOT{0,1}_ADDR`, `AP_STORAGE_SLOT{0,1}_SECTOR`,
`AP_STORAGE_SLOT_SIZE` (and `AP_STORAGE_BANK` on dual-bank H7) with `-D`.
Writes alternate between the two sectors and the header goes in last, so a
power cut during a save leaves the previous copy intact
(`tests/test_storage.cpp` cuts the power at every step of a save). One save is
one sector erase, and flash lasts about 10,000 erases per sector, which is
decades of normal tuning.

## MAVLink 2 signing

With signing on, only someone who has the 32-byte secret key can control the
vehicle. Every frame carries a SHA-256-based signature and a timestamp, and
the vehicle drops any frame that is unsigned, signed with another key,
tampered with, or replayed (a timestamp must be newer than the last one on
that link). The vehicle's own frames are signed too, so the GCS can check them.
This is the standard MAVLink 2 scheme
(https://mavlink.io/en/guide/message_signing.html), checked byte for byte
against pymavlink.

**Turn it on in SITL:**

```bash
./build/autopilot_sitl --signing-passphrase "our team secret"
```

The key is SHA-256 of the passphrase, the same as MAVProxy's
`signing setup <passphrase>`, so MAVProxy, Mission Planner (*Config > Signing*)
and pymavlink connect when given the same passphrase. The key is saved, so later
runs stay signed without the option. `--signing-off` removes it. If your
QGroundControl build has no MAVLink signing setting, it still shows telemetry,
but its commands are ignored, and the vehicle says so with a STATUSTEXT
"Signing on: unsigned MAVLink ignored" every 10 s.

**From a GCS (`SETUP_SIGNING`):** accepted only while disarmed, and only if
either

- the link is trusted (`GcsLink::Config::trusted_link`: a USB cable, or SITL
  on the same PC), or
- the request itself is signed with the current key (key rotation, or
  switching off with an all-zero key).

So nobody in radio range can install a key and lock you out, or switch
signing off. On a real board, set `trusted_link = true` only for the USB
port. If the key is lost, re-key over USB.

**Clock:** the timestamp is saved with the key and moves forward with boot
time. It jumps to the GCS's clock (real UTC time) as soon as a signed frame
arrives, and it is saved again at every disarm, so it never goes backwards
across reboots.

## Code map

```
comms/mavlink/mavlink_protocol.*   framing, CRC, v1+v2 parser, LE writer/reader
comms/mavlink/mavlink_messages.hpp message structs (wire order), enums
comms/params.*                     ParamStore: typed named params with setters
comms/settings.*                   saved parameters (load at boot, debounced save while disarmed)
comms/storage_layout.hpp           where each saved block lives in hal::Storage
comms/mavlink/mavlink_signing.*    MAVLink 2 signing: sign, verify, replay check, key storage
comms/mavlink/sha256.*             SHA-256 (no heap)
hal/flash_storage.*                power-cut-safe A/B flash storage (+ simulated flash for tests)
hal/stm32/stm32_flash.*            STM32 F4/F7/G4/H7 flash erase/program
hal/posix/file_storage.*           SITL settings file
comms/vehicle_params.*             the flight-stack parameter set + flight modes
comms/gcs_link.*                   GcsLink: streams, commands, params, events
hal/posix/udp_link.*               UDP transport (a hal::UARTDevice)
sim/sitl_main.cpp                  real-time SITL app (quad + GPS + battery sim)
tests/test_mavlink.cpp             golden frames + link behaviour (ctest)
tests/test_signing.cpp             signing vs pymavlink, GCS lockout, key rules, replay
tests/test_storage.cpp             file + flash storage, power cuts, params across reboots
tools/gen_mavlink_golden.py        regenerates tests/mavlink_golden.hpp
tools/sitl_gcs_check.py            end-to-end: pymavlink as GCS flies the SITL
tools/sitl_signing_check.py        end-to-end: saved params across restarts, signing, replay
```

On hardware, give `GcsLink` the telemetry UART (`hal->get_uart(n)`, USART1 in
`docs/HARDWARE_STM32_HAL.md`) and call `update()` from the main loop at
50–400 Hz. It sends one frame per `write()`, and reception is bounded per call.

## Testing

```bash
ctest --test-dir build                 # includes test_mavlink (108 checks)
pip install pymavlink
python3 tools/sitl_gcs_check.py        # pymavlink as GCS: params, arm, climb, hold, land, disarm
python3 tools/sitl_signing_check.py    # saved params across restarts, signing on/off, replay attack
python3 tools/sitl_vehicles_check.py   # one SITL process switched rover -> boat -> sub -> quad from the GCS
python3 tools/gen_mavlink_golden.py > tests/mavlink_golden.hpp   # after changing an encoder
```

## Not done yet

- Missions and guided/auto modes, which need a navigation controller first.
- GPS time is not yet fed into the signing clock (`Signing::advance_to()` is
  ready for it); until then the clock comes from storage and the GCS.
- The STM32 flash driver compiles for F4 and H7 against the ST HAL API, but has
  not run on a real board yet.
- PARAM_ERROR replies.
- Sending COMMAND_ACK progress for long-running commands.
- Calibration commands (`PREFLIGHT_CALIBRATION`), which currently get an ACK
  with `UNSUPPORTED`.
