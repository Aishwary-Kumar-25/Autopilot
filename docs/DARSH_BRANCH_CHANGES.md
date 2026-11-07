# What changed on the `darsh` branch

This branch starts from `AK` (commit d869699). It fixes the bugs found while reviewing that branch and adds the safety and test pieces that were missing. Everything builds with CMake. `ctest` runs 7 test programs and all of them pass.

```
mkdir build && cd build && cmake .. && make -j && ctest --output-on-failure
```

## 1. DroneCAN (commit 1)

| Problem on `AK` | Fix |
|---|---|
| Wrong message IDs. RawIMU used 1000 (that is ahrs.Solution), and StaticPressure/Temperature used 1020/1021 (1020 is TrueAirspeed). | Corrected all the IDs. Data type signatures are computed by `tools/dsdl_signature.py` and checked against ArduPilot's generated headers. |
| No multi-frame transfers, so RawIMU (47 B) and Fix2 (50 B) could never be received. | `dronecan_codec`: libcanard-compatible bit packing, a CRC-16 seeded with the signature, and reassembly that checks toggle and transfer-id. |
| Payload layouts were guessed. | Real encoders and decoders for NodeStatus, RawIMU, MagneticFieldStrength(2), StaticPressure/Temperature, Fix2 and ESC RawCommand. |
| ESC RawCommand was packed in the wrong bit order, so ESCs would read garbage. | Uses the DroneCAN bit order (tested against libcanard golden vectors). |
| A CAN node that was unplugged stayed "healthy" forever. | Per-sensor timeouts in the SensorManager. |
| CAN sensors were added to `_all_sensors` again on every loop, which leaked memory. | Registration is de-duplicated. |

## 2. Sensors and health monitoring (commit 1)

- Frame convention: body frame is FRD and earth frame is NED. The accelerometer reports specific force, so a level vehicle reads (0, 0, -g). I2C breakout chips get a board rotation. This fixed vertical acceleration, which was inverted.
- SensorManager checks every sensor for timeout, stuck readings, out-of-range values and gyro consistency (nearest-neighbour vote between IMUs).
  - Failover works for IMU, baro, compass and GPS.
  - A sensor that fails while armed stays failed for the rest of the flight.
  - A different chip model can be hot-swapped onto the same I2C address.
- NMEA sentences are checksum-checked, and the GPS now provides NED velocity.
- Tests use `CHECK` instead of `assert`, so they still run in Release builds.

## 3. Estimation (commits 1 and 2)

- Madgwick is the primary attitude estimator. A complementary filter runs alongside it as a dissimilar backup, and a selector chooses between them.
- GPS-aided attitude (commit 3, details in section 10). With GPS velocity, the primary estimator stops using the accelerometer as a gravity reference. It compares the change in GPS velocity with the integrated accelerometer over the same window instead. The backup filter still subtracts the GPS-derived acceleration from the accelerometer.
- Madgwick's gain is high during alignment and drops to 0.03 in flight. The backup filter has a 4 s time constant. Multirotor accelerometers barely see sustained tilt, so higher gains dragged the estimate toward level. The SITL test measured this.

## 4. Architecture (commit 2)

- There is now **one** SensorManager and SensorFusion, owned by `VehicleManager` and shared with the active vehicle through `VehicleBase::attach_shared_stack()`. Before this change, each vehicle built a second stack, and the two drained the same CAN RX queue between them. A vehicle created on its own (as in the unit tests) still owns its stack.
- `VehicleManager::arm` delegates to the vehicle, so there is one prearm check and one armed flag.
- `VehicleManager::step` publishes the demands that were actually sent to the mixer. Before, it always published zeros.
- When the vehicle profile is switched, the RC link state is carried over to the new vehicle.

## 5. Control (commit 2)

- PID changes:
  - The derivative is taken on the measurement, so a setpoint step causes no kick.
  - Anti-windup is conditional: the integrator does not grow while the output is saturated.
  - `imax = 0` now disables the I term. Before, it meant the integrator was unlimited.
  - A new `integrate=false` argument freezes the integrator.
- UAV controller changes:
  - The PIDs reset on arm and disarm, so integrators no longer carry over from a previous flight.
  - The integrators freeze on the ground.
  - Zero throttle gives zero torque demands. Before, the mixer spun motors up on the ground.
  - Tilt compensation keeps vertical thrust constant when the vehicle is tilted.
- ALT_HOLD changes:
  - The stick now commands climb rate, with a ±10% deadband. Before, the stick was ignored and the hover throttle was fixed at 0.5.
  - The hover throttle is learned in flight.
  - The vehicle takes off only on a deliberate climb stick.
  - A land detector was added.
- Arming now requires a live RC link.

## 6. Failsafe (new, `control/failsafe.*`)

| Trigger | Action |
|---|---|
| No RC input for 500 ms | LAND. Control returns to the pilot if RC comes back. |
| Battery below threshold for 2 s (`set_battery_voltage`) | LAND, latched. |
| No valid attitude estimate | LAND, latched. |
| No usable IMU | TERMINATE (disarm). |

The most severe active trigger decides the action. On LAND, a UAV levels, descends at 0.6 m/s and disarms once the land detector confirms it is on the ground. A UGV or ROV goes to neutral.

## 7. Software-in-the-loop flight test (new, `sim/quad_sim.hpp`, `tests/test_sitl_flight.cpp`)

The test flies a 6-DOF model of a 1.2 kg quad (motor lag, drag, ground contact). The real flight code runs it through `VehicleManager`: shared stack, controller, failsafe, mixer and PWM. The simulated IMUs, baro and compass support fault injection (stuck, dropout, gyro offset, NaN).

| Phase | Result |
|---|---|
| Arm at mid stick in ALT_HOLD | Stays on the ground. |
| Climb on stick, then hold | Holds 2.4 m within 0.05 m. Learned hover throttle is 0.493 (theory: 0.49). |
| 9° roll step | The estimate tracks 9°. The true angle is about 11.5°, because this test flies without GPS (see section 10 for the GPS-aided version). |
| IMU0 freezes in flight | Fails over to IMU1 and keeps flying level. |
| RC lost | Failsafe LAND, touchdown at 0.56 m/s, disarmed 7 s later. |

## 8. STM32 HAL (commit 2)

- On target, when CubeMX defines `USE_HAL_DRIVER`, the backend calls the ST HAL:
  - I2C, SPI and UART
  - bxCAN or FDCAN
  - PWM timers set to a 1 MHz tick
  - DWT microsecond timer
- CAN and UART receive into interrupt-fed ring buffers. Call `on_rx_isr` / `on_rx_byte_isr` from the HAL callbacks.
- Off target, every bus call returns failure. Before, the stubs returned `true`, which made the sensor manager believe hardware existed.
- Fixed an SPI register read that over-read a 1-byte buffer and returned data shifted by one byte.

## 9. Pub/sub

- The publish counter is now 32-bit. A 64-bit atomic is not lock-free on Cortex-M.
- Reads are tear-free: if the publisher lapped the reader during a copy, the read is retried.

## 10. GPS-aided attitude (commit 3)

**The problem.** A multirotor's accelerometer measures thrust plus drag, not gravity. While the quad holds a bank and keeps accelerating sideways, the reading stays along body Z and looks level. Madgwick then drifts toward level, the controller tilts further to hold the commanded angle, and the true bank grows. In SITL, a 15° bank stick held for 4 s ended at **26° true bank with an 11.7° estimate error**.

**Why the old compensation didn't help.**
- The previous code already subtracted GPS acceleration, but it never ran in SITL. A GPS with no fix stayed the active GPS forever, so a working GPS was never selected. The posix HAL auto-registers an NMEA GPS on the UART first, which triggered this. The sensor manager now marks a GPS without a 3D fix as DEGRADED, so a GPS with a fix takes over.
- Once GPS reached it, the old method cut the error to 2°, but its GPS acceleration lagged by ~250 ms because differentiating velocity adds lag on top of receiver latency. In a turn the acceleration vector rotates, so that lag points the correction the wrong way.

**The new method** (`fusion/ahrs_interface.cpp`, similar to ArduPilot DCM):
1. Integrate the gyro (Madgwick with no accelerometer feedback).
2. Every GPS sample, compare the change in GPS velocity over the last ~0.4 s with the integral of earth-frame specific force over the **same** window, shifted by the GPS latency (`set_gps_delay_ms`, default 100 ms). Rotate the estimate so the two agree, with a 1 s time constant. Specific force is kept in 10 ms buckets (1.6 s of history). Each correction rotates the stored buckets too, so overlapping windows don't count the same error twice.
3. Correct heading only from the tilt-compensated magnetometer, so magnetic disturbances can't tilt the estimate.
4. If GPS stops, coast on the gyro for 3 s, then fall back to plain Madgwick. This also covers 1 Hz receivers.

**Results** (`tests/test_sitl_gps_attitude.cpp`, worst estimate error during the manoeuvre):

| Manoeuvre | No GPS | GPS-aided |
|---|---|---|
| 15° bank held 4 s | 11.7° (truth 26.4°) | **0.46°** (truth 15.1°) |
| 12° pitch + 86°/s yaw, 6 s | 4.5° | **0.57°** |
| Same turn, real latency 0 / 200 ms (fusion assumes 100) | | 1.2° / 1.3° |
| Same turn, 5 Hz / 1 Hz GPS | | 0.42° / 1.45° |
| Bank with GPS lost after 2 s | | 0.38° at end of bank |

Other changes:
- The SITL rig moved into `tests/sitl_rig.hpp`.
- `sim::SimGPS` was added: NED velocity, configurable rate and latency, and dropout.
- Horizontal velocity is dead-reckoned for only 2 s after the last GPS sample. Before, it integrated accelerometer error forever on the bench.

## Still open / next steps

- Parameter storage (gains, battery threshold, board rotation) in flash.
- ~~A MAVLink telemetry link to the GCS.~~ Done in commit 5 (see below and docs/MAVLINK_QGC.md).
- An EKF that fuses GPS velocity and position and estimates gyro bias. The GPS tilt correction in section 10 has no integral term, so a gyro bias of b leaves a tilt error of about b / 1 s (0.6° for 0.01 rad/s).
- Without GPS, the accelerometer still reads drag as tilt, so the estimate is wrong while a fast-moving quad slows down. A rotor-drag model would fix this.
- A first build on real hardware: the STM32 code compiles only inside a CubeMX project, and nobody has built it there yet.

---

# Review round 2: bugs, STM32 optimizations, new features (commit 4)

Everything below was found and fixed without hardware. `ctest` now runs 12 test
programs (4 new here), all passing, also under AddressSanitizer + UBSan and with clang.

## Bugs fixed

| Area | Problem | Fix | Test |
|---|---|---|---|
| Mixer | When a demand saturated motors on both ends, the shift used the pre-shift minimum, so outputs were clipped and roll and yaw were distorted together. | Prioritised desaturation: roll/pitch first, then as much yaw as fits (bisection), then collective shifted (air-mode). | `test_mixer_saturation` (incl. 5000 random demands: always in range, roll/pitch never inverted) |
| Mixer | A NaN/Inf demand (diverged estimator) went straight to the PWM cast (undefined behaviour). | Non-finite demands become neutral. | same |
| DroneCAN ESC | Only the first 4 ESCs were ever commanded; a hexa/octa on CAN ESCs would lose motors 5+. | `send_dronecan_esc_commands` sends all channels as a multi-frame RawCommand. | same (8-motor transfer reassembled and decoded) |
| Arming | Arming with the throttle up spun the motors straight up (STABILIZE/MANUAL). Same for a rover/sub with drive sticks deflected. | `check_arming_inputs()` per vehicle: UAV throttle low (ALT_HOLD: not above mid stick), UGV/ROV sticks neutral. | `test_vehicle_safety` |
| Profile switch | `switch_vehicle()` while armed silently **disarmed** the vehicle: one bad command dropped a flying drone. `configure_uav_rotors()` / `configure_rov_thrusters()` changed the motor layout in flight. | All three are refused while armed. | `test_vehicle_controls` 3G |
| Controllers | Controllers ran while disarmed, so UGV/ROV integrators wound up on the bench and kicked on arming. | No controller step while disarmed; UGV/ROV PIDs reset on arm. | `test_vehicle_safety` |
| ALT_HOLD | Switching STABILIZE -> ALT_HOLD in flight held the altitude captured at **arming** (usually 0 m): the drone dived at 2 m/s toward the ground. | `set_mode()` captures the current altitude and seeds hover throttle from the manual collective. | `test_vehicle_safety` |
| Altitude | Baro failover stepped the altitude by the offset between the two baros (8.3 m in the test), making ALT_HOLD jump. | Re-reference the new baro to the current estimate. | `test_vehicle_safety` (old code: 8.35 m jump, new: 0) |
| UGV | Heading hold differentiated the wrapped heading error: a derivative kick on every new target and a spike at the +-180 deg wrap. Speed hold used unsigned speed, so reversing never worked. | D term from the gyro yaw rate; signed speed along the heading; heading captured on mode entry. | `test_vehicle_safety` |
| ROV | DEPTH_HOLD ignored the heave stick and held a stale target. | Stick commands sink/climb rate and re-captures depth; target captured on mode entry and arm. | |
| QMC5883L | Configured for the 8 G range but scaled as 2 G: every reading 4x too small. Status register (DRDY/overflow) never read. | 8 G scale; read status first, skip the data read when no new sample, reject overflow. | `test_drivers` |
| BMP280 | Blank calibration NVM silently replaced by the datasheet's example coefficients (plausible but wrong pressure). First altitude sample was taken against 101325 Pa, so the filter started tens of metres off. Signed left shifts in the Bosch formula are undefined behaviour in C++17 (UBSan caught it). | Fail init on blank NVM; restart the filter after setting the ground reference; multiply instead of shift. | `test_drivers` (incl. Bosch datasheet worked example: 25.08 C, 100653.27 Pa) |
| Gyro cal | Bias calibration accepted samples taken while the vehicle moved, baking the motion into the bias (attitude drifts all flight). | `GyroCalAccumulator`: mean/variance check, retries, init fails until still (re-probed every second while disarmed). | `test_drivers` |
| GPS | RMC status V didn't clear the fix. GGA couldn't tell 2D from 3D. | V clears the fix; GSA parsed for fix type and HDOP (a GPS without a 3D fix is DEGRADED, see section 10). | `test_drivers` |
| Scheduler | Re-arming each task from "now" made every task run slow (400 Hz task ran ~370 Hz); with simulated time starting at 0 a task ran on every call. | Phase-locked deadlines with overrun counter and max run time. | `test_drivers` |

## STM32 optimizations

- **No heap allocation in the flight loop:** `SensorManager::update_consistency()` built a `std::vector` every loop (400 Hz); now a fixed array, and one `sqrt` per IMU instead of one per pair.
- **NMEA parser rewritten** with a fixed buffer and in-place tokenizer. The old one allocated a `std::vector<std::string>` and a `std::stringstream` for every sentence, and pulled iostreams into the firmware image (tens of KB of flash on newlib).
- **Less I2C bus time:** the BMP280 is read at most every 20 ms (the chip makes ~26 samples/s; the loop read it 400 times/s), and the QMC5883L data is read only when DRDY is set. This also makes the baro's 50 Hz altitude filter actually run at 50 Hz.

## New features (hardware-free)

- `drivers/mag/compass_calibrator.*`: hard + soft iron (axis-aligned ellipsoid) least-squares fit with coverage, residual and plausibility checks. 64 bytes of state, no sample buffer, so it can run on the MCU. `CompassSensor::set_calibration()` applies it in the QMC5883L and DroneCAN compass drivers.
- `math/biquad_filter.hpp`: 2-pole Butterworth low-pass and notch filters. `UAVVehicle::set_gyro_filters()` adds them to the rate loop (off by default, so existing tuning is unchanged).
- `tests/fake_sensors.hpp`: scriptable IMU/baro for driving the real stack in unit tests.

## Still open

- MPU6050 and ICM20689 drivers are near-identical; a shared InvenSense base class would save flash.
- Compass calibration still needs a trigger (GCS command or stick gesture) and parameter storage for the result.

---

# Commit 5: MAVLink 2 link and QGroundControl SITL

The vehicle now talks MAVLink 2 to QGroundControl. Run `./build/autopilot_sitl`, open QGC, and the simulated quad connects on UDP 14550. Full details are in `docs/MAVLINK_QGC.md`.

What was added:
- A hand-written v2 codec, with no generated headers and no heap use. All 13 encoders match pymavlink byte for byte (`tests/mavlink_golden.hpp`). The parser accepts both v1 and v2.
- `GcsLink` streams heartbeat, system status, attitude, HUD, GPS/position and PWM outputs. It sends STATUSTEXT for arm/disarm, failsafe and its cause, sensor health and mode changes. It also handles the parameter protocol, arm/disarm, mode changes, message intervals and requests, and joystick input via MANUAL_CONTROL, which counts as RC input.
- 21 parameters can be tuned from the GCS: vehicle profile, frame, flight mode, failsafe settings and the multicopter PID gains. Tuning survives a profile switch.
- Safety rules: disarming in the air needs the force flag, and profile or frame changes are refused while armed.
- Tests: `test_mavlink` has 108 checks. `tools/sitl_gcs_check.py` uses pymavlink as the GCS and flies the SITL through params, arm, climb, ALT_HOLD, land and disarm.

Not done yet: verifying MAVLink signing (signed frames are parsed but not verified), missions and guided modes, and saving parameters across reboots.

---

# Commit 6: Saved settings and MAVLink 2 signing

Two things that need no board: tuning that survives a reboot, and a link that
only obeys a GCS holding the team's key (Arv's item). Details are in
`docs/MAVLINK_QGC.md`.

Saved settings:
- `hal::Storage` is a new HAL interface. SITL saves to a file
  (`autopilot_sitl.eeprom`). STM32 uses two flash sectors as alternate copies,
  with the header written last, so a power cut mid-save keeps the old copy.
  The test cuts the power at all 73 steps of a save and never gets a corrupt result.
- `comms::Settings` loads saved parameters at boot, through the same range
  checks as a GCS change. It saves a second after the last change and only
  while disarmed, because an STM32 flash erase stalls the CPU for 1-2 s. Only
  changed values are stored, by name, so firmware updates keep the tuning.
- `VEHICLE_TYPE` is saved too, so the board boots as the airframe it was set to.
- `MAV_CMD_PREFLIGHT_STORAGE` reloads, saves now, or resets to defaults
  (QGC's "Reset all to firmware's defaults").

MAVLink 2 signing:
- SHA-256 and the MAVLink 2 signature are written from the spec, and signed
  frames match pymavlink byte for byte.
- Once a key is set, unsigned, wrong-key, tampered and replayed frames are
  dropped before they are handled, and every frame the vehicle sends is signed.
- A key can only be installed while disarmed, and only over a trusted link
  (USB or SITL) or by a request signed with the current key. Someone in radio
  range therefore can't install a key or switch signing off. The key and the
  clock are saved.
- SITL: `--signing-passphrase P` uses the same key as MAVProxy's
  `signing setup P`.

Tests: `test_signing` (55 checks) and `test_storage` (81 checks) are new, and
all 15 ctests pass. `tools/sitl_signing_check.py` runs the real SITL with
pymavlink as the GCS. It covers params surviving a restart, an unsigned GCS
being ignored, a signed GCS working, a replay attack being ignored, the key
surviving a restart, and turning signing off. All checks pass.

Still open: GPS time into the signing clock, and running the flash driver on a
real STM32 (it compiles against the F4 and H7 HAL API, but has not run on hardware).
