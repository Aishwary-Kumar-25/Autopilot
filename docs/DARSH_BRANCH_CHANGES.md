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
- When GPS is available, its acceleration is subtracted from the accelerometer before the accelerometer is used as a gravity reference.
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
| 9° roll step | The estimate tracks 9°. The true angle is about 11.5°, because no GPS is available to correct the estimate (see section 3). |
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

## Still open / next steps

- Parameter storage (gains, battery threshold, board rotation) in flash.
- A MAVLink telemetry link to the GCS. UAP has a tested encoder for HEARTBEAT and ATTITUDE that can be ported.
- An EKF that fuses GPS velocity, so attitude stays unbiased during long manoeuvres.
- A first build on real hardware: the STM32 code compiles only inside a CubeMX project, and nobody has built it there yet.
