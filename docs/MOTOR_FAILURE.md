# Motor failure: detection and control re-allocation

What happens when a multirotor motor dies in flight (dead ESC or motor, lost
propeller), and how to try it in the simulator.

## Outcome by frame

| Frame | After the failure | Who flies |
|---|---|---|
| Hexa / octa / more | All axes re-allocated to the remaining motors. Roll, pitch and altitude are held as before; yaw authority is reduced (a hexa drifts in yaw at ~0.5 rad/s with the opposite motor at idle). | The pilot. GCS warning: land. |
| Quad | Yaw can't be controlled with 3 motors. The yaw loop is switched off, the vehicle spins (~20 rad/s) and stays upright on roll/pitch/thrust. | Failsafe LAND (reason MOTOR), disarms on touchdown. |
| Any frame without enough thrust or control left | Best-effort allocation. | Failsafe LAND. |

## Detection (`control/motor_failure_detector.*`)

Runs every control step after the mixer, on any frame the matrix mixer
describes, with no airframe model.

1. **Lost thrust (onset).** Motor i's healthy (roll, pitch) mixer coefficients
   ŝ_i say which way it tips the vehicle. From all motor commands, delayed by the
   motor's spin-up lag (`motor_tau_s`, 40 ms), the detector knows which way the
   motors *should* be turning the body. If they push along ŝ_i (by at least
   `onset_push`) while the body angular-accelerates against it
   (`alpha_min`, 5 rad/s²) for `onset_time_s` (30 ms), motor i is dead. The
   angular acceleration is the 25 Hz low-passed derivative of the gyro.
2. **Lost thrust (sustained).** A motor pinned at ≥ 0.95 while the others
   average 0.35 lower, for 0.4 s, in steady flight (body rate < 0.5 rad/s).
3. **ESC telemetry.** If the ESCs report RPM (`UAVVehicle::report_esc_rpm()`):
   commanded above 0.25 but below 1000 RPM for 50 ms.

Detection only runs while flying, and stops once yaw has been given up (the
spinning vehicle no longer matches its assumptions).

## Re-allocation (`ActuatorMatrixMixer::set_motor_failed()`)

With R the healthy rows (roll, pitch, yaw, collective per motor), G = RᵀR, and E
the healthy rows of the motors that are left, the new rows are

    rows' = Eᵀ (E Eᵀ)⁻¹ G

so the attitude and altitude loops see the same torque and thrust per unit
demand as before (no re-tuning). Roll, pitch and yaw come from the 4-axis solve;
the collective column from the roll/pitch/collective-only solve, so hover never
relies on yaw. If E Eᵀ is singular (quad on 3 motors) yaw is dropped (`NO_YAW`).
`collective_limit()` is the highest collective the remaining motors can deliver
(0.6 on a hexa on 5); below 1.15x hover the vehicle lands.

The desaturation handles the uneven collective column: roll/pitch first, then
yaw, then the collective as close to the demand as the motors allow; yaw may move
the collective by at most 0.1.

## Ground station

STATUSTEXT, e.g. `Motor 3 failed (lost thrust)` then `Flying on remaining
motors, yaw degraded: land` (hexa) or `No yaw control left: spinning, landing
now` and `Failsafe LAND: MOTOR` (quad). The dead motor's PWM shows 1000 us in
SERVO_OUTPUT_RAW. Parameter `MOT_FAIL_DETECT` = 0 turns detection off.

## Try it

    ./build/autopilot_sitl --frame 6 --fail-motor 3@20   # hexa, motor 3 dies 20 s after arming
    ./build/autopilot_sitl --frame 4 --fail-motor 2@20   # quad
    ./build/autopilot_sitl --frame 6 --fail-motor 3@20 --esc-telemetry

Connect QGroundControl, switch to ALT_HOLD, arm, climb and hover; watch the
messages and the motor outputs when the motor dies. Automated version:

    python3 tools/sitl_motor_failure_check.py --frame 6   # or 4, 8

## Limits

- The quad case is a controlled descent, not a hover: two motors on one
  diagonal lift about 1.0x the weight. A spinning-quad controller ("relaxed
  hover", Mueller & D'Andrea 2014) could hold altitude.
- Flat rotor plane assumed; ROV thrusters are not handled.
- A lost propeller makes the ESC report higher RPM, so only the lost-thrust rule
  sees it.
- Thresholds were tuned on the simulated quad/hexa/octa; check against real logs.
