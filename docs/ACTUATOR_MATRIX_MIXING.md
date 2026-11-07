# Generalized Actuator Matrix Mixer & Thruster Allocation

## 1. Overview

In robotics and autonomous vehicles (especially Autonomous Underwater Vehicles, Remotely Operated Vehicles, and Multirotors), physical thruster/motor layouts vary significantly across platforms:
- **Budget / Educational ROVs**: 3 or 4 thrusters (e.g., 2 horizontal differential + 1 vertical center thruster).
- **Standard Inspection ROVs**: 6 vectored thrusters (e.g., BlueROV2 with 4 horizontal vectored 45° thrusters + 2 vertical thrusters).
- **Advanced AUVs / Heavy Work-Class ROVs**: 8 thrusters (e.g., BlueROV2 Heavy, RoboSub competition AUVs) featuring 4 horizontal vectored thrusters + 4 independent vertical thrusters for **fully decoupled 6-DOF motion**.
- **High-Redundancy & Heavy-Lift Systems**: 10, 12, or up to 16 thrusters/rotors.

Traditional flight controllers (such as standard ArduPilot / ArduSub) hardcode fixed motor geometry classes. If a user needs a different number of thrusters or wants to adjust geometry in the field, they are forced to modify source code, recompile, and reflash the flight controller.

The **E-Club Autopilot** introduces the **Generalized Actuator Matrix Mixer (`ActuatorMatrixMixer`)**, a dynamic control allocation engine that supports **2 to 16 thrusters/motors** and provides a clean runtime interface to select the number of thrusters on a single binary without reflashing.

---

## 2. Mathematical Formulation: Control Allocation

Actuator setpoints $\mathbf{u} \in \mathbb{R}^N$ ($N \in [2, 16]$) are computed from generalized control wrench demands $\boldsymbol{\tau} \in \mathbb{R}^6$:

$$\boldsymbol{\tau} = \begin{bmatrix} \tau_{\text{roll}} \\ \tau_{\text{pitch}} \\ \tau_{\text{yaw}} \\ F_{\text{heave}} \\ F_{\text{surge}} \\ F_{\text{sway}} \end{bmatrix}$$

Each actuator channel $i$ has an allocation row $\mathbf{B}_i$:
$$u_i = \mathbf{B}_{i, \text{roll}} \tau_{\text{roll}} + \mathbf{B}_{i, \text{pitch}} \tau_{\text{pitch}} + \mathbf{B}_{i, \text{yaw}} \tau_{\text{yaw}} + \mathbf{B}_{i, \text{heave}} F_{\text{heave}} + \mathbf{B}_{i, \text{surge}} F_{\text{surge}} + \mathbf{B}_{i, \text{sway}} F_{\text{sway}}$$

If the motor direction is configured as reversed:
$$u_i \leftarrow -u_i$$

---

## 3. Built-In Thruster Geometry Presets

### A. 3-Thruster Simple ROV (`ROV_3_SIMPLE`)
Common in student projects, low-cost inspection rigs, and SeaPerch-style educational kits:
- **T1 (Horizontal Port)**: Surge: $+1.0$, Yaw: $+0.7$
- **T2 (Horizontal Starboard)**: Surge: $+1.0$, Yaw: $-0.7$
- **T3 (Vertical Center)**: Heave: $+1.0$
- **Channels 3–15**: Inactive (safe disarmed pulse).

### B. 4-Thruster Basic ROV (`ROV_4_BASIC`)
- **T1 (Horizontal Port)**: Surge: $+1.0$, Yaw: $+0.7$
- **T2 (Horizontal Starboard)**: Surge: $+1.0$, Yaw: $-0.7$
- **T3 (Vertical Port)**: Heave: $+1.0$, Roll: $-1.0$
- **T4 (Vertical Starboard)**: Heave: $+1.0$, Roll: $+1.0$
- Provides roll leveling and depth hold in addition to differential steering.

### C. 6-Thruster Vectored ROV (`ROV_6_VECTORED` — ArduSub BlueROV2 Standard)
Four horizontal thrusters oriented at 45° to provide coupled surge, sway, and yaw:
- **T1 (Forward-Right 45°)**: Surge: $+1.0$, Sway: $-1.0$, Yaw: $+1.0$
- **T2 (Forward-Left 45°)**:  Surge: $+1.0$, Sway: $+1.0$, Yaw: $-1.0$
- **T3 (Rear-Right 45°)**:    Surge: $-1.0$, Sway: $-1.0$, Yaw: $+1.0$
- **T4 (Rear-Left 45°)**:     Surge: $-1.0$, Sway: $+1.0$, Yaw: $-1.0$
- **T5 (Vertical-Right)**:   Heave: $+1.0$, Roll: $-1.0$, Pitch: $+1.0$
- **T6 (Vertical-Left)**:    Heave: $+1.0$, Roll: $+1.0$, Pitch: $+1.0$

### D. 8-Thruster Heavy AUV (`ROV_8_HEAVY` — BlueROV2 Heavy & RoboSub AUV)
Solves the pitch-coupling limitation of 6-thruster ROVs by using **four independent vertical thrusters**:
- **Horizontal Thrusters (T1–T4)**: Identical to the 4-thruster vectored perimeter for surge, sway, and yaw.
- **Vertical Thrusters (T5–T8)**:
  - **T5 (Bow-Right Vertical)**:   Heave: $+1.0$, Roll: $-1.0$, Pitch: $+1.0$
  - **T6 (Bow-Left Vertical)**:    Heave: $+1.0$, Roll: $+1.0$, Pitch: $+1.0$
  - **T7 (Stern-Right Vertical)**: Heave: $+1.0$, Roll: $-1.0$, Pitch: $-1.0$
  - **T8 (Stern-Left Vertical)**:  Heave: $+1.0$, Roll: $+1.0$, Pitch: $-1.0$

#### Why 8 Thrusters Enable Decoupled 6-DOF:
1. **Pure Pitch Up**: $T5$ and $T6$ push UP ($>1500\,\mu\text{s}$), $T7$ and $T8$ push DOWN ($<1500\,\mu\text{s}$) $\implies$ Net Heave = 0.
2. **Pure Roll Right**: Port ($T6, T8$) push UP, Starboard ($T5, T7$) push DOWN $\implies$ Net Heave = 0.
3. **Pure Heave (Climb/Sink)**: All four ($T5, T6, T7, T8$) push UP equally $\implies$ Net Pitch = 0, Net Roll = 0.

### E. Custom $N$-Thruster Geometry (`setup_rov_n_symmetric` / `set_actuator_config`)
For 10, 12, or 16 thrusters:
- Automatically distributes horizontal thrusters around perimeter at symmetrical angles:
  $$\alpha_i = i \cdot \frac{2\pi}{N_{\text{horiz}}} + \frac{\pi}{N_{\text{horiz}}}$$
- Allocates 4 or more corner vertical thrusters for pitch/roll/heave authority.
- Custom coefficients can also be set explicitly per channel via `set_custom_thruster()`.

---

## 4. Desaturation & Dynamic Scaling

When pilot stick inputs demand full surge and maximum pitch simultaneously, raw output requests can exceed $[-1.0, 1.0]$. Naive clamping clips actuator signals, distorting the intended motion vector and causing loss of attitude control.

### Bidirectional (ROV) Scaling
Find maximum absolute demand:
$$M = \max_{i} |u_i|$$
If $M > 1.0$:
$$u_i \leftarrow \frac{u_i}{M}$$
This guarantees that the **relative proportions of torques and forces remain intact**, preserving vehicle stability and directional vectoring without saturation clipping.

### Unidirectional (UAV) Scaling
For multirotors, stabilization authority (roll/pitch/yaw) is prioritized over throttle:
If $\max_i u_i > 1.0$:
$$\Delta_{\text{throttle}} = \max_i u_i - 1.0$$
$$u_i \leftarrow u_i - \Delta_{\text{throttle}}$$
Throttle is reduced automatically to maintain roll and pitch control authority, preventing aircraft flip-over.

---

## 5. Runtime Interface: How to Choose the Number of Thrusters

### C++ API Examples

```cpp
#include "vehicles/vehicle_manager.hpp"

// 1. Configure number of thrusters via VehicleManager
vehicles::VehicleManager vm(&hal, vehicles::VehicleType::ROV_SUBMARINE);
vm.init();

// Switch to 8-thruster Heavy AUV layout:
vm.configure_rov_thrusters(8);
assert(vm.get_active_actuator_count() == 8);

// Switch to 3-thruster budget ROV layout:
vm.configure_rov_thrusters(3);
assert(vm.get_active_actuator_count() == 3);

// Switch to 6-thruster standard BlueROV2 layout:
vm.configure_rov_thrusters(6);
assert(vm.get_active_actuator_count() == 6);

// 2. Configure via ROVVehicle directly
auto* rov = dynamic_cast<vehicles::ROVVehicle*>(vm.get_active_vehicle());
rov->set_thruster_count(8, vehicles::ROVLayout::HEAVY_8);

// 3. Define a custom thruster matrix row
// set_custom_thruster(channel, roll, pitch, yaw, heave, surge, sway)
rov->set_custom_thruster(0, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, -1.0f);
```

---

## 6. DroneCAN ESC Output Integration

In addition to driving 16 hardware PWM channels on the STM32 timers, `ActuatorMatrixMixer` natively encodes and broadcasts **DroneCAN ESC setpoints**:

- **Message Type**: `uavcan.equipment.esc.RawCommand` (DSDL ID: 1030 / `0x406`)
- **CAN ID Format**: 29-bit Extended ID (`Priority: 20`, `MsgID: 1030`, `NodeID: 1`)
- **Payload Encoding**: Packed 14-bit signed integers $[-8192, 8191]$:
  $$\text{cmd}_i = \text{clamp}\left(\text{int14}(u_i \times 8191), -8192, 8191\right)$$
- **Hardware Compatibility**: Compatible with Zubax, KDE Direct, and T-Motor DroneCAN ESCs over differential CAN bus wires (`CAN_H` / `CAN_L`).

```cpp
hal::CANFrame frame;
uint8_t transfer_id = 0;
mixer.generate_dronecan_esc_frame(demands, transfer_id, frame, armed);
hal.get_can()->send_frame(frame);
```

---

## 7. Automated Test Coverage

The actuator matrix mixer is fully validated under automated testing:
- **Test File**: `tests/test_vehicle_controls.cpp`
- **Build & Run**:
  ```bash
  cd build
  cmake ..
  make test_vehicle_controls
  ./test_vehicle_controls
  ```
- **Verified Capabilities**:
  - Pure decoupled pitch, roll, and heave on 8-thruster AUVs.
  - Surge and differential yaw on 3-thruster ROVs.
  - Dynamic runtime switching across 3, 6, 8, 12, and 16 channels.
  - Multi-rotor scaling across 4, 6, 8, and 12 rotors.
  - Bit-packed DroneCAN RawCommand frame generation.
