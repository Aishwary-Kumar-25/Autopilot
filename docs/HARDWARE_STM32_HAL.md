# STM32 Hardware Abstraction Layer & Pinout Specification

## Target Microcontroller Class

The autopilot is engineered to run across the standard class of high-performance 32-bit ARM Cortex-M microcontrollers utilized in the Pixhawk and PX4 ecosystems:

| Microcontroller | Architecture | Core Clock | Hardware FPU | Typical Board Reference |
|:---------------:|:------------:|:----------:|:------------:|:-----------------------:|
| **STM32F405 / STM32F427** | ARM Cortex-M4 | 168 - 180 MHz | Single Precision | Pixhawk 1 / PX4 FMUv2 / FMUv3 |
| **STM32F745 / STM32F765** | ARM Cortex-M7 | 216 MHz | Double Precision | Pixhawk 4 / PX4 FMUv5 |
| **STM32H743 / STM32H753** | ARM Cortex-M7 | 400 - 480 MHz | Double Precision | Pixhawk 5X / 6X / PX4 FMUv6X |

---

## Hardware Timers & PWM Pin Availability

The autopilot provides an abstracted `PWMOutput` subsystem supporting 8 hardware output channels. On the STM32, these map directly to general-purpose and advanced-control timers:

| Autopilot Channel | STM32 Timer | Channel | Typical Physical Pin | Typical Assignment |
|:-----------------:|:-----------:|:-------:|:--------------------:|:-------------------|
| **Channel 0** | `TIM1` | `CH1` | `PE9` or `PA8` | Motor 1 (UAV) / Steering Servo (UGV) / Thruster 1 (ROV) |
| **Channel 1** | `TIM1` | `CH2` | `PE11` or `PA9` | Motor 2 (UAV) / Throttle ESC (UGV) / Thruster 2 (ROV) |
| **Channel 2** | `TIM1` | `CH3` | `PE13` or `PA10` | Motor 3 (UAV) / Aux (UGV) / Thruster 3 (ROV) |
| **Channel 3** | `TIM1` | `CH4` | `PE14` or `PA11` | Motor 4 (UAV) / Aux (UGV) / Thruster 4 (ROV) |
| **Channel 4** | `TIM4` | `CH1` | `PD12` | Motor 5 (Hexa) / Thruster 5 Vertical (ROV) |
| **Channel 5** | `TIM4` | `CH2` | `PD13` | Motor 6 (Hexa) / Thruster 6 Vertical (ROV) |
| **Channel 6** | `TIM4` | `CH3` | `PD14` | Auxiliary PWM / Gimbal Tilt / Camera Trigger |
| **Channel 7** | `TIM4` | `CH4` | `PD15` | Auxiliary PWM / Gripper / Payload Release |

### PWM Signal Timing Configuration
- **Standard Servo / Reversible Thruster**: 50 Hz (20ms period), 1000 µs (min), 1500 µs (neutral), 2000 µs (max).
- **High-Rate Multirotor ESC**: 400 Hz (2.5ms period), 1000 µs (zero throttle) to 2000 µs (full throttle).
- **Timer Prescaler Calculation**:
  $$\text{Prescaler} = \frac{f_{\text{TIM\_CLK}}}{1{,}000{,}000} - 1$$
  $$\text{Auto-Reload Register (ARR)} = \frac{1{,}000{,}000}{f_{\text{PWM}}} - 1$$
  $$\text{CCR} = \text{Pulse Width in Microseconds}$$

---

## Serial Communication & Bus Mapping

| Peripheral | STM32 Hardware Unit | Default Pins | Connected Device |
|:----------:|:-------------------:|:------------:|:-----------------|
| **I2C1** | `I2C1` | `PB8` (SCL), `PB9` (SDA) | Plug-and-Play Sensor Bus 1 (IMU / Baro) |
| **I2C2** | `I2C2` | `PB10` (SCL), `PB11` (SDA) | External Sensor Bus 2 (Compass / Magnetometer) |
| **SPI1** | `SPI1` | `PA5` (SCK), `PA6` (MISO), `PA7` (MOSI), `PA4` (CS) | High-speed external/modular IMU |
| **USART1** | `USART1` | `PA9` (TX), `PA10` (RX) | Telemetry Ground Station (MAVLink / Serial) |
| **USART2** | `USART2` | `PD5` (TX), `PD6` (RX) | GPS Module (38400 / 115200 baud) |
| **USART3** | `USART3` | `PD8` (TX), `PD9` (RX) | RC Receiver (SBUS / CRSF / PPM) |

---

## Building and Cross-Compiling

### 1. Build for Host Simulation & Unit Testing (x86_64 / Linux)
```bash
cd Autopilot_eclub
mkdir -p build && cd build
cmake ..
make -j$(nproc)
ctest --output-on-failure
./autopilot_app uav   # or 'ugv' or 'rov'
```

### 2. Cross-Compile for STM32 (ARM Cortex-M)
When targeting physical STM32 hardware with `arm-none-eabi-gcc`:
```bash
cmake -B build_stm32 \
  -DCMAKE_SYSTEM_NAME=Generic \
  -DCMAKE_SYSTEM_PROCESSOR=arm \
  -DCMAKE_C_COMPILER=arm-none-eabi-gcc \
  -DCMAKE_CXX_COMPILER=arm-none-eabi-g++ \
  -DSTM32_TARGET=ON \
  -DCMAKE_CXX_FLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
cmake --build build_stm32
```
