#pragma once

#include <cstdint>
#include <cstddef>

namespace autopilot {
namespace hal {

/**
 * @brief Hardware-independent I2C interface for plug-and-play sensors.
 */
class I2CDevice {
public:
    virtual ~I2CDevice() = default;

    /**
     * @brief Probe device at specific 7-bit I2C address to see if it ACKs.
     */
    virtual bool probe(uint8_t dev_addr) = 0;

    /**
     * @brief Read registers from an I2C device.
     */
    virtual bool read_registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) = 0;

    /**
     * @brief Write registers to an I2C device.
     */
    virtual bool write_registers(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) = 0;

    /**
     * @brief Convenience single-byte read.
     */
    virtual bool read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t& val) {
        return read_registers(dev_addr, reg_addr, &val, 1);
    }

    /**
     * @brief Convenience single-byte write.
     */
    virtual bool write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t val) {
        return write_registers(dev_addr, reg_addr, &val, 1);
    }
};

/**
 * @brief Hardware-independent SPI interface.
 */
class SPIDevice {
public:
    virtual ~SPIDevice() = default;

    virtual bool transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) = 0;
    virtual bool read_registers(uint8_t reg_addr, uint8_t* data, size_t len) = 0;
    virtual bool write_registers(uint8_t reg_addr, const uint8_t* data, size_t len) = 0;
};

/**
 * @brief Hardware CAN frame structure (CAN 2.0B).
 */
struct CANFrame {
    uint32_t id{0};          // 11-bit or 29-bit CAN ID (DroneCAN uses 29-bit)
    bool is_extended{true};  // 29-bit extended ID flag
    bool is_rtr{false};      // Remote transmission request
    uint8_t dlc{0};          // Data length code (0 - 8 bytes)
    uint8_t data[8]{0};
};

/**
 * @brief Hardware-independent CAN Bus interface for DroneCAN / UAVCAN sensors and ESCs.
 */
class CANDevice {
public:
    virtual ~CANDevice() = default;

    virtual bool init(uint32_t bitrate_bps = 1000000) = 0; // 1 Mbps default DroneCAN rate
    virtual bool send_frame(const CANFrame& frame) = 0;
    virtual bool receive_frame(CANFrame& frame) = 0;
    virtual size_t available() = 0;
};

/**
 * @brief Hardware-independent UART/Serial interface (GPS, Telemetry).
 */
class UARTDevice {
public:
    virtual ~UARTDevice() = default;

    virtual bool init(uint32_t baud_rate) = 0;
    virtual size_t write(const uint8_t* data, size_t len) = 0;
    virtual size_t read(uint8_t* buffer, size_t max_len) = 0;
    virtual size_t available() = 0;
};

/**
 * @brief PWM Output interface for ESCs, servos, and thrusters.
 * Supports configurable PWM channels, min/max pulses (typically 1000us - 2000us).
 */
class PWMOutput {
public:
    static constexpr uint8_t MAX_CHANNELS = 8;

    virtual ~PWMOutput() = default;

    /**
     * @brief Initialize PWM outputs at specified update frequency (e.g. 50Hz, 400Hz).
     */
    virtual bool init(uint32_t frequency_hz) = 0;

    /**
     * @brief Set pulse width for a given channel in microseconds (1000 - 2000 us).
     */
    virtual void set_channel(uint8_t channel, uint16_t pulse_us) = 0;

    /**
     * @brief Read back current commanded pulse width.
     */
    virtual uint16_t get_channel(uint8_t channel) const = 0;

    /**
     * @brief Arm/enable or disarm/disable PWM output lines.
     */
    virtual void enable_output(bool enable) = 0;

    /**
     * @brief Check if PWM hardware is armed/enabled.
     */
    virtual bool is_enabled() const = 0;
};

/**
 * @brief Timekeeping interface.
 */
class Time {
public:
    virtual ~Time() = default;
    virtual uint32_t millis() = 0;
    virtual uint64_t micros() = 0;
    virtual void delay_ms(uint32_t ms) = 0;
    virtual void delay_us(uint32_t us) = 0;
};

/**
 * @brief Top-level Hardware Abstraction Layer aggregating all peripherals.
 */
class HAL {
public:
    virtual ~HAL() = default;
    virtual bool init() = 0;

    virtual I2CDevice* get_i2c(uint8_t bus_index = 0) = 0;
    virtual SPIDevice* get_spi(uint8_t bus_index = 0) = 0;
    virtual CANDevice* get_can(uint8_t bus_index = 0) = 0;
    virtual UARTDevice* get_uart(uint8_t port_index = 0) = 0;
    virtual PWMOutput* get_pwm() = 0;
    virtual Time* get_time() = 0;
};

// Global HAL access helper
HAL* get_hal();
void set_hal(HAL* hal);

} // namespace hal
} // namespace autopilot
