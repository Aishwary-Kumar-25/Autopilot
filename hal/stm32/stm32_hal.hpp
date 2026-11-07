#pragma once

#include "hal/hal.hpp"
#include "stm32_pwm.hpp"

namespace autopilot {
namespace hal {

/**
 * @brief STM32 Hardware I2C Driver for Plug-and-Play sensors.
 * Compatible with STM32F4/F7/H7 I2C peripherals (I2C1, I2C2, I2C3, I2C4).
 */
class STM32I2CDevice : public I2CDevice {
public:
    STM32I2CDevice(void* hi2c_handle = nullptr);

    void set_handle(void* hi2c_handle) { _hi2c = hi2c_handle; }
    bool probe(uint8_t dev_addr) override;
    bool read_registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) override;
    bool write_registers(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) override;

private:
    void* _hi2c{nullptr};
};

/**
 * @brief STM32 Hardware SPI Driver.
 */
class STM32SPIDevice : public SPIDevice {
public:
    STM32SPIDevice(void* hspi_handle = nullptr, void* cs_gpio = nullptr, uint16_t cs_pin = 0);

    bool transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) override;
    bool read_registers(uint8_t reg_addr, uint8_t* data, size_t len) override;
    bool write_registers(uint8_t reg_addr, const uint8_t* data, size_t len) override;

private:
    void* _hspi{nullptr};
    void* _cs_port{nullptr};
    uint16_t _cs_pin{0};
};

/**
 * @brief STM32 Hardware CAN / FDCAN Driver for DroneCAN.
 */
class STM32CANDevice : public CANDevice {
public:
    STM32CANDevice(void* hcan_handle = nullptr);

    bool init(uint32_t bitrate_bps = 1000000) override;
    bool send_frame(const CANFrame& frame) override;
    bool receive_frame(CANFrame& frame) override;
    size_t available() override;

private:
    void* _hcan{nullptr};
};

/**
 * @brief STM32 Hardware UART Driver with DMA/Interrupt ring buffer.
 */
class STM32UARTDevice : public UARTDevice {
public:
    STM32UARTDevice(void* huart_handle = nullptr);

    bool init(uint32_t baud_rate) override;
    size_t write(const uint8_t* data, size_t len) override;
    size_t read(uint8_t* buffer, size_t max_len) override;
    size_t available() override;

private:
    void* _huart{nullptr};
};

/**
 * @brief STM32 DWT / SysTick high-resolution microsecond timer.
 */
class STM32Time : public Time {
public:
    STM32Time();
    uint32_t millis() override;
    uint64_t micros() override;
    void delay_ms(uint32_t ms) override;
    void delay_us(uint32_t us) override;
};

/**
 * @brief STM32 Target Hardware Abstraction Layer.
 */
class STM32HAL : public HAL {
public:
    STM32HAL();
    bool init() override;

    I2CDevice* get_i2c(uint8_t bus_index = 0) override;
    SPIDevice* get_spi(uint8_t bus_index = 0) override;
    CANDevice* get_can(uint8_t bus_index = 0) override;
    UARTDevice* get_uart(uint8_t port_index = 0) override;
    PWMOutput* get_pwm() override;
    Time* get_time() override;

    STM32I2CDevice& i2c() { return _i2c; }
    STM32SPIDevice& spi() { return _spi; }
    STM32CANDevice& can() { return _can; }
    STM32UARTDevice& uart() { return _uart; }
    STM32PWMOutput& pwm() { return _pwm; }
    STM32Time& time() { return _time; }

private:
    STM32I2CDevice _i2c;
    STM32SPIDevice _spi;
    STM32CANDevice _can;
    STM32UARTDevice _uart;
    STM32PWMOutput _pwm;
    STM32Time _time;
};

} // namespace hal
} // namespace autopilot
