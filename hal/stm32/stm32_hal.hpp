#pragma once

#include "hal/hal.hpp"
#include "stm32_pwm.hpp"
#include "stm32_flash.hpp"

/*
 * Build modes
 *  - On target (STM32CubeMX project, USE_HAL_DRIVER defined): every method calls
 *    the ST HAL. CAN uses bxCAN (HAL_CAN_MODULE_ENABLED, F1/F4/F7) or FDCAN
 *    (HAL_FDCAN_MODULE_ENABLED, G4/H7). RX for CAN and UART is interrupt driven:
 *    call on_rx_isr()/on_rx_byte_isr() from the corresponding HAL callback.
 *  - Off target (host build): there is no hardware, so every bus call FAILS
 *    (returns false / 0). Returning "success" from an empty stub made the sensor
 *    manager believe sensors were present and healthy.
 */

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

    /** Push a received frame (call from the CAN RX FIFO interrupt callback). */
    void on_rx_isr(const CANFrame& frame);
    uint32_t rx_overruns() const { return _rx_overruns; }

private:
    void* _hcan{nullptr};
    static constexpr uint16_t RX_SIZE = 64;   // power of two
    CANFrame _rx[RX_SIZE];
    volatile uint16_t _rx_head{0};   // written by ISR
    volatile uint16_t _rx_tail{0};   // written by main loop
    uint32_t _rx_overruns{0};
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

    /** Push a received byte (call from HAL_UART_RxCpltCallback / IDLE+DMA handler). */
    void on_rx_byte_isr(uint8_t byte);
    /** Arms single-byte interrupt reception (call once after init, and from the RX callback). */
    void restart_rx_it();

private:
    void* _huart{nullptr};
    static constexpr uint16_t RX_SIZE = 512;  // power of two: ~100 ms of GPS at 38400 baud
    uint8_t _rx[RX_SIZE]{};
    volatile uint16_t _rx_head{0};
    volatile uint16_t _rx_tail{0};
    uint8_t _it_byte{0};
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
    /** Flash-backed settings storage (mounted in init(); nullptr if the flash can't be read). */
    Storage* get_storage() override;

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
    static constexpr size_t STORAGE_SIZE = 4096;   // RAM mirror; layout in comms/storage_layout.hpp
    uint8_t _storage_mirror[STORAGE_SIZE];
    STM32FlashDevice _flash;
    FlashStorage _storage{&_flash, _storage_mirror, STORAGE_SIZE};
};

} // namespace hal
} // namespace autopilot
