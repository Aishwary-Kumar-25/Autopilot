#include "stm32_hal.hpp"
#include <cstring>

namespace autopilot {
namespace hal {

// ---------------- STM32I2CDevice ----------------

STM32I2CDevice::STM32I2CDevice(void* hi2c_handle) : _hi2c(hi2c_handle) {}

bool STM32I2CDevice::probe(uint8_t dev_addr) {
    if (!_hi2c) return false;
    // On hardware STM32 HAL:
    // HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
    //     (I2C_HandleTypeDef*)_hi2c, (uint16_t)(dev_addr << 1), 2, 10);
    // return status == HAL_OK;
    return true;
}

bool STM32I2CDevice::read_registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!_hi2c || !data || len == 0) return false;
    // On hardware STM32 HAL:
    // HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
    //     (I2C_HandleTypeDef*)_hi2c, (uint16_t)(dev_addr << 1),
    //     reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 50);
    // return status == HAL_OK;
    return true;
}

bool STM32I2CDevice::write_registers(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!_hi2c || !data || len == 0) return false;
    // On hardware STM32 HAL:
    // HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
    //     (I2C_HandleTypeDef*)_hi2c, (uint16_t)(dev_addr << 1),
    //     reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, 50);
    // return status == HAL_OK;
    return true;
}

// ---------------- STM32SPIDevice ----------------

STM32SPIDevice::STM32SPIDevice(void* hspi_handle, void* cs_gpio, uint16_t cs_pin)
    : _hspi(hspi_handle), _cs_port(cs_gpio), _cs_pin(cs_pin) {}

bool STM32SPIDevice::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) {
    if (!_hspi || len == 0) return false;
    // Assert CS: HAL_GPIO_WritePin((GPIO_TypeDef*)_cs_port, _cs_pin, GPIO_PIN_RESET);
    // HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)_hspi, (uint8_t*)tx_data, rx_data, len, 50);
    // Deassert CS: HAL_GPIO_WritePin((GPIO_TypeDef*)_cs_port, _cs_pin, GPIO_PIN_SET);
    return true;
}

bool STM32SPIDevice::read_registers(uint8_t reg_addr, uint8_t* data, size_t len) {
    uint8_t tx_cmd = reg_addr | 0x80; // SPI read bit
    return transfer(&tx_cmd, data, len);
}

bool STM32SPIDevice::write_registers(uint8_t reg_addr, const uint8_t* data, size_t len) {
    uint8_t tx_cmd = reg_addr & 0x7F; // SPI write bit
    return transfer(&tx_cmd, const_cast<uint8_t*>(data), len);
}

// ---------------- STM32CANDevice ----------------

STM32CANDevice::STM32CANDevice(void* hcan_handle) : _hcan(hcan_handle) {}

bool STM32CANDevice::init(uint32_t bitrate_bps) {
    (void)bitrate_bps;
    // On hardware STM32, configures bxCAN or FDCAN peripheral:
    // HAL_CAN_Start((CAN_HandleTypeDef*)_hcan);
    // HAL_CAN_ActivateNotification(..., CAN_IT_RX_FIFO0_MSG_PENDING);
    return true;
}

bool STM32CANDevice::send_frame(const CANFrame& frame) {
    if (!_hcan) return false;
    // CAN_TxHeaderTypeDef tx_header;
    // tx_header.ExtId = frame.id;
    // tx_header.IDE = frame.is_extended ? CAN_ID_EXT : CAN_ID_STD;
    // tx_header.DLC = frame.dlc;
    // uint32_t tx_mailbox;
    // return HAL_CAN_AddTxMessage((CAN_HandleTypeDef*)_hcan, &tx_header, (uint8_t*)frame.data, &tx_mailbox) == HAL_OK;
    (void)frame;
    return true;
}

bool STM32CANDevice::receive_frame(CANFrame& frame) {
    if (!_hcan) return false;
    // Pop frame from DMA/interrupt RX ring buffer
    (void)frame;
    return false;
}

size_t STM32CANDevice::available() {
    return 0;
}

// ---------------- STM32UARTDevice ----------------

STM32UARTDevice::STM32UARTDevice(void* huart_handle) : _huart(huart_handle) {}

bool STM32UARTDevice::init(uint32_t baud_rate) {
    // Hardware baud rate initialization or DMA circular buffer setup
    (void)baud_rate;
    return true;
}

size_t STM32UARTDevice::write(const uint8_t* data, size_t len) {
    if (!_huart || !data || len == 0) return 0;
    // HAL_UART_Transmit((UART_HandleTypeDef*)_huart, (uint8_t*)data, len, 100);
    return len;
}

size_t STM32UARTDevice::read(uint8_t* buffer, size_t max_len) {
    // Read from interrupt/DMA circular ring buffer
    (void)buffer;
    (void)max_len;
    return 0;
}

size_t STM32UARTDevice::available() {
    // Check ring buffer head - tail
    return 0;
}

// ---------------- STM32Time ----------------

STM32Time::STM32Time() {
    // Initialize ARM Cortex DWT (Data Watchpoint and Trace) cycle counter for sub-microsecond timing:
    // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t STM32Time::millis() {
    // return HAL_GetTick();
    return 0;
}

uint64_t STM32Time::micros() {
    // uint32_t cycles = DWT->CYCCNT;
    // return (uint64_t)cycles / (SystemCoreClock / 1000000);
    return 0;
}

void STM32Time::delay_ms(uint32_t ms) {
    // HAL_Delay(ms);
    (void)ms;
}

void STM32Time::delay_us(uint32_t us) {
    // uint32_t start_cycles = DWT->CYCCNT;
    // uint32_t wait_cycles = us * (SystemCoreClock / 1000000);
    // while ((DWT->CYCCNT - start_cycles) < wait_cycles);
    (void)us;
}

// ---------------- STM32HAL ----------------

STM32HAL::STM32HAL() {}

bool STM32HAL::init() {
    return true;
}

I2CDevice* STM32HAL::get_i2c(uint8_t) {
    return &_i2c;
}

SPIDevice* STM32HAL::get_spi(uint8_t) {
    return &_spi;
}

CANDevice* STM32HAL::get_can(uint8_t) {
    return &_can;
}

UARTDevice* STM32HAL::get_uart(uint8_t) {
    return &_uart;
}

PWMOutput* STM32HAL::get_pwm() {
    return &_pwm;
}

Time* STM32HAL::get_time() {
    return &_time;
}

} // namespace hal
} // namespace autopilot
