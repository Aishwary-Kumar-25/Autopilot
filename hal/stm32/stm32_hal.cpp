#include "stm32_hal.hpp"
#include <atomic>
#include <cstring>

#if defined(USE_HAL_DRIVER)
#include "main.h"   // CubeMX: pulls in the family HAL (stm32f4xx_hal.h, stm32h7xx_hal.h, ...)
#define AP_ON_TARGET 1
#else
#define AP_ON_TARGET 0
#endif

namespace autopilot {
namespace hal {

// ---------------- STM32I2CDevice ----------------

STM32I2CDevice::STM32I2CDevice(void* hi2c_handle) : _hi2c(hi2c_handle) {}

bool STM32I2CDevice::probe(uint8_t dev_addr) {
    if (!_hi2c) return false;
#if AP_ON_TARGET
    return HAL_I2C_IsDeviceReady(static_cast<I2C_HandleTypeDef*>(_hi2c), static_cast<uint16_t>(dev_addr << 1), 2, 10) == HAL_OK;
#else
    (void)dev_addr;
    return false;
#endif
}

bool STM32I2CDevice::read_registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!_hi2c || !data || len == 0) return false;
#if AP_ON_TARGET
    return HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef*>(_hi2c), static_cast<uint16_t>(dev_addr << 1), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, data, static_cast<uint16_t>(len), 10) == HAL_OK;
#else
    (void)dev_addr; (void)reg_addr;
    return false;
#endif
}

bool STM32I2CDevice::write_registers(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!_hi2c || !data || len == 0) return false;
#if AP_ON_TARGET
    return HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef*>(_hi2c), static_cast<uint16_t>(dev_addr << 1), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, const_cast<uint8_t*>(data), static_cast<uint16_t>(len), 10) == HAL_OK;
#else
    (void)dev_addr; (void)reg_addr;
    return false;
#endif
}

// ---------------- STM32SPIDevice ----------------

STM32SPIDevice::STM32SPIDevice(void* hspi_handle, void* cs_gpio, uint16_t cs_pin)
    : _hspi(hspi_handle), _cs_port(cs_gpio), _cs_pin(cs_pin) {}

bool STM32SPIDevice::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) {
    if (!_hspi || len == 0 || (!tx_data && !rx_data)) return false;
#if AP_ON_TARGET
    auto* h = static_cast<SPI_HandleTypeDef*>(_hspi);
    HAL_StatusTypeDef st;
    if (_cs_port) HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(_cs_port), _cs_pin, GPIO_PIN_RESET);
    if (tx_data && rx_data) {
        st = HAL_SPI_TransmitReceive(h, const_cast<uint8_t*>(tx_data), rx_data, static_cast<uint16_t>(len), 10);
    } else if (tx_data) {
        st = HAL_SPI_Transmit(h, const_cast<uint8_t*>(tx_data), static_cast<uint16_t>(len), 10);
    } else {
        st = HAL_SPI_Receive(h, rx_data, static_cast<uint16_t>(len), 10);
    }
    if (_cs_port) HAL_GPIO_WritePin(static_cast<GPIO_TypeDef*>(_cs_port), _cs_pin, GPIO_PIN_SET);
    return st == HAL_OK;
#else
    return false;
#endif
}

// Register access is ONE chip-select window: [cmd byte][len data bytes].
// (The old version clocked `len` bytes out of a 1-byte command buffer: a buffer
// over-read, and the data came back shifted by one byte.)
bool STM32SPIDevice::read_registers(uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!data || len == 0 || len > 64) return false;
    uint8_t tx[65]{};
    uint8_t rx[65]{};
    tx[0] = reg_addr | 0x80; // SPI read bit
    if (!transfer(tx, rx, len + 1)) return false;
    std::memcpy(data, rx + 1, len);
    return true;
}

bool STM32SPIDevice::write_registers(uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!data || len == 0 || len > 64) return false;
    uint8_t tx[65]{};
    tx[0] = reg_addr & 0x7F; // SPI write bit
    std::memcpy(tx + 1, data, len);
    return transfer(tx, nullptr, len + 1);
}

// ---------------- STM32CANDevice ----------------

STM32CANDevice::STM32CANDevice(void* hcan_handle) : _hcan(hcan_handle) {}

bool STM32CANDevice::init(uint32_t bitrate_bps) {
    (void)bitrate_bps; // bit timing comes from the CubeMX peripheral configuration
    if (!_hcan) return false;
#if AP_ON_TARGET && defined(HAL_CAN_MODULE_ENABLED)
    auto* h = static_cast<CAN_HandleTypeDef*>(_hcan);
    CAN_FilterTypeDef f{};            // accept everything into FIFO0
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(h, &f) != HAL_OK) return false;
    if (HAL_CAN_Start(h) != HAL_OK) return false;
    return HAL_CAN_ActivateNotification(h, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
#elif AP_ON_TARGET && defined(HAL_FDCAN_MODULE_ENABLED)
    auto* h = static_cast<FDCAN_HandleTypeDef*>(_hcan);
    if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) return false;
    if (HAL_FDCAN_Start(h) != HAL_OK) return false;
    return HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK;
#else
    return false;
#endif
}

bool STM32CANDevice::send_frame(const CANFrame& frame) {
    if (!_hcan || frame.dlc > 8) return false;
#if AP_ON_TARGET && defined(HAL_CAN_MODULE_ENABLED)
    CAN_TxHeaderTypeDef hdr{};
    hdr.IDE = frame.is_extended ? CAN_ID_EXT : CAN_ID_STD;
    if (frame.is_extended) hdr.ExtId = frame.id; else hdr.StdId = frame.id;
    hdr.RTR = frame.is_rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = frame.dlc;
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(static_cast<CAN_HandleTypeDef*>(_hcan), &hdr, const_cast<uint8_t*>(frame.data),
                                &mailbox) == HAL_OK;
#elif AP_ON_TARGET && defined(HAL_FDCAN_MODULE_ENABLED)
    static const uint32_t DLC[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
                                    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
                                    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};
    FDCAN_TxHeaderTypeDef hdr{};
    hdr.Identifier = frame.id;
    hdr.IdType = frame.is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    hdr.TxFrameType = frame.is_rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    hdr.DataLength = DLC[frame.dlc];
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(static_cast<FDCAN_HandleTypeDef*>(_hcan), &hdr,
                                         const_cast<uint8_t*>(frame.data)) == HAL_OK;
#else
    (void)frame;
    return false;
#endif
}

void STM32CANDevice::on_rx_isr(const CANFrame& frame) {
    const uint16_t next = static_cast<uint16_t>((_rx_head + 1) & (RX_SIZE - 1));
    if (next == _rx_tail) {   // full: drop newest, count it
        ++_rx_overruns;
        return;
    }
    _rx[_rx_head] = frame;
    std::atomic_signal_fence(std::memory_order_release); // data before index (ISR -> main)
    _rx_head = next;
}

bool STM32CANDevice::receive_frame(CANFrame& frame) {
    if (_rx_tail == _rx_head) return false;
    std::atomic_signal_fence(std::memory_order_acquire);
    frame = _rx[_rx_tail];
    _rx_tail = static_cast<uint16_t>((_rx_tail + 1) & (RX_SIZE - 1));
    return true;
}

size_t STM32CANDevice::available() {
    return static_cast<uint16_t>((_rx_head - _rx_tail) & (RX_SIZE - 1));
}

// ---------------- STM32UARTDevice ----------------

STM32UARTDevice::STM32UARTDevice(void* huart_handle) : _huart(huart_handle) {}

bool STM32UARTDevice::init(uint32_t baud_rate) {
    if (!_huart) return false;
#if AP_ON_TARGET
    auto* h = static_cast<UART_HandleTypeDef*>(_huart);
    if (h->Init.BaudRate != baud_rate) {
        h->Init.BaudRate = baud_rate;
        if (HAL_UART_Init(h) != HAL_OK) return false;
    }
    _rx_head = _rx_tail = 0;
    restart_rx_it();
    return true;
#else
    (void)baud_rate;
    return false;
#endif
}

void STM32UARTDevice::restart_rx_it() {
#if AP_ON_TARGET
    if (_huart) HAL_UART_Receive_IT(static_cast<UART_HandleTypeDef*>(_huart), &_it_byte, 1);
#endif
}

void STM32UARTDevice::on_rx_byte_isr(uint8_t byte) {
    const uint16_t next = static_cast<uint16_t>((_rx_head + 1) & (RX_SIZE - 1));
    if (next != _rx_tail) {
        _rx[_rx_head] = byte;
        std::atomic_signal_fence(std::memory_order_release);
        _rx_head = next;
    }
}

size_t STM32UARTDevice::write(const uint8_t* data, size_t len) {
    if (!_huart || !data || len == 0) return 0;
#if AP_ON_TARGET
    return HAL_UART_Transmit(static_cast<UART_HandleTypeDef*>(_huart), const_cast<uint8_t*>(data),
                             static_cast<uint16_t>(len), 10) == HAL_OK ? len : 0;
#else
    return 0;
#endif
}

size_t STM32UARTDevice::read(uint8_t* buffer, size_t max_len) {
    if (!buffer) return 0;
    size_t n = 0;
    while (n < max_len && _rx_tail != _rx_head) {
        buffer[n++] = _rx[_rx_tail];
        _rx_tail = static_cast<uint16_t>((_rx_tail + 1) & (RX_SIZE - 1));
    }
    return n;
}

size_t STM32UARTDevice::available() {
    return static_cast<uint16_t>((_rx_head - _rx_tail) & (RX_SIZE - 1));
}

// ---------------- STM32Time ----------------

STM32Time::STM32Time() {
#if AP_ON_TARGET && defined(DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // DWT cycle counter for micros()
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

uint32_t STM32Time::millis() {
#if AP_ON_TARGET
    return HAL_GetTick();
#else
    return 0;
#endif
}

uint64_t STM32Time::micros() {
#if AP_ON_TARGET && defined(DWT)
    // Extend the 32-bit cycle counter (wraps every ~25 s at 168 MHz). Must be
    // called at least once per wrap period: the flight loop does.
    static uint32_t last = 0;
    static uint64_t high = 0;
    const uint32_t now = DWT->CYCCNT;
    if (now < last) high += (1ULL << 32);
    last = now;
    return (high | now) / (SystemCoreClock / 1000000U);
#elif AP_ON_TARGET
    return static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
#else
    return 0;
#endif
}

void STM32Time::delay_ms(uint32_t ms) {
#if AP_ON_TARGET
    HAL_Delay(ms);
#else
    (void)ms;
#endif
}

void STM32Time::delay_us(uint32_t us) {
#if AP_ON_TARGET && defined(DWT)
    const uint32_t start = DWT->CYCCNT;
    const uint32_t wait = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < wait) {
    }
#else
    (void)us;
#endif
}

// ---------------- STM32HAL ----------------

STM32HAL::STM32HAL() {}

bool STM32HAL::init() {
    // Peripherals are initialised by CubeMX (MX_*_Init) before this is called;
    // the handles are passed in via the device constructors / set_handle().
    _storage.mount();   // settings flash; a blank or damaged copy mounts as empty storage
    return AP_ON_TARGET != 0;
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

Storage* STM32HAL::get_storage() {
    return _storage.mounted() ? &_storage : nullptr;
}

} // namespace hal
} // namespace autopilot
