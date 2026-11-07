#include "posix_hal.hpp"
#include <cstring>
#include <iostream>

namespace autopilot {
namespace hal {

static HAL* g_current_hal = nullptr;

HAL* get_hal() {
    return g_current_hal;
}

void set_hal(HAL* hal) {
    g_current_hal = hal;
}

// ---------------- PosixI2CDevice ----------------

bool PosixI2CDevice::probe(uint8_t dev_addr) {
    if (_simulate_error) return false;
    return _devices.find(dev_addr) != _devices.end();
}

bool PosixI2CDevice::read_registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (_simulate_error || data == nullptr) return false;
    auto it = _devices.find(dev_addr);
    if (it == _devices.end()) return false;

    const auto& reg_map = it->second;
    for (size_t i = 0; i < len; ++i) {
        uint8_t curr_reg = reg_addr + static_cast<uint8_t>(i);
        auto reg_it = reg_map.find(curr_reg);
        if (reg_it != reg_map.end()) {
            data[i] = reg_it->second;
        } else {
            data[i] = 0x00;
        }
    }
    return true;
}

bool PosixI2CDevice::write_registers(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (_simulate_error || data == nullptr) return false;
    auto it = _devices.find(dev_addr);
    if (it == _devices.end()) return false;

    auto& reg_map = it->second;
    for (size_t i = 0; i < len; ++i) {
        uint8_t curr_reg = reg_addr + static_cast<uint8_t>(i);
        reg_map[curr_reg] = data[i];
    }
    return true;
}

void PosixI2CDevice::attach_device(uint8_t dev_addr, const RegisterMap& initial_regs) {
    _devices[dev_addr] = initial_regs;
}

void PosixI2CDevice::detach_device(uint8_t dev_addr) {
    _devices.erase(dev_addr);
}

void PosixI2CDevice::set_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t val) {
    _devices[dev_addr][reg_addr] = val;
}

uint8_t PosixI2CDevice::get_register(uint8_t dev_addr, uint8_t reg_addr) const {
    auto it = _devices.find(dev_addr);
    if (it == _devices.end()) return 0;
    auto reg_it = it->second.find(reg_addr);
    if (reg_it == it->second.end()) return 0;
    return reg_it->second;
}

// ---------------- PosixSPIDevice ----------------

bool PosixSPIDevice::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) {
    if (rx_data && tx_data) {
        std::memcpy(rx_data, tx_data, len);
    }
    return true;
}

bool PosixSPIDevice::read_registers(uint8_t, uint8_t* data, size_t len) {
    if (data) {
        std::memset(data, 0, len);
    }
    return true;
}

bool PosixSPIDevice::write_registers(uint8_t, const uint8_t*, size_t) {
    return true;
}

// ---------------- PosixCANDevice ----------------

bool PosixCANDevice::init(uint32_t bitrate_bps) {
    _bitrate = bitrate_bps;
    _rx_queue.clear();
    _tx_frames.clear();
    return true;
}

bool PosixCANDevice::send_frame(const CANFrame& frame) {
    _tx_frames.push_back(frame);
    return true;
}

bool PosixCANDevice::receive_frame(CANFrame& frame) {
    if (_rx_queue.empty()) return false;
    frame = _rx_queue.front();
    _rx_queue.erase(_rx_queue.begin());
    return true;
}

size_t PosixCANDevice::available() {
    return _rx_queue.size();
}

void PosixCANDevice::inject_frame(const CANFrame& frame) {
    _rx_queue.push_back(frame);
}

// ---------------- PosixUARTDevice ----------------

bool PosixUARTDevice::init(uint32_t) {
    _rx_buffer.clear();
    _tx_buffer.clear();
    return true;
}

size_t PosixUARTDevice::write(const uint8_t* data, size_t len) {
    if (!data || len == 0) return 0;
    _tx_buffer.insert(_tx_buffer.end(), data, data + len);
    return len;
}

size_t PosixUARTDevice::read(uint8_t* buffer, size_t max_len) {
    if (!buffer || max_len == 0 || _rx_buffer.empty()) return 0;
    size_t count = std::min(max_len, _rx_buffer.size());
    std::memcpy(buffer, _rx_buffer.data(), count);
    _rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + count);
    return count;
}

size_t PosixUARTDevice::available() {
    return _rx_buffer.size();
}

void PosixUARTDevice::inject_input(const std::string& input) {
    _rx_buffer.insert(_rx_buffer.end(), input.begin(), input.end());
}

std::string PosixUARTDevice::get_output_history() const {
    return std::string(_tx_buffer.begin(), _tx_buffer.end());
}

void PosixUARTDevice::clear_output_history() {
    _tx_buffer.clear();
}

// ---------------- PosixPWMOutput ----------------

PosixPWMOutput::PosixPWMOutput() {
    _channels.fill(1000);
}

bool PosixPWMOutput::init(uint32_t frequency_hz) {
    _frequency = frequency_hz;
    _channels.fill(1000);
    _enabled = false;
    return true;
}

void PosixPWMOutput::set_channel(uint8_t channel, uint16_t pulse_us) {
    if (channel < MAX_CHANNELS) {
        _channels[channel] = pulse_us;
    }
}

uint16_t PosixPWMOutput::get_channel(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return _channels[channel];
    }
    return 1000;
}

void PosixPWMOutput::enable_output(bool enable) {
    _enabled = enable;
}

bool PosixPWMOutput::is_enabled() const {
    return _enabled;
}

// ---------------- PosixTime ----------------

PosixTime::PosixTime() {
    _start_time = std::chrono::steady_clock::now();
}

uint32_t PosixTime::millis() {
    if (_manual_time) {
        return static_cast<uint32_t>(_simulated_micros / 1000);
    }
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - _start_time).count();
}

uint64_t PosixTime::micros() {
    if (_manual_time) {
        return _simulated_micros;
    }
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - _start_time).count();
}

void PosixTime::delay_ms(uint32_t ms) {
    if (_manual_time) {
        _simulated_micros += static_cast<uint64_t>(ms) * 1000;
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

void PosixTime::delay_us(uint32_t us) {
    if (_manual_time) {
        _simulated_micros += us;
    } else {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
}

void PosixTime::advance_time_ms(uint32_t ms) {
    _simulated_micros += static_cast<uint64_t>(ms) * 1000;
}

void PosixTime::set_manual_time_mode(bool manual) {
    _manual_time = manual;
}

// ---------------- PosixHAL ----------------

PosixHAL::PosixHAL() {
    set_hal(this);
}

bool PosixHAL::init() {
    return true;
}

I2CDevice* PosixHAL::get_i2c(uint8_t) {
    return &_i2c;
}

SPIDevice* PosixHAL::get_spi(uint8_t) {
    return &_spi;
}

CANDevice* PosixHAL::get_can(uint8_t) {
    return &_can;
}

UARTDevice* PosixHAL::get_uart(uint8_t) {
    return &_uart;
}

PWMOutput* PosixHAL::get_pwm() {
    return &_pwm;
}

Time* PosixHAL::get_time() {
    return &_time;
}

} // namespace hal
} // namespace autopilot
