#pragma once

#include "hal/hal.hpp"
#include <chrono>
#include <vector>
#include <unordered_map>
#include <functional>
#include <array>
#include <thread>

namespace autopilot {
namespace hal {

/**
 * @brief Simulated/POSIX I2C Device for unit testing and SIL (Software-in-the-loop).
 * Allows dynamic registration of device addresses, simulating plug/unplug events.
 */
class PosixI2CDevice : public I2CDevice {
public:
    using RegisterMap = std::unordered_map<uint8_t, uint8_t>;

    bool probe(uint8_t dev_addr) override;
    bool read_registers(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) override;
    bool write_registers(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) override;

    // Simulation helpers
    void attach_device(uint8_t dev_addr, const RegisterMap& initial_regs);
    void detach_device(uint8_t dev_addr);
    void set_register(uint8_t dev_addr, uint8_t reg_addr, uint8_t val);
    uint8_t get_register(uint8_t dev_addr, uint8_t reg_addr) const;
    void set_simulate_bus_error(bool error) { _simulate_error = error; }

private:
    std::unordered_map<uint8_t, RegisterMap> _devices;
    bool _simulate_error{false};
};

class PosixSPIDevice : public SPIDevice {
public:
    bool transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t len) override;
    bool read_registers(uint8_t reg_addr, uint8_t* data, size_t len) override;
    bool write_registers(uint8_t reg_addr, const uint8_t* data, size_t len) override;
};

class PosixCANDevice : public CANDevice {
public:
    bool init(uint32_t bitrate_bps = 1000000) override;
    bool send_frame(const CANFrame& frame) override;
    bool receive_frame(CANFrame& frame) override;
    size_t available() override;

    void inject_frame(const CANFrame& frame);
    const std::vector<CANFrame>& get_transmitted_frames() const { return _tx_frames; }
    void clear_transmitted_frames() { _tx_frames.clear(); }

private:
    std::vector<CANFrame> _rx_queue;
    std::vector<CANFrame> _tx_frames;
    uint32_t _bitrate{1000000};
};

class PosixUARTDevice : public UARTDevice {
public:
    bool init(uint32_t baud_rate) override;
    size_t write(const uint8_t* data, size_t len) override;
    size_t read(uint8_t* buffer, size_t max_len) override;
    size_t available() override;

    // Inject simulated input (e.g. GPS NMEA sentences)
    void inject_input(const std::string& input);
    std::string get_output_history() const;
    void clear_output_history();

private:
    std::vector<uint8_t> _rx_buffer;
    std::vector<uint8_t> _tx_buffer;
};

class PosixPWMOutput : public PWMOutput {
public:
    bool init(uint32_t frequency_hz) override;
    void set_channel(uint8_t channel, uint16_t pulse_us) override;
    uint16_t get_channel(uint8_t channel) const override;
    void enable_output(bool enable) override;
    bool is_enabled() const override;

private:
    std::array<uint16_t, MAX_CHANNELS> _channels{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    bool _enabled{false};
    uint32_t _frequency{50};
};

class PosixTime : public Time {
public:
    PosixTime();
    uint32_t millis() override;
    uint64_t micros() override;
    void delay_ms(uint32_t ms) override;
    void delay_us(uint32_t us) override;

    // Simulation time warp (allows fast deterministic testing)
    void advance_time_ms(uint32_t ms);
    void set_manual_time_mode(bool manual);

private:
    std::chrono::steady_clock::time_point _start_time;
    bool _manual_time{false};
    uint64_t _simulated_micros{0};
};

class PosixHAL : public HAL {
public:
    PosixHAL();
    bool init() override;

    I2CDevice* get_i2c(uint8_t bus_index = 0) override;
    SPIDevice* get_spi(uint8_t bus_index = 0) override;
    CANDevice* get_can(uint8_t bus_index = 0) override;
    UARTDevice* get_uart(uint8_t port_index = 0) override;
    PWMOutput* get_pwm() override;
    Time* get_time() override;

    PosixI2CDevice& i2c() { return _i2c; }
    PosixSPIDevice& spi() { return _spi; }
    PosixCANDevice& can() { return _can; }
    PosixUARTDevice& uart() { return _uart; }
    PosixPWMOutput& pwm() { return _pwm; }
    PosixTime& time() { return _time; }

private:
    PosixI2CDevice _i2c;
    PosixSPIDevice _spi;
    PosixCANDevice _can;
    PosixUARTDevice _uart;
    PosixPWMOutput _pwm;
    PosixTime _time;
};

} // namespace hal
} // namespace autopilot
