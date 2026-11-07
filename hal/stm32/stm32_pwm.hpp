#pragma once

#include "hal/hal.hpp"
#include <cstdint>

namespace autopilot {
namespace hal {

/**
 * @brief Pin and Hardware Timer descriptor for STM32 PWM Output Channels.
 * Designed to conform to Pixhawk / PX4 standard timer assignments.
 */
struct STM32PWMChannelConfig {
    void* timer_handle{nullptr};   // e.g. TIM_HandleTypeDef* in STM32 HAL
    uint32_t timer_channel{0};     // e.g. TIM_CHANNEL_1
    uint32_t timer_clock_hz{0};    // e.g. 84000000 or 168000000 Hz
    void* gpio_port{nullptr};      // e.g. GPIOA, GPIOB, etc.
    uint16_t gpio_pin{0};          // e.g. GPIO_PIN_8
};

/**
 * @brief STM32 Hardware PWM Output driver.
 * Supports configurable PWM channels across STM32 timers (TIM1, TIM2, TIM3, TIM4, TIM8).
 * Configured for 50Hz (Analog Servo), 400Hz (High-rate Multirotor ESC), or OneShot125.
 */
class STM32PWMOutput : public PWMOutput {
public:
    STM32PWMOutput();
    ~STM32PWMOutput() override = default;

    /**
     * @brief Configure a specific output channel with hardware timer and pin definitions.
     */
    void configure_channel(uint8_t channel_idx, const STM32PWMChannelConfig& config);

    bool init(uint32_t frequency_hz) override;
    void set_channel(uint8_t channel, uint16_t pulse_us) override;
    uint16_t get_channel(uint8_t channel) const override;
    void enable_output(bool enable) override;
    bool is_enabled() const override;

    /**
     * @brief Set fail-safe pulse width for all channels (typically 900-1000us or 1500us for ROV).
     */
    void set_failsafe_pulse(uint16_t pulse_us);

private:
    STM32PWMChannelConfig _configs[MAX_CHANNELS];
    uint16_t _pulses[MAX_CHANNELS]{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    uint16_t _failsafe_pulse{1000};
    uint32_t _frequency{50};
    bool _enabled{false};
};

} // namespace hal
} // namespace autopilot
