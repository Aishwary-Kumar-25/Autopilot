#include "stm32_pwm.hpp"
#include <cstring>

#if defined(USE_HAL_DRIVER)
#include "main.h"
#define AP_ON_TARGET 1
#else
#define AP_ON_TARGET 0
#endif

namespace autopilot {
namespace hal {

STM32PWMOutput::STM32PWMOutput() {
    for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        _configs[i] = STM32PWMChannelConfig{};
        _pulses[i] = 1000;
    }
}

void STM32PWMOutput::configure_channel(uint8_t channel_idx, const STM32PWMChannelConfig& config) {
    if (channel_idx < MAX_CHANNELS) {
        _configs[channel_idx] = config;
    }
}

bool STM32PWMOutput::init(uint32_t frequency_hz) {
    if (frequency_hz == 0 || frequency_hz > 490) return false; // standard PWM ESC/servo range
    _frequency = frequency_hz;
#if AP_ON_TARGET
    // 1 MHz timer tick => CCR is the pulse width in microseconds.
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        auto* tim = static_cast<TIM_HandleTypeDef*>(_configs[i].timer_handle);
        if (!tim || _configs[i].timer_clock_hz < 1000000U) continue;
        __HAL_TIM_SET_PRESCALER(tim, (_configs[i].timer_clock_hz / 1000000U) - 1U);
        __HAL_TIM_SET_AUTORELOAD(tim, (1000000U / frequency_hz) - 1U);
    }
#endif
    enable_output(false);
    return true;
}

void STM32PWMOutput::set_channel(uint8_t channel, uint16_t pulse_us) {
    if (channel < MAX_CHANNELS) {
        _pulses[channel] = pulse_us;
#if AP_ON_TARGET
        auto* tim = static_cast<TIM_HandleTypeDef*>(_configs[channel].timer_handle);
        if (tim) {
            __HAL_TIM_SET_COMPARE(tim, _configs[channel].timer_channel, _enabled ? pulse_us : _failsafe_pulse);
        }
#endif
    }
}

uint16_t STM32PWMOutput::get_channel(uint8_t channel) const {
    if (channel < MAX_CHANNELS) {
        return _pulses[channel];
    }
    return _failsafe_pulse;
}

void STM32PWMOutput::enable_output(bool enable) {
    _enabled = enable;
    // Disarmed outputs keep running at the failsafe pulse (ESCs need a valid
    // signal to stay calibrated/armed-safe); armed outputs carry the demand.
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        set_channel(i, _pulses[i]);
#if AP_ON_TARGET
        auto* tim = static_cast<TIM_HandleTypeDef*>(_configs[i].timer_handle);
        if (tim) HAL_TIM_PWM_Start(tim, _configs[i].timer_channel);
#endif
    }
}

bool STM32PWMOutput::is_enabled() const {
    return _enabled;
}

void STM32PWMOutput::set_failsafe_pulse(uint16_t pulse_us) {
    _failsafe_pulse = pulse_us;
}

} // namespace hal
} // namespace autopilot
