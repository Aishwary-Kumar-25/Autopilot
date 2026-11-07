#include "stm32_pwm.hpp"
#include <cstring>

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
    _frequency = frequency_hz;
    // On hardware STM32, timer prescalers and auto-reload registers (ARR)
    // are configured based on timer clock frequency to generate 1 microsecond timer ticks:
    // PSC = (TimerClock / 1000000) - 1
    // ARR = (1000000 / frequency_hz) - 1
    // CCR = pulse_width_us
    enable_output(false);
    return true;
}

void STM32PWMOutput::set_channel(uint8_t channel, uint16_t pulse_us) {
    if (channel < MAX_CHANNELS) {
        _pulses[channel] = pulse_us;
        if (_enabled) {
            // Hardware register write:
            // __HAL_TIM_SET_COMPARE(timer_handle, timer_channel, pulse_us);
        }
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
    if (_enabled) {
        for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
            set_channel(i, _pulses[i]);
            // HAL_TIM_PWM_Start(timer_handle, timer_channel);
        }
    } else {
        for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
            // Apply safe zero/failsafe pulse
            // __HAL_TIM_SET_COMPARE(timer_handle, timer_channel, _failsafe_pulse);
            // HAL_TIM_PWM_Stop(timer_handle, timer_channel);
        }
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
