#pragma once

#include "hal/hal.hpp"
#include <functional>
#include <vector>
#include <cstdint>

namespace autopilot {
namespace system {

struct ScheduledTask {
    const char* name;
    std::function<void()> func;
    uint32_t interval_micros;
    uint64_t last_run_micros{0};
    uint32_t max_run_time_micros{0};
};

/**
 * @brief Multi-rate real-time cyclic task scheduler.
 * Modeled after ArduPilot AP_Scheduler for predictable real-time execution.
 */
class Scheduler {
public:
    Scheduler(hal::Time* time);

    void add_task(const char* name, std::function<void()> func, uint32_t rate_hz);

    /**
     * @brief Run scheduled tasks that are due.
     */
    void run();

private:
    hal::Time* _time{nullptr};
    std::vector<ScheduledTask> _tasks;
};

} // namespace system
} // namespace autopilot
