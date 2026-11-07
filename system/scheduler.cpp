#include "scheduler.hpp"

namespace autopilot {
namespace system {

Scheduler::Scheduler(hal::Time* time) : _time(time) {}

void Scheduler::add_task(const char* name, std::function<void()> func, uint32_t rate_hz) {
    if (rate_hz == 0) return;
    ScheduledTask task;
    task.name = name;
    task.func = func;
    task.interval_micros = 1000000 / rate_hz;
    task.last_run_micros = 0;
    _tasks.push_back(task);
}

void Scheduler::run() {
    if (!_time) return;

    uint64_t now_us = _time->micros();

    for (auto& task : _tasks) {
        if (task.last_run_micros == 0 || (now_us - task.last_run_micros) >= task.interval_micros) {
            task.last_run_micros = now_us;
            if (task.func) {
                task.func();
            }
        }
    }
}

} // namespace system
} // namespace autopilot
