#include "scheduler.hpp"

namespace autopilot {
namespace system {

Scheduler::Scheduler(hal::Time* time) : _time(time) {}

void Scheduler::add_task(const char* name, std::function<void()> func, uint32_t rate_hz) {
    if (rate_hz == 0 || rate_hz > 1000000) return;
    ScheduledTask task;
    task.name = name;
    task.func = std::move(func);
    task.interval_micros = 1000000 / rate_hz;
    _tasks.push_back(std::move(task));
}

void Scheduler::run() {
    if (!_time) return;

    for (auto& task : _tasks) {
        const uint64_t now_us = _time->micros();
        if (!task.started) {
            task.started = true;
            task.next_run_micros = now_us;
        }
        if (now_us < task.next_run_micros) continue;

        // Phase-locked: the next deadline advances by exactly one period, so the
        // average rate is the requested rate. (Re-arming from "now" made every task
        // run slow by the polling jitter, e.g. a 400 Hz loop ran at ~370 Hz.)
        task.next_run_micros += task.interval_micros;
        if (now_us >= task.next_run_micros) {
            // More than a whole period late: don't burst-run to catch up, re-sync.
            ++task.overruns;
            task.next_run_micros = now_us + task.interval_micros;
        }
        if (task.func) {
            task.func();
            const uint64_t took = _time->micros() - now_us;
            if (took > task.max_run_time_micros) task.max_run_time_micros = static_cast<uint32_t>(took);
        }
    }
}

} // namespace system
} // namespace autopilot
