#pragma once

#include <cstdint>
#include <cstring>
#include <atomic>
#include <array>

namespace autopilot {
namespace system {

/**
 * @brief Ultra-lightweight, zero-allocation Publish/Subscribe message topic.
 * Designed for real-time embedded systems (inspired by PX4 uORB and ROS).
 * Decouples sensor drivers, estimators, and vehicle controllers so a single
 * firmware binary can switch between UAV, UGV, and ROV without reflashing.
 */
template <typename T, size_t QUEUE_SIZE = 4>
class Topic {
public:
    Topic() : _write_index(0), _publish_count(0) {}

    /**
     * @brief Publish a new message instance to all subscribers.
     * Lock-free and ISR-safe.
     */
    void publish(const T& msg) {
        size_t next_idx = (_write_index.load(std::memory_order_relaxed) + 1) % QUEUE_SIZE;
        _buffer[next_idx] = msg;
        _write_index.store(next_idx, std::memory_order_release);
        _publish_count.fetch_add(1, std::memory_order_relaxed);
    }

    /**
     * @brief Subscriber handle tracking its individual read position.
     */
    class Subscriber {
    public:
        Subscriber(Topic<T, QUEUE_SIZE>& topic)
            : _topic(topic), _last_read_count(0) {}

        /**
         * @brief Check if new data has been published since last read.
         */
        bool updated() const {
            return _topic._publish_count.load(std::memory_order_relaxed) != _last_read_count;
        }

        /**
         * @brief Copy latest published message.
         * Returns true if fresh data was copied, false if no new data was published.
         */
        bool copy(T& dest) {
            uint64_t current_count = _topic._publish_count.load(std::memory_order_relaxed);
            if (current_count == _last_read_count) {
                return false; // No new data
            }

            size_t idx = _topic._write_index.load(std::memory_order_acquire);
            dest = _topic._buffer[idx];
            _last_read_count = current_count;
            return true;
        }

        /**
         * @brief Get latest message regardless of whether it's new.
         */
        void get_latest(T& dest) const {
            size_t idx = _topic._write_index.load(std::memory_order_acquire);
            dest = _topic._buffer[idx];
        }

    private:
        Topic<T, QUEUE_SIZE>& _topic;
        uint64_t _last_read_count;
    };

    Subscriber subscribe() {
        return Subscriber(*this);
    }

    uint64_t get_publish_count() const {
        return _publish_count.load(std::memory_order_relaxed);
    }

private:
    std::array<T, QUEUE_SIZE> _buffer{};
    std::atomic<size_t> _write_index{0};
    std::atomic<uint64_t> _publish_count{0};
};

} // namespace system
} // namespace autopilot

