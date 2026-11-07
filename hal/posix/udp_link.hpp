#pragma once
/**
 * MAVLink-over-UDP transport for SITL, exposed as a hal::UARTDevice so the
 * GCS link code is identical on a desktop and on a telemetry UART.
 *
 * Behaves like ArduPilot/PX4 SITL: it sends to the GCS port (default
 * 127.0.0.1:14550, where QGroundControl listens) from a local port, and once
 * a packet arrives it replies to whoever sent it (so a GCS on another machine
 * works after it sends its first heartbeat). Non-blocking; each write() is one
 * datagram (GcsLink writes exactly one MAVLink frame per call).
 */
#include "hal/hal.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace autopilot {
namespace hal {

class UdpLink : public UARTDevice {
public:
    /**
     * @param remote_host  where to send until a GCS talks to us (IPv4 dotted quad)
     * @param remote_port  GCS port (14550 = QGroundControl default)
     * @param local_port   port to bind (0 = any free port)
     */
    UdpLink(std::string remote_host = "127.0.0.1", uint16_t remote_port = 14550, uint16_t local_port = 0);
    ~UdpLink() override;
    UdpLink(const UdpLink&) = delete;
    UdpLink& operator=(const UdpLink&) = delete;

    bool init(uint32_t baud_rate = 0) override;   // baud ignored; opens the socket
    size_t write(const uint8_t* data, size_t len) override;
    size_t read(uint8_t* buffer, size_t max_len) override;
    size_t available() override;

    bool is_open() const { return _fd >= 0; }
    uint16_t local_port() const { return _bound_port; }
    const std::string& last_error() const { return _err; }

private:
    std::string _remote_host;
    uint16_t _remote_port;
    uint16_t _local_port;
    uint16_t _bound_port{0};
    int _fd{-1};
    uint32_t _remote_addr{0};    // network byte order
    uint16_t _remote_port_n{0};  // network byte order
    std::vector<uint8_t> _rx;
    size_t _rx_pos{0};
    std::string _err;

    void poll_socket();
};

} // namespace hal
} // namespace autopilot
