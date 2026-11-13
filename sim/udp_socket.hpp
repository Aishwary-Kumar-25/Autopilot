#pragma once
/**
 * Minimal non-blocking IPv4 UDP socket for the SITL side channels (simulator
 * bridge, external physics). Test/SITL only, POSIX sockets.
 */
#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace autopilot {
namespace sim {

class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket() { close(); }
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    /** Bind to 127.0.0.1-or-any:port (0 = any free port). */
    bool open(uint16_t port, bool any_interface = false) {
        _fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (_fd < 0) return fail("socket");
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port);
        a.sin_addr.s_addr = htonl(any_interface ? INADDR_ANY : INADDR_LOOPBACK);
        if (::bind(_fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) return fail("bind");
        ::fcntl(_fd, F_SETFL, ::fcntl(_fd, F_GETFL, 0) | O_NONBLOCK);
        int sz = 1 << 20;
        ::setsockopt(_fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof(sz));
        ::setsockopt(_fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
        socklen_t len = sizeof(a);
        ::getsockname(_fd, reinterpret_cast<sockaddr*>(&a), &len);
        _port = ntohs(a.sin_port);
        return true;
    }

    void close() {
        if (_fd >= 0) ::close(_fd);
        _fd = -1;
    }

    bool set_peer(const std::string& host, uint16_t port) {
        _peer = sockaddr_in{};
        _peer.sin_family = AF_INET;
        _peer.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &_peer.sin_addr) != 1) return fail("bad address " + host);
        _have_peer = true;
        return true;
    }

    /** Send one datagram to the peer (the configured one, or whoever wrote to us last). */
    bool send(const void* data, size_t len) {
        if (_fd < 0 || !_have_peer) return false;
        return ::sendto(_fd, data, len, 0, reinterpret_cast<const sockaddr*>(&_peer), sizeof(_peer)) ==
               static_cast<ssize_t>(len);
    }
    bool send(const std::string& s) { return send(s.data(), s.size()); }

    /**
     * Receive one datagram (waits up to timeout_ms, 0 = don't wait). Returns its
     * length, or -1 if nothing arrived. The sender becomes the peer when
     * follow_sender is set.
     */
    int recv(void* buf, size_t cap, int timeout_ms = 0) {
        if (_fd < 0) return -1;
        if (timeout_ms > 0) {
            pollfd p{_fd, POLLIN, 0};
            if (::poll(&p, 1, timeout_ms) <= 0) return -1;
        }
        sockaddr_in from{};
        socklen_t fl = sizeof(from);
        const ssize_t n = ::recvfrom(_fd, buf, cap, 0, reinterpret_cast<sockaddr*>(&from), &fl);
        if (n < 0) return -1;
        if (follow_sender) {
            _peer = from;
            _have_peer = true;
        }
        return static_cast<int>(n);
    }

    bool follow_sender{true};
    bool is_open() const { return _fd >= 0; }
    bool has_peer() const { return _have_peer; }
    uint16_t port() const { return _port; }
    const std::string& error() const { return _err; }

private:
    int _fd{-1};
    uint16_t _port{0};
    sockaddr_in _peer{};
    bool _have_peer{false};
    std::string _err;
    bool fail(const std::string& what) {
        _err = what + ": " + std::strerror(errno);
        close();
        return false;
    }
};

} // namespace sim
} // namespace autopilot
