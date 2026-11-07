#include "hal/posix/udp_link.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace autopilot {
namespace hal {

UdpLink::UdpLink(std::string remote_host, uint16_t remote_port, uint16_t local_port)
    : _remote_host(std::move(remote_host)), _remote_port(remote_port), _local_port(local_port) {}

UdpLink::~UdpLink() {
    if (_fd >= 0) ::close(_fd);
}

bool UdpLink::init(uint32_t) {
    if (_fd >= 0) return true;
    in_addr remote{};
    if (::inet_pton(AF_INET, _remote_host.c_str(), &remote) != 1) {
        _err = "bad remote address " + _remote_host;
        return false;
    }
    _remote_addr = remote.s_addr;
    _remote_port_n = htons(_remote_port);

    _fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (_fd < 0) {
        _err = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    const int yes = 1;
    ::setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(_local_port);
    if (::bind(_fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        _err = std::string("bind: ") + std::strerror(errno);
        ::close(_fd);
        _fd = -1;
        return false;
    }
    socklen_t len = sizeof(local);
    ::getsockname(_fd, reinterpret_cast<sockaddr*>(&local), &len);
    _bound_port = ntohs(local.sin_port);
    ::fcntl(_fd, F_SETFL, ::fcntl(_fd, F_GETFL, 0) | O_NONBLOCK);
    return true;
}

size_t UdpLink::write(const uint8_t* data, size_t len) {
    if (_fd < 0 || len == 0) return 0;
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = _remote_addr;
    to.sin_port = _remote_port_n;
    const ssize_t n = ::sendto(_fd, data, len, 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
    return n < 0 ? 0 : static_cast<size_t>(n); // no listener yet (ECONNREFUSED) is fine
}

void UdpLink::poll_socket() {
    if (_fd < 0) return;
    if (_rx_pos >= _rx.size()) {
        _rx.clear();
        _rx_pos = 0;
    }
    uint8_t buf[2048];
    for (int i = 0; i < 64; ++i) {
        sockaddr_in from{};
        socklen_t fl = sizeof(from);
        const ssize_t n = ::recvfrom(_fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &fl);
        if (n <= 0) break;
        // Reply to whoever is talking to us (GCS on another host / port).
        _remote_addr = from.sin_addr.s_addr;
        _remote_port_n = from.sin_port;
        _rx.insert(_rx.end(), buf, buf + n);
    }
}

size_t UdpLink::read(uint8_t* buffer, size_t max_len) {
    if (_rx_pos >= _rx.size()) poll_socket();
    const size_t n = std::min(max_len, _rx.size() - _rx_pos);
    if (n) std::memcpy(buffer, _rx.data() + _rx_pos, n);
    _rx_pos += n;
    return n;
}

size_t UdpLink::available() {
    if (_rx_pos >= _rx.size()) poll_socket();
    return _rx.size() - _rx_pos;
}

} // namespace hal
} // namespace autopilot
