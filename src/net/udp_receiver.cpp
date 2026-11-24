/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#include "udp_receiver.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <ctime>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>


namespace multiscan
{

namespace
{

uint64_t toNanoseconds(const timespec& time)
{
    return static_cast<uint64_t>(time.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(time.tv_nsec);
}

/* The SO_TIMESTAMPNS receive time of a message, or 0. */
uint64_t kernelReceiveTime(msghdr& header)
{
    for (cmsghdr* control = CMSG_FIRSTHDR(&header); control;
         control = CMSG_NXTHDR(&header, control))
    {
        if (control->cmsg_level == SOL_SOCKET &&
            control->cmsg_type == SCM_TIMESTAMPNS)
        {
            timespec time;
            std::memcpy(&time, CMSG_DATA(control), sizeof(time));
            return toNanoseconds(time);
        }
    }
    return 0;
}

}  // namespace


struct UdpReceiver::Socket
{
    using ControlBuffer = std::array<uint8_t, CMSG_SPACE(sizeof(timespec))>;

    int descriptor{-1};
    uint16_t port{0};
    std::vector<uint8_t> storage;  // batch_size * MAX_DATAGRAM_BYTES
    std::vector<mmsghdr> messages;
    std::vector<iovec> io_vectors;
    std::vector<ControlBuffer> control_buffers;

    ~Socket()
    {
        if (this->descriptor >= 0)
        {
            ::close(this->descriptor);
        }
    }
};



// --- Setup -------------------------------------------------------------------

UdpReceiver::UdpReceiver(size_t batch_size) :
    batch_size{std::max<size_t>(batch_size, 1)},
    wake_event_fd{::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)}
{
}

UdpReceiver::~UdpReceiver()
{
    this->close();
    if (this->wake_event_fd >= 0)
    {
        ::close(this->wake_event_fd);
    }
}

void UdpReceiver::close()
{
    this->sockets.clear();
    this->bound_ports.clear();
}

bool UdpReceiver::open(
    const std::vector<uint16_t>& ports,
    int receive_buffer_bytes,
    std::string& error)
{
    this->close();
    if (this->wake_event_fd < 0)
    {
        error = std::string("eventfd: ") + std::strerror(errno);
        return false;
    }
    std::vector<uint16_t> unique_ports;
    for (const uint16_t port : ports)
    {
        if (port == 0 ||
            std::find(unique_ports.begin(), unique_ports.end(), port) ==
                unique_ports.end())
        {
            unique_ports.push_back(port);
        }
    }
    for (const uint16_t port : unique_ports)
    {
        if (!this->openSocket(port, receive_buffer_bytes, error))
        {
            this->close();
            return false;
        }
    }
    return true;
}

bool UdpReceiver::openSocket(
    uint16_t port,
    int receive_buffer_bytes,
    std::string& error)
{
    Socket& socket = *this->sockets.emplace_back(std::make_unique<Socket>());
    socket.descriptor =
        ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket.descriptor < 0)
    {
        error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    if (receive_buffer_bytes > 0)
    {
        ::setsockopt(
            socket.descriptor,
            SOL_SOCKET,
            SO_RCVBUF,
            &receive_buffer_bytes,
            sizeof(receive_buffer_bytes));
    }
    const int enable = 1;
    ::setsockopt(
        socket.descriptor,
        SOL_SOCKET,
        SO_TIMESTAMPNS,
        &enable,
        sizeof(enable));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (::bind(
            socket.descriptor,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)) < 0)
    {
        error =
            "bind port " + std::to_string(port) + ": " + std::strerror(errno);
        return false;
    }
    socklen_t address_size = sizeof(address);
    ::getsockname(
        socket.descriptor,
        reinterpret_cast<sockaddr*>(&address),
        &address_size);
    socket.port = ntohs(address.sin_port);
    this->bound_ports.push_back(socket.port);

    int granted = 0;
    socklen_t granted_size = sizeof(granted);
    ::getsockopt(
        socket.descriptor,
        SOL_SOCKET,
        SO_RCVBUF,
        &granted,
        &granted_size);
    // Linux reports double the usable size
    this->granted_buffer_bytes = granted / 2;

    socket.storage.resize(this->batch_size * MAX_DATAGRAM_BYTES);
    socket.messages.resize(this->batch_size);
    socket.io_vectors.resize(this->batch_size);
    socket.control_buffers.resize(this->batch_size);
    return true;
}



// --- Receiving ---------------------------------------------------------------

void UdpReceiver::stop()
{
    this->stopped = true;
    if (this->wake_event_fd >= 0)
    {
        const uint64_t one = 1;
        [[maybe_unused]]
        const auto written = ::write(this->wake_event_fd, &one, sizeof(one));
    }
}

int UdpReceiver::receive(std::vector<Datagram>& out, int timeout_ms)
{
    out.clear();
    if (this->stopped || this->wake_event_fd < 0 || this->sockets.empty())
    {
        return -1;
    }
    std::vector<pollfd> descriptors;
    descriptors.reserve(this->sockets.size() + 1);
    descriptors.push_back({this->wake_event_fd, POLLIN, 0});
    for (const auto& socket : this->sockets)
    {
        descriptors.push_back({socket->descriptor, POLLIN, 0});
    }

    const int ready =
        ::poll(descriptors.data(), descriptors.size(), timeout_ms);
    if (ready < 0)
    {
        return errno == EINTR ? 0 : -1;
    }
    if (descriptors[0].revents & POLLIN)
    {
        return -1;  // stopped; the eventfd stays readable
    }
    if (ready == 0)
    {
        return 0;
    }

    for (size_t i = 0; i < this->sockets.size(); i++)
    {
        if ((descriptors[i + 1].revents & POLLIN) &&
            !this->readSocket(*this->sockets[i], out))
        {
            return -1;
        }
    }
    return static_cast<int>(out.size());
}

bool UdpReceiver::readSocket(Socket& socket, std::vector<Datagram>& out)
{
    for (size_t i = 0; i < this->batch_size; i++)
    {
        socket.io_vectors[i] = {
            socket.storage.data() + i * MAX_DATAGRAM_BYTES,
            MAX_DATAGRAM_BYTES};
        msghdr& header = socket.messages[i].msg_hdr;
        header = msghdr{};
        header.msg_iov = &socket.io_vectors[i];
        header.msg_iovlen = 1;
        header.msg_control = socket.control_buffers[i].data();
        header.msg_controllen = socket.control_buffers[i].size();
    }
    const int received = ::recvmmsg(
        socket.descriptor,
        socket.messages.data(),
        static_cast<unsigned>(this->batch_size),
        MSG_DONTWAIT,
        nullptr);
    if (received < 0)
    {
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }

    for (int i = 0; i < received; i++)
    {
        msghdr& header = socket.messages[i].msg_hdr;
        if (header.msg_flags & MSG_TRUNC)
        {
            this->truncated_datagrams++;
            continue;
        }
        Datagram datagram;
        datagram.data = {
            socket.storage.data() + i * MAX_DATAGRAM_BYTES,
            socket.messages[i].msg_len};
        datagram.port = socket.port;
        datagram.receive_time_ns = kernelReceiveTime(header);
        if (datagram.receive_time_ns == 0)
        {
            timespec now;
            ::clock_gettime(CLOCK_REALTIME, &now);
            datagram.receive_time_ns = toNanoseconds(now);
        }
        out.push_back(datagram);
    }
    return true;
}

}  // namespace multiscan
