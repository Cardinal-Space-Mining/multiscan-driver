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
#include <ctime>
#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>


namespace multiscan
{

struct UdpReceiver::Socket
{
    int fd = -1;
    uint16_t port = 0;
    std::vector<uint8_t> storage;  // batch * MAX_DATAGRAM_BYTES
    std::vector<mmsghdr> msgs;
    std::vector<iovec> iovs;
    std::vector<std::array<uint8_t, CMSG_SPACE(sizeof(timespec))>> controls;
};

UdpReceiver::UdpReceiver(size_t batch_size) :
    batch{std::max<size_t>(batch_size, 1)},
    wake_fd{::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)}
{
}

UdpReceiver::~UdpReceiver()
{
    close();
    if (wake_fd >= 0)
    {
        ::close(wake_fd);
    }
}

void UdpReceiver::close()
{
    for (Socket* s : sockets)
    {
        if (s->fd >= 0)
        {
            ::close(s->fd);
        }
        delete s;
    }
    sockets.clear();
    bound_ports.clear();
}

bool UdpReceiver::open(
    const std::vector<uint16_t>& ports,
    int rcvbuf_bytes,
    std::string& error)
{
    close();
    if (wake_fd < 0)
    {
        error = std::string("eventfd: ") + std::strerror(errno);
        return false;
    }
    std::vector<uint16_t> unique;
    for (uint16_t p : ports)
    {
        if (p == 0 ||
            std::find(unique.begin(), unique.end(), p) == unique.end())
        {
            unique.push_back(p);
        }
    }
    for (uint16_t p : unique)
    {
        auto* s = new Socket;
        sockets.push_back(s);
        s->fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (s->fd < 0)
        {
            error = std::string("socket: ") + std::strerror(errno);
            close();
            return false;
        }
        if (rcvbuf_bytes > 0)
        {
            ::setsockopt(
                s->fd,
                SOL_SOCKET,
                SO_RCVBUF,
                &rcvbuf_bytes,
                sizeof(rcvbuf_bytes));
        }
        int on = 1;
        ::setsockopt(s->fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(p);
        if (::bind(s->fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            error =
                "bind port " + std::to_string(p) + ": " + std::strerror(errno);
            close();
            return false;
        }
        socklen_t len = sizeof(addr);
        ::getsockname(s->fd, reinterpret_cast<sockaddr*>(&addr), &len);
        s->port = ntohs(addr.sin_port);
        bound_ports.push_back(s->port);

        int granted = 0;
        socklen_t glen = sizeof(granted);
        ::getsockopt(s->fd, SOL_SOCKET, SO_RCVBUF, &granted, &glen);
        rcvbuf_actual = granted / 2;  // Linux reports double the usable size

        s->storage.resize(batch * MAX_DATAGRAM_BYTES);
        s->msgs.resize(batch);
        s->iovs.resize(batch);
        s->controls.resize(batch);
    }
    return true;
}

void UdpReceiver::stop()
{
    stopped = true;
    if (wake_fd >= 0)
    {
        const uint64_t one = 1;
        [[maybe_unused]]
        auto r = ::write(wake_fd, &one, sizeof(one));
    }
}

int UdpReceiver::receive(std::vector<Datagram>& out, int timeout_ms)
{
    out.clear();
    if (stopped || wake_fd < 0 || sockets.empty())
    {
        return -1;
    }
    std::vector<pollfd> fds;
    fds.reserve(sockets.size() + 1);
    fds.push_back({wake_fd, POLLIN, 0});
    for (Socket* s : sockets)
    {
        fds.push_back({s->fd, POLLIN, 0});
    }

    int r = ::poll(fds.data(), fds.size(), timeout_ms);
    if (r < 0)
    {
        return errno == EINTR ? 0 : -1;
    }
    if (fds[0].revents & POLLIN)
    {
        return -1;  // stopped; the eventfd stays readable
    }
    if (r == 0)
    {
        return 0;
    }

    for (size_t i = 0; i < sockets.size(); i++)
    {
        if (!(fds[i + 1].revents & POLLIN))
        {
            continue;
        }
        Socket& s = *sockets[i];
        for (size_t k = 0; k < batch; k++)
        {
            s.iovs[k] = {
                s.storage.data() + k * MAX_DATAGRAM_BYTES,
                MAX_DATAGRAM_BYTES};
            msghdr& h = s.msgs[k].msg_hdr;
            h = msghdr{};
            h.msg_iov = &s.iovs[k];
            h.msg_iovlen = 1;
            h.msg_control = s.controls[k].data();
            h.msg_controllen = s.controls[k].size();
        }
        const int n = ::recvmmsg(
            s.fd,
            s.msgs.data(),
            static_cast<unsigned>(batch),
            MSG_DONTWAIT,
            nullptr);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                continue;
            }
            return -1;
        }
        for (int k = 0; k < n; k++)
        {
            msghdr& h = s.msgs[k].msg_hdr;
            if (h.msg_flags & MSG_TRUNC)
            {
                truncated++;
                continue;
            }
            Datagram d;
            d.data = {
                s.storage.data() + k * MAX_DATAGRAM_BYTES,
                s.msgs[k].msg_len};
            d.port = s.port;
            for (cmsghdr* c = CMSG_FIRSTHDR(&h); c; c = CMSG_NXTHDR(&h, c))
            {
                if (c->cmsg_level == SOL_SOCKET &&
                    c->cmsg_type == SCM_TIMESTAMPNS)
                {
                    timespec ts;
                    std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
                    d.recv_ns =
                        static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
                        static_cast<uint64_t>(ts.tv_nsec);
                }
            }
            if (d.recv_ns == 0)
            {
                timespec ts;
                ::clock_gettime(CLOCK_REALTIME, &ts);
                d.recv_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
                            static_cast<uint64_t>(ts.tv_nsec);
            }
            out.push_back(d);
        }
    }
    return static_cast<int>(out.size());
}

}  // namespace multiscan
