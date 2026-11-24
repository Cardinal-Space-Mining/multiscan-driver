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

/*  Receives UDP datagrams on one or more ports (Linux).
 *
 *  - batched reads (recvmmsg) into preallocated buffers
 *  - kernel receive timestamps (SO_TIMESTAMPNS, CLOCK_REALTIME)
 *  - large receive buffers (SO_RCVBUF, capped by net.core.rmem_max)
 *  - stop() wakes a blocked receive() immediately from any thread; it is
 *    final, receive() returns -1 from then on
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>


namespace multiscan
{

class UdpReceiver
{
public:
    struct Datagram
    {
        std::span<const uint8_t> data;  // valid until the next receive()
        uint64_t receive_time_ns{0};    // kernel receive time, CLOCK_REALTIME
        uint16_t port{0};               // local port it arrived on
    };

    static constexpr size_t MAX_DATAGRAM_BYTES = 65536;

public:
    explicit UdpReceiver(size_t batch_size = 16);
    ~UdpReceiver();

    UdpReceiver(const UdpReceiver&) = delete;
    UdpReceiver& operator=(const UdpReceiver&) = delete;

public:
    /* Bind 0.0.0.0:port for each port (duplicates ignored, 0 = any free
     * port). Returns false and sets `error` on failure. */
    bool open(
        const std::vector<uint16_t>& ports,
        int receive_buffer_bytes,
        std::string& error);
    void close();

    /* Wait up to timeout_ms for data and return what is available (at most
     * batch_size datagrams per socket). Returns the number of datagrams,
     * 0 on timeout, -1 once stop() was called or on a socket error. */
    int receive(std::vector<Datagram>& out, int timeout_ms);

    /* Wake receive() and make it (and every later call) return -1.
     * Safe to call from any thread at any time. */
    void stop();

public:
    const std::vector<uint16_t>& ports() const { return this->bound_ports; }
    /* Receive buffer size the kernel granted (per socket, bytes). */
    int receiveBufferBytes() const { return this->granted_buffer_bytes; }
    uint64_t truncatedDatagrams() const { return this->truncated_datagrams; }

protected:
    struct Socket;

protected:
    bool
        openSocket(uint16_t port, int receive_buffer_bytes, std::string& error);
    /* Reads what one socket has; false on a socket error. */
    bool readSocket(Socket& socket, std::vector<Datagram>& out);

protected:
    std::vector<std::unique_ptr<Socket>> sockets;
    std::vector<uint16_t> bound_ports;
    size_t batch_size;
    int wake_event_fd{-1};  // created once, lives as long as the object
    std::atomic<bool> stopped{false};
    int granted_buffer_bytes{0};
    uint64_t truncated_datagrams{0};
};

}  // namespace multiscan
