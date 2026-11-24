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

/*  Minimal CoLa A (ASCII SOPAS) client over TCP, as used by the multiScan136
 *  on port 2111 (firmware 2.3: port 2111 speaks only CoLa A, port 2112 only
 *  CoLa B).
 *
 *  Telegrams are <STX>text<ETX>. Requests are answered in order; event
 *  telegrams (sSN, sEA) can arrive at any time and are passed to the event
 *  handler, including while waiting for a reply. One request at a time: the
 *  sensor rejects overlapping requests (Sopas_Error_METHODIN_SERVER_BUSY).
 */
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>


namespace multiscan::sopas
{

struct Telegram
{
    std::string type;                    // sRA, sWA, sAN, sEA, sSN, sFA, ...
    std::string name;                    // variable/method name ("" for sFA)
    std::vector<std::string> arguments;  // space separated arguments
    std::string raw;                     // full text between STX and ETX
};

enum class Status
{
    Ok,
    NotConnected,
    Timeout,
    IoError,     // connection lost or socket error
    SopasError,  // sFA: see Result::sopas_error
    BadReply,    // reply doesn't answer the request
};

struct Result
{
    Status status{Status::NotConnected};
    Telegram reply;
    int sopas_error{0};  // Sopas_Error_* code when status == SopasError

    bool ok() const { return this->status == Status::Ok; }
    std::string describe() const;
};


/* Name of a Sopas_Error_* code (operating instructions, table 455). */
const char* sopasErrorName(int code);

/* Parse a CoLa A number: "+123" / "-5" are decimal, anything else is hex
 * (replies are hex; negative values are two's complement). */
std::optional<int64_t> parseNumber(const std::string& token);

/* Split "sRA Name a b c" into a Telegram (no validation). */
Telegram parseTelegram(const std::string& text);


class ColaClient
{
public:
    using EventHandler = std::function<void(const Telegram&)>;

public:
    ColaClient() = default;
    ~ColaClient();

    ColaClient(const ColaClient&) = delete;
    ColaClient& operator=(const ColaClient&) = delete;

public:
    bool connect(
        const std::string& host,
        uint16_t port,
        int timeout_ms,
        std::string& error);
    void disconnect();
    bool connected() const { return this->socket_fd >= 0; }

    /* IPv4 address of this end of the connection: the interface that
     * reaches the sensor ("" when not connected). */
    std::string localAddress() const;

    /* Send `telegram` (without STX/ETX) and wait for its reply. The reply
     * type must match the request (sRN->sRA, sWN->sWA, sMN->sAN,
     * sEN->sEA) and name the same variable, or be sFA. */
    Result request(const std::string& telegram, int timeout_ms);

    /* Process events that arrive within timeout_ms without a request.
     * Returns false if the connection was lost. */
    bool poll(int timeout_ms);

    void onEvent(EventHandler handler)
    {
        this->event_handler = std::move(handler);
    }

protected:
    enum class ReadResult
    {
        Telegram,
        Timeout,
        Closed
    };

protected:
    Status sendFrame(const std::string& telegram, int64_t deadline_ms);
    Result awaitReply(const Telegram& request, int64_t deadline_ms);
    ReadResult readTelegram(std::string& text, int64_t deadline_ms);
    /* Takes one framed telegram out of the receive buffer, if there is one. */
    bool takeTelegram(std::string& text);
    ReadResult receiveMore(int64_t deadline_ms);

protected:
    int socket_fd{-1};
    std::string receive_buffer;  // bytes received but not yet framed
    EventHandler event_handler;
};

}  // namespace multiscan::sopas
