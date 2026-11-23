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

#include "cola_client.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>


namespace multiscan::sopas
{

namespace
{
constexpr char STX = 0x02;
constexpr char ETX = 0x03;
constexpr size_t MAX_TELEGRAM_BYTES = 1 << 20;

int64_t nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

const char* replyTypeFor(const std::string& request_type)
{
    if (request_type == "sRN")
    {
        return "sRA";
    }
    if (request_type == "sWN")
    {
        return "sWA";
    }
    if (request_type == "sMN")
    {
        return "sAN";
    }
    if (request_type == "sEN")
    {
        return "sEA";
    }
    return nullptr;
}

bool isEvent(const Telegram& t)
{
    // sSN: event data; sEA without a pending sEN is a registration echo
    return t.type == "sSN";
}
}  // namespace


const char* sopasErrorName(int code)
{
    static constexpr std::array<const char*, 27> NAMES = {
        "Sopas_Ok",
        "Sopas_Error_METHODIN_ACCESSDENIED (wrong user level)",
        "Sopas_Error_METHODIN_UNKNOWNINDEX (unknown method)",
        "Sopas_Error_VARIABLE_UNKNOWNINDEX (unknown variable)",
        "Sopas_Error_LOCALCONDITIONFAILED (value out of range)",
        "Sopas_Error_INVALID_DATA",
        "Sopas_Error_UNKNOWN_ERROR",
        "Sopas_Error_BUFFER_OVERFLOW",
        "Sopas_Error_BUFFER_UNDERFLOW",
        "Sopas_Error_ERROR_UNKNOWN_TYPE",
        "Sopas_Error_VARIABLE_WRITE_ACCESSDENIED (not logged in or read-only)",
        "Sopas_Error_UNKNOWN_CMD_FOR_NAMESERVER",
        "Sopas_Error_UNKNOWN_COLA_COMMAND",
        "Sopas_Error_METHODIN_SERVER_BUSY",
        "Sopas_Error_FLEX_OUT_OF_BOUNDS",
        "Sopas_Error_EVENTREG_UNKNOWNINDEX",
        "Sopas_Error_COLA_A_VALUE_OVERFLOW",
        "Sopas_Error_COLA_A_INVALID_CHARACTER",
        "Sopas_Error_OSAI_NO_MESSAGE",
        "Sopas_Error_OSAI_NO_ANSWER_MESSAGE",
        "Sopas_Error_INTERNAL",
        "Sopas_Error_HubAddressCorrupted",
        "Sopas_Error_HubAddressDecoding",
        "Sopas_Error_HubAddressAddressExceeded",
        "Sopas_Error_HubAddressBlankExpected",
        "Sopas_Error_AsyncMethodsAreSuppressed",
        "Sopas_Error_ComplexArraysNotSupported",
    };
    return (code >= 0 && static_cast<size_t>(code) < NAMES.size())
               ? NAMES[code]
               : "unknown SOPAS error";
}

std::string Result::describe() const
{
    switch (status)
    {
        case Status::Ok:
            return "ok";
        case Status::NotConnected:
            return "not connected";
        case Status::Timeout:
            return "no reply (timeout)";
        case Status::IoError:
            return "connection lost";
        case Status::SopasError:
        {
            char code[16];
            std::snprintf(
                code,
                sizeof(code),
                "%X",
                static_cast<unsigned>(sopas_error));
            return std::string("sFA ") + code + ": " +
                   sopasErrorName(sopas_error);
        }
        case Status::BadReply:
            return "unexpected reply '" + reply.raw + "'";
    }
    return "?";
}

std::optional<int64_t> parseNumber(const std::string& t)
{
    if (t.empty())
    {
        return std::nullopt;
    }
    errno = 0;
    char* end = nullptr;
    int64_t v;
    if (t[0] == '+' || t[0] == '-')
    {
        v = std::strtoll(t.c_str(), &end, 10);
    }
    else
    {
        v = static_cast<int64_t>(std::strtoull(t.c_str(), &end, 16));
    }
    if (errno != 0 || end == t.c_str() || *end != '\0')
    {
        return std::nullopt;
    }
    return v;
}

Telegram parseTelegram(const std::string& text)
{
    Telegram t;
    t.raw = text;
    std::vector<std::string> tok;
    size_t i = 0;
    while (i < text.size())
    {
        while (i < text.size() && text[i] == ' ')
        {
            i++;
        }
        size_t j = i;
        while (j < text.size() && text[j] != ' ')
        {
            j++;
        }
        if (j > i)
        {
            tok.push_back(text.substr(i, j - i));
        }
        i = j;
    }
    if (!tok.empty())
    {
        t.type = tok[0];
    }
    size_t first_arg = 1;
    if (tok.size() > 1 && t.type != "sFA")
    {
        t.name = tok[1];
        first_arg = 2;
    }
    if (tok.size() > first_arg)
    {
        t.args.assign(tok.begin() + static_cast<long>(first_arg), tok.end());
    }
    return t;
}


ColaClient::~ColaClient() { disconnect(); }

void ColaClient::disconnect()
{
    if (fd >= 0)
    {
        ::close(fd);
        fd = -1;
    }
    rx.clear();
}

bool ColaClient::connect(
    const std::string& host,
    uint16_t port,
    int timeout_ms,
    std::string& error)
{
    disconnect();
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (int rc = ::getaddrinfo(
            host.c_str(),
            std::to_string(port).c_str(),
            &hints,
            &res);
        rc != 0 || !res)
    {
        error = "cannot resolve '" + host + "': " + gai_strerror(rc);
        return false;
    }
    fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0)
    {
        ::freeaddrinfo(res);
        error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    int rc = ::connect(fd, res->ai_addr, res->ai_addrlen);
    ::freeaddrinfo(res);
    if (rc < 0 && errno == EINPROGRESS)
    {
        pollfd p{fd, POLLOUT, 0};
        rc = ::poll(&p, 1, timeout_ms);
        if (rc == 0)
        {
            error = "connect to " + host + ":" + std::to_string(port) +
                    " timed out";
            disconnect();
            return false;
        }
        int so_error = 0;
        socklen_t len = sizeof(so_error);
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
        rc = so_error ? -1 : 0;
        errno = so_error;
    }
    if (rc < 0)
    {
        error = "connect to " + host + ":" + std::to_string(port) + ": " +
                std::strerror(errno);
        disconnect();
        return false;
    }
    int on = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
    return true;
}

std::string ColaClient::localAddress() const
{
    if (fd < 0)
    {
        return "";
    }
    sockaddr_in a{};
    socklen_t len = sizeof(a);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len) < 0)
    {
        return "";
    }
    char buf[INET_ADDRSTRLEN];
    return ::inet_ntop(AF_INET, &a.sin_addr, buf, sizeof(buf))
               ? std::string(buf)
               : std::string();
}

ColaClient::ReadResult ColaClient::readTelegram(
    std::string& out,
    int64_t deadline_ms)
{
    while (true)
    {
        // frame: skip anything before STX, take everything up to ETX
        const size_t stx = rx.find(STX);
        if (stx != std::string::npos)
        {
            const size_t etx = rx.find(ETX, stx + 1);
            if (etx != std::string::npos)
            {
                out = rx.substr(stx + 1, etx - stx - 1);
                rx.erase(0, etx + 1);
                return ReadResult::Telegram;
            }
            if (stx > 0)
            {
                rx.erase(0, stx);
            }
        }
        else
        {
            rx.clear();
        }
        if (rx.size() > MAX_TELEGRAM_BYTES)
        {
            rx.clear();  // garbage without ETX
        }

        const int64_t left = deadline_ms - nowMs();
        if (left <= 0)
        {
            return ReadResult::Timeout;
        }
        pollfd p{fd, POLLIN, 0};
        const int rc = ::poll(&p, 1, static_cast<int>(left));
        if (rc == 0)
        {
            return ReadResult::Timeout;
        }
        if (rc < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return ReadResult::Closed;
        }
        char buf[4096];
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n == 0)
        {
            return ReadResult::Closed;
        }
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                continue;
            }
            return ReadResult::Closed;
        }
        rx.append(buf, static_cast<size_t>(n));
    }
}

Result ColaClient::request(const std::string& telegram, int timeout_ms)
{
    Result r;
    if (fd < 0)
    {
        return r;
    }

    const std::string frame =
        std::string(1, STX) + telegram + std::string(1, ETX);
    size_t sent = 0;
    const int64_t deadline = nowMs() + timeout_ms;
    while (sent < frame.size())
    {
        const ssize_t n =
            ::send(fd, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                pollfd p{fd, POLLOUT, 0};
                if (::poll(
                        &p,
                        1,
                        static_cast<int>(
                            std::max<int64_t>(0, deadline - nowMs()))) <= 0)
                {
                    r.status = Status::Timeout;
                    return r;
                }
                continue;
            }
            disconnect();
            r.status = Status::IoError;
            return r;
        }
        sent += static_cast<size_t>(n);
    }

    const Telegram req = parseTelegram(telegram);
    const char* want = replyTypeFor(req.type);
    std::string text;
    while (true)
    {
        switch (readTelegram(text, deadline))
        {
            case ReadResult::Timeout:
                r.status = Status::Timeout;
                return r;
            case ReadResult::Closed:
                disconnect();
                r.status = Status::IoError;
                return r;
            case ReadResult::Telegram:
                break;
        }
        Telegram t = parseTelegram(text);
        if (isEvent(t))
        {
            if (event_handler)
            {
                event_handler(t);
            }
            continue;
        }
        r.reply = std::move(t);
        if (r.reply.type == "sFA")
        {
            r.status = Status::SopasError;
            r.sopas_error =
                r.reply.args.empty()
                    ? -1
                    : static_cast<int>(
                          parseNumber(r.reply.args[0]).value_or(-1));
            return r;
        }
        if (want && r.reply.type == want && r.reply.name == req.name)
        {
            r.status = Status::Ok;
            return r;
        }
        if (r.reply.type == "sRA" || r.reply.type == "sWA" ||
            r.reply.type == "sAN" || r.reply.type == "sEA")
        {
            continue;  // stale reply to an earlier request that timed out
        }
        r.status = Status::BadReply;
        return r;
    }
}

bool ColaClient::poll(int timeout_ms)
{
    if (fd < 0)
    {
        return false;
    }
    const int64_t deadline = nowMs() + timeout_ms;
    std::string text;
    while (true)
    {
        switch (readTelegram(text, deadline))
        {
            case ReadResult::Timeout:
                return true;
            case ReadResult::Closed:
                disconnect();
                return false;
            case ReadResult::Telegram:
                break;
        }
        const Telegram t = parseTelegram(text);
        if (event_handler)
        {
            event_handler(t);
        }
    }
}

}  // namespace multiscan::sopas
