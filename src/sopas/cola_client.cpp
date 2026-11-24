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

bool isEvent(const Telegram& telegram)
{
    // sSN: event data; sEA without a pending sEN is a registration echo
    return telegram.type == "sSN";
}

bool isAnyReply(const Telegram& telegram)
{
    return telegram.type == "sRA" || telegram.type == "sWA" ||
           telegram.type == "sAN" || telegram.type == "sEA";
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
    switch (this->status)
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
                static_cast<unsigned>(this->sopas_error));
            return std::string("sFA ") + code + ": " +
                   sopasErrorName(this->sopas_error);
        }
        case Status::BadReply:
            return "unexpected reply '" + this->reply.raw + "'";
    }
    return "?";
}

std::optional<int64_t> parseNumber(const std::string& token)
{
    if (token.empty())
    {
        return std::nullopt;
    }
    errno = 0;
    char* end = nullptr;
    int64_t value;
    if (token[0] == '+' || token[0] == '-')
    {
        value = std::strtoll(token.c_str(), &end, 10);
    }
    else
    {
        value = static_cast<int64_t>(std::strtoull(token.c_str(), &end, 16));
    }
    if (errno != 0 || end == token.c_str() || *end != '\0')
    {
        return std::nullopt;
    }
    return value;
}

Telegram parseTelegram(const std::string& text)
{
    Telegram telegram;
    telegram.raw = text;
    std::vector<std::string> tokens;
    size_t position = 0;
    while (position < text.size())
    {
        while (position < text.size() && text[position] == ' ')
        {
            position++;
        }
        size_t token_end = position;
        while (token_end < text.size() && text[token_end] != ' ')
        {
            token_end++;
        }
        if (token_end > position)
        {
            tokens.push_back(text.substr(position, token_end - position));
        }
        position = token_end;
    }
    if (!tokens.empty())
    {
        telegram.type = tokens[0];
    }
    size_t first_argument = 1;
    if (tokens.size() > 1 && telegram.type != "sFA")
    {
        telegram.name = tokens[1];
        first_argument = 2;
    }
    if (tokens.size() > first_argument)
    {
        telegram.arguments.assign(
            tokens.begin() + static_cast<long>(first_argument),
            tokens.end());
    }
    return telegram;
}



// --- Connection --------------------------------------------------------------

ColaClient::~ColaClient() { this->disconnect(); }

void ColaClient::disconnect()
{
    if (this->socket_fd >= 0)
    {
        ::close(this->socket_fd);
        this->socket_fd = -1;
    }
    this->receive_buffer.clear();
}

bool ColaClient::connect(
    const std::string& host,
    uint16_t port,
    int timeout_ms,
    std::string& error)
{
    this->disconnect();
    const std::string endpoint = host + ":" + std::to_string(port);

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* address = nullptr;
    if (const int status = ::getaddrinfo(
            host.c_str(),
            std::to_string(port).c_str(),
            &hints,
            &address);
        status != 0 || !address)
    {
        error = "cannot resolve '" + host + "': " + gai_strerror(status);
        return false;
    }
    this->socket_fd =
        ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (this->socket_fd < 0)
    {
        ::freeaddrinfo(address);
        error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    int status =
        ::connect(this->socket_fd, address->ai_addr, address->ai_addrlen);
    ::freeaddrinfo(address);

    // non-blocking: wait for the connection to complete
    if (status < 0 && errno == EINPROGRESS)
    {
        pollfd writable{this->socket_fd, POLLOUT, 0};
        if (::poll(&writable, 1, timeout_ms) == 0)
        {
            error = "connect to " + endpoint + " timed out";
            this->disconnect();
            return false;
        }
        int socket_error = 0;
        socklen_t socket_error_size = sizeof(socket_error);
        ::getsockopt(
            this->socket_fd,
            SOL_SOCKET,
            SO_ERROR,
            &socket_error,
            &socket_error_size);
        status = socket_error ? -1 : 0;
        errno = socket_error;
    }
    if (status < 0)
    {
        error = "connect to " + endpoint + ": " + std::strerror(errno);
        this->disconnect();
        return false;
    }
    const int enable = 1;
    ::setsockopt(
        this->socket_fd,
        IPPROTO_TCP,
        TCP_NODELAY,
        &enable,
        sizeof(enable));
    return true;
}

std::string ColaClient::localAddress() const
{
    if (this->socket_fd < 0)
    {
        return "";
    }
    sockaddr_in address{};
    socklen_t address_size = sizeof(address);
    if (::getsockname(
            this->socket_fd,
            reinterpret_cast<sockaddr*>(&address),
            &address_size) < 0)
    {
        return "";
    }
    char text[INET_ADDRSTRLEN];
    return ::inet_ntop(AF_INET, &address.sin_addr, text, sizeof(text))
               ? std::string(text)
               : std::string();
}



// --- Requests and events -----------------------------------------------------

Result ColaClient::request(const std::string& telegram, int timeout_ms)
{
    Result result;
    if (this->socket_fd < 0)
    {
        return result;
    }
    const int64_t deadline_ms = nowMs() + timeout_ms;
    result.status = this->sendFrame(telegram, deadline_ms);
    if (result.status != Status::Ok)
    {
        return result;
    }
    return this->awaitReply(parseTelegram(telegram), deadline_ms);
}

bool ColaClient::poll(int timeout_ms)
{
    if (this->socket_fd < 0)
    {
        return false;
    }
    const int64_t deadline_ms = nowMs() + timeout_ms;
    std::string text;
    while (true)
    {
        switch (this->readTelegram(text, deadline_ms))
        {
            case ReadResult::Timeout:
                return true;
            case ReadResult::Closed:
                this->disconnect();
                return false;
            case ReadResult::Telegram:
                break;
        }
        const Telegram event = parseTelegram(text);
        if (this->event_handler)
        {
            this->event_handler(event);
        }
    }
}

/* Sends <STX>telegram<ETX>. */
Status ColaClient::sendFrame(const std::string& telegram, int64_t deadline_ms)
{
    const std::string frame =
        std::string(1, STX) + telegram + std::string(1, ETX);
    size_t sent = 0;
    while (sent < frame.size())
    {
        const ssize_t written = ::send(
            this->socket_fd,
            frame.data() + sent,
            frame.size() - sent,
            MSG_NOSIGNAL);
        if (written >= 0)
        {
            sent += static_cast<size_t>(written);
            continue;
        }
        if (errno != EAGAIN && errno != EINTR)
        {
            this->disconnect();
            return Status::IoError;
        }
        pollfd writable{this->socket_fd, POLLOUT, 0};
        const int64_t remaining_ms =
            std::max<int64_t>(0, deadline_ms - nowMs());
        if (::poll(&writable, 1, static_cast<int>(remaining_ms)) <= 0)
        {
            return Status::Timeout;
        }
    }
    return Status::Ok;
}

/* Reads until the reply to `request` arrives; events on the way go to the
 * event handler, stale replies to earlier requests are skipped. */
Result ColaClient::awaitReply(const Telegram& request, int64_t deadline_ms)
{
    Result result;
    const char* expected_type = replyTypeFor(request.type);
    std::string text;
    while (true)
    {
        switch (this->readTelegram(text, deadline_ms))
        {
            case ReadResult::Timeout:
                result.status = Status::Timeout;
                return result;
            case ReadResult::Closed:
                this->disconnect();
                result.status = Status::IoError;
                return result;
            case ReadResult::Telegram:
                break;
        }
        Telegram reply = parseTelegram(text);
        if (isEvent(reply))
        {
            if (this->event_handler)
            {
                this->event_handler(reply);
            }
            continue;
        }

        result.reply = std::move(reply);
        if (result.reply.type == "sFA")
        {
            result.status = Status::SopasError;
            result.sopas_error =
                result.reply.arguments.empty()
                    ? -1
                    : static_cast<int>(
                          parseNumber(result.reply.arguments[0]).value_or(-1));
            return result;
        }
        if (expected_type && result.reply.type == expected_type &&
            result.reply.name == request.name)
        {
            result.status = Status::Ok;
            return result;
        }
        if (isAnyReply(result.reply))
        {
            continue;  // stale reply to an earlier request that timed out
        }
        result.status = Status::BadReply;
        return result;
    }
}



// --- Framing -----------------------------------------------------------------

ColaClient::ReadResult ColaClient::readTelegram(
    std::string& text,
    int64_t deadline_ms)
{
    while (!this->takeTelegram(text))
    {
        const ReadResult received = this->receiveMore(deadline_ms);
        if (received != ReadResult::Telegram)
        {
            return received;
        }
    }
    return ReadResult::Telegram;
}

/* Skips anything before STX and takes everything up to ETX. */
bool ColaClient::takeTelegram(std::string& text)
{
    std::string& buffer = this->receive_buffer;
    const size_t start = buffer.find(STX);
    if (start == std::string::npos)
    {
        buffer.clear();
        return false;
    }
    const size_t end = buffer.find(ETX, start + 1);
    if (end != std::string::npos)
    {
        text = buffer.substr(start + 1, end - start - 1);
        buffer.erase(0, end + 1);
        return true;
    }
    if (start > 0)
    {
        buffer.erase(0, start);
    }
    if (buffer.size() > MAX_TELEGRAM_BYTES)
    {
        buffer.clear();  // garbage without ETX
    }
    return false;
}

/* Waits for more bytes. ReadResult::Telegram here means "got some". */
ColaClient::ReadResult ColaClient::receiveMore(int64_t deadline_ms)
{
    while (true)
    {
        const int64_t remaining_ms = deadline_ms - nowMs();
        if (remaining_ms <= 0)
        {
            return ReadResult::Timeout;
        }
        pollfd readable{this->socket_fd, POLLIN, 0};
        const int ready = ::poll(&readable, 1, static_cast<int>(remaining_ms));
        if (ready == 0)
        {
            return ReadResult::Timeout;
        }
        if (ready < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return ReadResult::Closed;
        }
        char chunk[4096];
        const ssize_t received =
            ::recv(this->socket_fd, chunk, sizeof(chunk), 0);
        if (received == 0)
        {
            return ReadResult::Closed;
        }
        if (received < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                continue;
            }
            return ReadResult::Closed;
        }
        this->receive_buffer.append(chunk, static_cast<size_t>(received));
        return ReadResult::Telegram;
    }
}

}  // namespace multiscan::sopas
