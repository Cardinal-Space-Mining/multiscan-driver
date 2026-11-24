/*  In-process simulation of the multiScan136 CoLa A interface for tests.
 *
 *  Behaves like firmware 2.3.0 as recorded in
 *  test/data/sopas_transcript_fw2.3.txt: variables start with the recorded
 *  values; sWN needs a login (else sFA A), unknown variables give sFA 3,
 *  writes are echoed back by sRN the way the device prints them (hex, negative
 *  values as 32-bit two's complement), sMN Run logs out, DevSta reads 2
 *  (service mode) while logged in.
 *
 *  Faults can be injected: rejected or ignored writes, events before
 *  replies, byte-by-byte sending, stale replies, dropped connections.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>


namespace ms_test
{

class FakeSensor
{
public:
    struct Request
    {
        std::string telegram;
        bool logged_in;  // login state when it arrived
    };

public:
    explicit FakeSensor(const std::string& transcript_path)
    {
        this->loadTranscript(transcript_path);
        this->listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        const int enable = 1;
        ::setsockopt(
            this->listen_fd,
            SOL_SOCKET,
            SO_REUSEADDR,
            &enable,
            sizeof(enable));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;  // any free port
        ::bind(
            this->listen_fd,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address));
        socklen_t address_size = sizeof(address);
        ::getsockname(
            this->listen_fd,
            reinterpret_cast<sockaddr*>(&address),
            &address_size);
        this->listen_port = ntohs(address.sin_port);
        ::listen(this->listen_fd, 4);
        this->server_thread = std::thread([this] { this->serve(); });
    }

    ~FakeSensor()
    {
        this->running = false;
        ::shutdown(this->listen_fd, SHUT_RDWR);
        this->server_thread.join();
        ::close(this->listen_fd);
    }

public:
    uint16_t port() const { return this->listen_port; }

    // --- Inspection ----------------------------------------------------------
    std::vector<Request> requests()
    {
        std::lock_guard lock{this->state_mutex};
        return this->request_log;
    }
    std::string value(const std::string& variable)
    {
        std::lock_guard lock{this->state_mutex};
        return this->variables.count(variable) ? this->variables[variable]
                                               : std::string("<unset>");
    }
    void setValue(const std::string& variable, const std::string& value)
    {
        std::lock_guard lock{this->state_mutex};
        this->variables[variable] = value;
    }

    /* Device formatting of a written argument list: signed decimal numbers
     * come back as hex (negative ones as 32-bit two's complement). */
    static std::string toDeviceFormat(const std::string& arguments)
    {
        std::istringstream in(arguments);
        std::string token;
        std::string formatted;
        while (in >> token)
        {
            if (token[0] == '+' || token[0] == '-')
            {
                const uint32_t number =
                    static_cast<uint32_t>(std::stoll(token));
                char hex[16];
                std::snprintf(hex, sizeof(hex), "%X", number);
                token = hex;
            }
            formatted += (formatted.empty() ? "" : " ") + token;
        }
        return formatted;
    }

public:
    // --- Fault injection (set before connecting) -----------------------------
    std::string password{"F4724744"};
    std::map<std::string, int> fail_writes;        // variable -> sFA code
    std::set<std::string> ignore_writes;           // accepted but not stored
    std::vector<std::string> events_before_reply;  // sent before every reply
    bool byte_by_byte{false};
    /* Send a reply to "sRN Stale" before the first reply. */
    bool stale_reply_first{false};
    /* Drop the connection after this many requests. */
    size_t close_after{SIZE_MAX};
    std::set<std::string> no_reply;  // telegrams (exact) never answered

    // --- State the tests check -----------------------------------------------
    bool interlaced{false};  // set by sMN mCLsetscancfglist

protected:
    /* The variables start with the values the real sensor replied. */
    void loadTranscript(const std::string& path)
    {
        std::ifstream transcript(path);
        std::string line;
        while (std::getline(transcript, line))
        {
            if (line.rfind("< sRA ", 0) != 0)
            {
                continue;
            }
            const std::string reply = line.substr(6);
            const size_t space = reply.find(' ');
            const std::string name = reply.substr(0, space);
            if (!this->variables.count(name))
            {
                this->variables[name] =
                    space == std::string::npos ? "" : reply.substr(space + 1);
            }
        }
        this->variables["DevSta"] = "3";
    }

    void serve()
    {
        while (this->running)
        {
            pollfd readable{this->listen_fd, POLLIN, 0};
            if (::poll(&readable, 1, 50) <= 0)
            {
                continue;
            }
            const int client_fd = ::accept(this->listen_fd, nullptr, nullptr);
            if (client_fd < 0)
            {
                continue;
            }
            const int enable =
                1;  // events and replies are separate small sends
            ::setsockopt(
                client_fd,
                IPPROTO_TCP,
                TCP_NODELAY,
                &enable,
                sizeof(enable));
            this->session(client_fd);
            ::close(client_fd);
        }
    }

    void sendTelegram(int client_fd, const std::string& text)
    {
        const std::string frame = "\x02" + text + "\x03";
        if (!this->byte_by_byte)
        {
            ::send(client_fd, frame.data(), frame.size(), MSG_NOSIGNAL);
            return;
        }
        for (const char byte : frame)
        {
            ::send(client_fd, &byte, 1, MSG_NOSIGNAL);
        }
    }

    /* One client connection: frames requests and answers them. */
    void session(int client_fd)
    {
        bool logged_in = false;
        bool stale_sent = false;
        size_t requests_received = 0;
        std::string received;
        while (this->running)
        {
            pollfd readable{client_fd, POLLIN, 0};
            if (::poll(&readable, 1, 50) <= 0)
            {
                continue;
            }
            char chunk[1024];
            const ssize_t chunk_size =
                ::recv(client_fd, chunk, sizeof(chunk), 0);
            if (chunk_size <= 0)
            {
                return;
            }
            received.append(chunk, static_cast<size_t>(chunk_size));

            size_t end;
            while ((end = received.find('\x03')) != std::string::npos)
            {
                const size_t start = received.find('\x02');
                const std::string request =
                    received.substr(start + 1, end - start - 1);
                received.erase(0, end + 1);
                {
                    std::lock_guard lock{this->state_mutex};
                    this->request_log.push_back({request, logged_in});
                }
                if (++requests_received > this->close_after)
                {
                    return;
                }
                if (this->no_reply.count(request))
                {
                    continue;
                }
                for (const std::string& event : this->events_before_reply)
                {
                    this->sendTelegram(client_fd, event);
                }
                if (this->stale_reply_first && !stale_sent)
                {
                    this->sendTelegram(client_fd, "sRA Stale 1");
                    stale_sent = true;
                }
                this->sendTelegram(client_fd, this->handle(request, logged_in));
            }
        }
    }

    std::string handle(const std::string& request, bool& logged_in)
    {
        std::istringstream in(request);
        std::string type;
        std::string name;
        std::string arguments;
        in >> type >> name;
        std::getline(in, arguments);
        if (!arguments.empty() && arguments[0] == ' ')
        {
            arguments.erase(0, 1);
        }

        std::lock_guard lock{this->state_mutex};
        if (type == "sMN")
        {
            return this->handleMethod(name, arguments, logged_in);
        }
        if (type == "sRN")
        {
            return this->handleRead(name, logged_in);
        }
        if (type == "sWN")
        {
            return this->handleWrite(name, arguments, logged_in);
        }
        return "sFA C";
    }

    std::string handleMethod(
        const std::string& name,
        const std::string& arguments,
        bool& logged_in)
    {
        if (name == "SetAccessMode")
        {
            logged_in = (arguments == "3 " + this->password);
            return std::string("sAN SetAccessMode ") + (logged_in ? "1" : "0");
        }
        if (name == "Run")
        {
            logged_in = false;
            return "sAN Run 1";
        }
        if (!logged_in)
        {
            return "sFA 1";
        }
        if (name == "LMCstartmeas")
        {
            return "sAN LMCstartmeas 0";
        }
        if (name == "mCLsetscancfglist")
        {
            this->interlaced = (arguments == "1");
            return "sAN mCLsetscancfglist 0";
        }
        return "sFA 2";
    }

    std::string handleRead(const std::string& name, bool logged_in)
    {
        if (!this->variables.count(name))
        {
            return "sFA 3";
        }
        if (name == "DevSta" && logged_in)
        {
            return "sRA DevSta 2";  // service mode while logged in
        }
        const std::string& value = this->variables[name];
        return "sRA " + name + (value.empty() ? "" : " " + value);
    }

    std::string handleWrite(
        const std::string& name,
        const std::string& arguments,
        bool logged_in)
    {
        if (!this->variables.count(name))
        {
            return "sFA 3";
        }
        if (!logged_in)
        {
            return "sFA A";
        }
        if (this->fail_writes.count(name))
        {
            char code[8];
            std::snprintf(code, sizeof(code), "%X", this->fail_writes[name]);
            return std::string("sFA ") + code;
        }
        if (!this->ignore_writes.count(name))
        {
            this->variables[name] = toDeviceFormat(arguments);
        }
        return "sWA " + name;
    }

protected:
    int listen_fd{-1};
    uint16_t listen_port{0};
    std::thread server_thread;
    std::atomic<bool> running{true};

    std::mutex state_mutex;
    std::map<std::string, std::string> variables;  // guarded by state_mutex
    std::vector<Request> request_log;              // guarded by state_mutex
};

}  // namespace ms_test
