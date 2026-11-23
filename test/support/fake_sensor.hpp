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

    explicit FakeSensor(const std::string& transcript_path)
    {
        loadTranscript(transcript_path);
        listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        int on = 1;
        ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = 0;
        ::bind(listen_fd, reinterpret_cast<sockaddr*>(&a), sizeof(a));
        socklen_t len = sizeof(a);
        ::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&a), &len);
        listen_port = ntohs(a.sin_port);
        ::listen(listen_fd, 4);
        server = std::thread([this] { serve(); });
    }

    ~FakeSensor()
    {
        running = false;
        ::shutdown(listen_fd, SHUT_RDWR);
        server.join();
        ::close(listen_fd);
    }

    uint16_t port() const { return listen_port; }

    // --- fault injection (set before connecting) ---
    std::string password = "F4724744";
    std::map<std::string, int> fail_writes;        // variable -> sFA code
    std::set<std::string> ignore_writes;           // accepted but not stored
    std::vector<std::string> events_before_reply;  // sent before every reply
    bool byte_by_byte = false;
    bool stale_reply_first =
        false;  // send a reply to "sRN Stale" before the first reply
    size_t close_after =
        SIZE_MAX;  // drop the connection after this many requests
    std::set<std::string> no_reply;  // telegrams (exact) never answered

    // --- inspection ---
    std::vector<Request> requests()
    {
        std::lock_guard l(mtx);
        return log;
    }
    std::string value(const std::string& var)
    {
        std::lock_guard l(mtx);
        return vars.count(var) ? vars[var] : std::string("<unset>");
    }
    void setValue(const std::string& var, const std::string& v)
    {
        std::lock_guard l(mtx);
        vars[var] = v;
    }

    /* Device formatting of a written argument list. */
    static std::string toDeviceFormat(const std::string& args)
    {
        std::istringstream in(args);
        std::string tok, out;
        while (in >> tok)
        {
            if (tok[0] == '+' || tok[0] == '-')
            {
                const uint32_t v = static_cast<uint32_t>(std::stoll(tok));
                char buf[16];
                std::snprintf(buf, sizeof(buf), "%X", v);
                tok = buf;
            }
            out += (out.empty() ? "" : " ") + tok;
        }
        return out;
    }

private:
    void loadTranscript(const std::string& path)
    {
        std::ifstream f(path);
        std::string line;
        while (std::getline(f, line))
        {
            if (line.rfind("< sRA ", 0) != 0)
            {
                continue;
            }
            const std::string rest = line.substr(6);
            const size_t sp = rest.find(' ');
            const std::string name = rest.substr(0, sp);
            if (!vars.count(name))
            {
                vars[name] = sp == std::string::npos ? "" : rest.substr(sp + 1);
            }
        }
        vars["DevSta"] = "3";
    }

    void serve()
    {
        while (running)
        {
            pollfd p{listen_fd, POLLIN, 0};
            if (::poll(&p, 1, 50) <= 0)
            {
                continue;
            }
            const int fd = ::accept(listen_fd, nullptr, nullptr);
            if (fd < 0)
            {
                continue;
            }
            int on = 1;  // events and replies are separate small sends
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
            session(fd);
            ::close(fd);
        }
    }

    void sendTelegram(int fd, const std::string& text)
    {
        const std::string frame = "\x02" + text + "\x03";
        if (byte_by_byte)
        {
            for (char c : frame)
            {
                ::send(fd, &c, 1, MSG_NOSIGNAL);
            }
        }
        else
        {
            ::send(fd, frame.data(), frame.size(), MSG_NOSIGNAL);
        }
    }

    void session(int fd)
    {
        bool logged_in = false;
        bool stale_sent = false;
        size_t n_requests = 0;
        std::string rx;
        while (running)
        {
            pollfd p{fd, POLLIN, 0};
            if (::poll(&p, 1, 50) <= 0)
            {
                continue;
            }
            char buf[1024];
            const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
            if (n <= 0)
            {
                return;
            }
            rx.append(buf, static_cast<size_t>(n));
            size_t etx;
            while ((etx = rx.find('\x03')) != std::string::npos)
            {
                const size_t stx = rx.find('\x02');
                const std::string req = rx.substr(stx + 1, etx - stx - 1);
                rx.erase(0, etx + 1);
                {
                    std::lock_guard l(mtx);
                    log.push_back({req, logged_in});
                }
                if (++n_requests > close_after)
                {
                    return;
                }
                if (no_reply.count(req))
                {
                    continue;
                }
                for (const auto& e : events_before_reply)
                {
                    sendTelegram(fd, e);
                }
                if (stale_reply_first && !stale_sent)
                {
                    sendTelegram(fd, "sRA Stale 1");
                    stale_sent = true;
                }
                sendTelegram(fd, handle(req, logged_in));
            }
        }
    }

    std::string handle(const std::string& req, bool& logged_in)
    {
        std::istringstream in(req);
        std::string type, name, args;
        in >> type >> name;
        std::getline(in, args);
        if (!args.empty() && args[0] == ' ')
        {
            args.erase(0, 1);
        }

        std::lock_guard l(mtx);
        if (type == "sMN")
        {
            if (name == "SetAccessMode")
            {
                logged_in = (args == "3 " + password);
                return std::string("sAN SetAccessMode ") +
                       (logged_in ? "1" : "0");
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
                interlaced = (args == "1");
                return "sAN mCLsetscancfglist 0";
            }
            return "sFA 2";
        }
        if (type == "sRN")
        {
            if (!vars.count(name))
            {
                return "sFA 3";
            }
            if (name == "DevSta" && logged_in)
            {
                return "sRA DevSta 2";  // service mode while logged in
            }
            return "sRA " + name + (vars[name].empty() ? "" : " " + vars[name]);
        }
        if (type == "sWN")
        {
            if (!vars.count(name))
            {
                return "sFA 3";
            }
            if (!logged_in)
            {
                return "sFA A";
            }
            if (fail_writes.count(name))
            {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "%X", fail_writes[name]);
                return std::string("sFA ") + buf;
            }
            if (!ignore_writes.count(name))
            {
                vars[name] = toDeviceFormat(args);
            }
            return "sWA " + name;
        }
        return "sFA C";
    }

    int listen_fd = -1;
    uint16_t listen_port = 0;
    std::thread server;
    std::atomic<bool> running{true};
    std::mutex mtx;
    std::map<std::string, std::string> vars;
    std::vector<Request> log;

public:
    bool interlaced = false;
};

}  // namespace ms_test
