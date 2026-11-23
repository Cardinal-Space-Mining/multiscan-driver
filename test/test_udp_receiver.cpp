/*  UdpReceiver on the loopback interface. */
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "net/udp_receiver.hpp"

using multiscan::UdpReceiver;

namespace
{

void sendTo(uint16_t port, const std::vector<uint8_t>& payload)
{
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(::sendto(fd, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)),
              static_cast<ssize_t>(payload.size()));
    ::close(fd);
}

std::vector<uint8_t> pattern(size_t n, uint8_t seed)
{
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; i++) v[i] = static_cast<uint8_t>(i * 31 + seed);
    return v;
}

uint64_t realtimeNs()
{
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace


TEST(UdpReceiver, ReceivesBatchesFromSeveralPortsWithTimestamps)
{
    UdpReceiver r;
    std::string err;
    ASSERT_TRUE(r.open({0, 0}, 1 << 20, err)) << err;
    ASSERT_EQ(r.ports().size(), 2u);
    const uint16_t scan_port = r.ports()[0], imu_port = r.ports()[1];

    const uint64_t t0 = realtimeNs();
    std::vector<std::vector<uint8_t>> scans;
    for (uint8_t i = 0; i < 10; i++)
    {
        scans.push_back(pattern(14160, i));  // the largest real segment
        sendTo(scan_port, scans.back());
    }
    sendTo(imu_port, pattern(64, 99));
    const uint64_t t1 = realtimeNs();

    std::vector<UdpReceiver::Datagram> batch;
    std::vector<std::vector<uint8_t>> got_scan, got_imu;
    for (int tries = 0; tries < 20 && got_scan.size() + got_imu.size() < 11; tries++)
    {
        ASSERT_GE(r.receive(batch, 100), 0);
        for (const auto& d : batch)
        {
            EXPECT_GE(d.recv_ns, t0);
            EXPECT_LE(d.recv_ns, t1 + 1'000'000);
            (d.port == scan_port ? got_scan : got_imu).emplace_back(d.data.begin(), d.data.end());
        }
    }
    EXPECT_EQ(got_scan, scans);
    ASSERT_EQ(got_imu.size(), 1u);
    EXPECT_EQ(got_imu[0], pattern(64, 99));
    EXPECT_GT(r.receiveBufferBytes(), 0);
}

TEST(UdpReceiver, TimesOutWithoutData)
{
    UdpReceiver r;
    std::string err;
    ASSERT_TRUE(r.open({0}, 0, err)) << err;
    std::vector<UdpReceiver::Datagram> batch;
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_EQ(r.receive(batch, 30), 0);
    EXPECT_GE(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(25));
    EXPECT_TRUE(batch.empty());
}

TEST(UdpReceiver, StopWakesABlockedReceiveImmediately)
{
    UdpReceiver r;
    std::string err;
    ASSERT_TRUE(r.open({0}, 0, err)) << err;
    std::atomic<int> result{1};
    std::chrono::steady_clock::time_point returned;
    std::thread t([&] {
        std::vector<UdpReceiver::Datagram> batch;
        result = r.receive(batch, 10'000);
        returned = std::chrono::steady_clock::now();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto stopped = std::chrono::steady_clock::now();
    r.stop();
    t.join();
    EXPECT_EQ(result, -1);
    EXPECT_LT(returned - stopped, std::chrono::milliseconds(50));
    std::vector<UdpReceiver::Datagram> batch;
    EXPECT_EQ(r.receive(batch, 0), -1) << "stop is final";
}

TEST(UdpReceiver, StopBeforeOpenStillStops)
{
    UdpReceiver r;
    r.stop();
    std::string err;
    ASSERT_TRUE(r.open({0}, 0, err)) << err;
    std::vector<UdpReceiver::Datagram> batch;
    EXPECT_EQ(r.receive(batch, 1000), -1);
}

TEST(UdpReceiver, ReportsBindFailures)
{
    UdpReceiver a, b;
    std::string err;
    ASSERT_TRUE(a.open({0}, 0, err)) << err;
    EXPECT_FALSE(b.open({a.ports()[0]}, 0, err));
    EXPECT_NE(err.find("bind"), std::string::npos);
}

TEST(UdpReceiver, SamePortTwiceOpensOneSocket)
{
    UdpReceiver a;
    std::string err;
    ASSERT_TRUE(a.open({0}, 0, err)) << err;
    const uint16_t p = a.ports()[0];
    a.close();
    UdpReceiver b;
    ASSERT_TRUE(b.open({p, p}, 0, err)) << err;
    EXPECT_EQ(b.ports().size(), 1u);
}
