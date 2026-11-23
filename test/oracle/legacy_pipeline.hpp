/*  Test oracle: the data path of multiscan_driver at commit 184806e, minus the
 *  sockets and ROS. recvSegmentData() validation, parseSegment(), addSegment()
 *  and publishScan() are copied from src/multiscan_driver.cpp with only the
 *  ROS types replaced, and run on the vendored sick_scan_xd parser.
 *
 *  The rewrite is checked against this. Remove it together with
 *  src/sick_scan_xd once the new pipeline has replaced it everywhere.
 */
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>


namespace ms_test::legacy
{

struct Cloud
{
    std::vector<uint8_t> data;
    uint32_t point_step = 0;
    uint32_t width = 0;
    uint32_t stamp_sec = 0;
    uint32_t stamp_nsec = 0;
    size_t pointCount() const
    {
        return point_step ? data.size() / point_step : 0;
    }
};

struct Imu
{
    uint32_t stamp_sec = 0, stamp_nsec = 0;
    double angular_velocity[3]{}, linear_acceleration[3]{},
        orientation_wxyz[4]{};
};

class Pipeline
{
public:
    explicit Pipeline(int echo_selection = 1, int max_segment_buffering = 3);
    ~Pipeline();

    /* Feed one complete UDP datagram, as the driver's receive loop would. */
    void feed(std::span<const uint8_t> datagram, uint64_t recv_time_ns);

    std::vector<Cloud> clouds;
    std::vector<Imu> imus;
    size_t receive_rejects = 0;  // recvSegmentData() < 0 (framing/CRC)
    size_t parse_failures = 0;   // parseSegment() == false

private:
    struct State;
    State* s;
};

}  // namespace ms_test::legacy
