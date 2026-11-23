#include "legacy_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>

#include "sick_scan_xd/compact_parser.h"
#include "sick_scan_xd/scansegment_parser_output.h"
#include "sick_scan_xd/udp_sockets.h"

#include <multiscan_driver/multiscan_spec.hpp>
#include "point_fields.hpp"
#ifndef MS_DRIVER_POINT_TYPE_FIELDS
    #define MS_DRIVER_POINT_TYPE_FIELDS MS_POINT_FIELD_ENABLE_ALL
#endif

using namespace sick_scansegment_xd;


namespace ms_test::legacy
{

struct Pipeline::State
{
    using SegmentQueue = std::deque<ScanSegmentParserOutput>;
    std::array<SegmentQueue, ms136::SEGMENTS_PER_FRAME> samples;
    size_t sample_fill_mask = 0;
    int echo_selection;
    int max_segment_buffering;
};

Pipeline::Pipeline(int echo_selection, int max_segment_buffering) : s{new State}
{
    s->echo_selection = echo_selection;
    s->max_segment_buffering = std::max(1, max_segment_buffering);
}

Pipeline::~Pipeline() { delete s; }


// multiscan_driver.cpp: moveSegmentsNoIMU()
static void moveSegmentsNoIMU(ScanSegmentParserOutput& a, ScanSegmentParserOutput& b)
{
    a.scandata = std::move(b.scandata);
    a.timestamp = std::move(b.timestamp);
    a.timestamp_sec = b.timestamp_sec;
    a.timestamp_nsec = b.timestamp_nsec;
    a.segmentIndex = b.segmentIndex;
    a.telegramCnt = b.telegramCnt;
}

void Pipeline::feed(std::span<const uint8_t> datagram, uint64_t recv_time_ns)
{
    static const std::vector<uint8_t> UDP_MSG_START_SEQ({0x02, 0x02, 0x02, 0x02});
    std::vector<uint8_t> udp_buffer(datagram.begin(), datagram.end());
    const size_t bytes_received = udp_buffer.size();
    const chrono_system_time recv_start_time{
        std::chrono::duration_cast<chrono_system_clock::duration>(std::chrono::nanoseconds(recv_time_ns))};

    // --- recvSegmentData() (compact branch, one complete datagram) ---------
    const bool recv_ok = (bytes_received > UDP_MSG_START_SEQ.size() + 8) &&
                         std::equal(udp_buffer.begin(), udp_buffer.begin() + UDP_MSG_START_SEQ.size(),
                                    UDP_MSG_START_SEQ.begin());
    if (!recv_ok)
    {
        receive_rejects++;
        return;
    }
    uint32_t payload_len_bytes = 0, n_bytes_req = 0;
    if (!CompactDataParser::ParseSegment(udp_buffer.data(), bytes_received, 0, payload_len_bytes, n_bytes_req))
    {
        receive_rejects++;
        return;
    }
    const uint32_t bytes_to_receive = static_cast<uint32_t>(payload_len_bytes + sizeof(uint32_t));
    const size_t bytes_valid = std::min<size_t>(bytes_received, bytes_to_receive);
    const uint32_t u32_payload_crc = Convert4Byte(udp_buffer.data() + bytes_valid - sizeof(uint32_t));
    const std::vector<uint8_t> payload{udp_buffer.begin(), udp_buffer.begin() + bytes_valid - sizeof(uint32_t)};
    if (u32_payload_crc != sick_scansegment_xd::crc32(0, payload.data(), payload.size()))
    {
        receive_rejects++;
        return;
    }

    // --- parseSegment() ----------------------------------------------------
    ScanSegmentParserOutput segment;
    if (!CompactDataParser::Parse(udp_buffer, recv_start_time, segment, 0, true, false))
    {
        parse_failures++;
        return;
    }

    // --- publishImu() ------------------------------------------------------
    if (segment.imudata.valid)
    {
        Imu m;
        m.stamp_sec = segment.timestamp_sec;
        m.stamp_nsec = segment.timestamp_nsec;
        m.angular_velocity[0] = segment.imudata.angular_velocity_x;
        m.angular_velocity[1] = segment.imudata.angular_velocity_y;
        m.angular_velocity[2] = segment.imudata.angular_velocity_z;
        m.linear_acceleration[0] = segment.imudata.acceleration_x;
        m.linear_acceleration[1] = segment.imudata.acceleration_y;
        m.linear_acceleration[2] = segment.imudata.acceleration_z;
        m.orientation_wxyz[0] = segment.imudata.orientation_w;
        m.orientation_wxyz[1] = segment.imudata.orientation_x;
        m.orientation_wxyz[2] = segment.imudata.orientation_y;
        m.orientation_wxyz[3] = segment.imudata.orientation_z;
        imus.push_back(m);
    }

    // --- addSegment() ------------------------------------------------------
    if (!segment.scandata.empty())
    {
        const size_t idx = static_cast<size_t>(segment.segmentIndex);
        if (idx < ms136::SEGMENTS_PER_FRAME)
        {
            s->samples[idx].emplace_front();
            if (s->samples[idx].size() > static_cast<size_t>(s->max_segment_buffering))
            {
                s->samples[idx].resize(s->max_segment_buffering);
            }
            moveSegmentsNoIMU(s->samples[idx].front(), segment);
            s->sample_fill_mask |= (1 << idx);
        }
    }

    if (s->sample_fill_mask < (1u << ms136::SEGMENTS_PER_FRAME) - 1)
    {
        return;
    }

    // --- publishScan() -----------------------------------------------------
    using ScanLayer = ScanSegmentParserOutput::Scangroup;
    using ScanLine = ScanSegmentParserOutput::Scanline;
    using LidarPoint = ScanSegmentParserOutput::LidarPoint;

    constexpr size_t POINT_CONTINUOUS_BYTE_LEN = COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS) * 4;
    constexpr size_t POINT_BYTE_LEN = COMPUTE_NUM_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS) * 4;

    Cloud scan;
    scan.data.reserve(ms136::MAX_POINTS_PER_SCAN * POINT_BYTE_LEN);

    uint64_t earliest_ts = std::numeric_limits<uint64_t>::max();
    for (size_t seg_i = 0; seg_i < ms136::SEGMENTS_PER_FRAME; seg_i++)
    {
        ScanSegmentParserOutput& seg = s->samples[seg_i].front();
        const uint64_t ts = static_cast<uint64_t>(seg.timestamp_sec) * 1000000000UL +
                            static_cast<uint64_t>(seg.timestamp_nsec);
        earliest_ts = std::min(ts, earliest_ts);

        for (ScanLayer& layer : seg.scandata)
        {
            if (layer.scanlines.empty())
            {
                continue;
            }
            const size_t echo_i = (s->echo_selection >= 1 &&
                                   static_cast<size_t>(s->echo_selection) <= layer.scanlines.size())
                                      ? static_cast<size_t>(s->echo_selection - 1)
                                      : 0;
            ScanLine& line = layer.scanlines[echo_i];
            const size_t points_per_seg = line.points.size();

            for (LidarPoint& point : line.points)
            {
                point.pointIdx = static_cast<uint32_t>((seg_i * points_per_seg) + point.pointIdx);

                scan.data.resize(scan.data.size() + POINT_BYTE_LEN);
                uint8_t* const point_data = scan.data.data() + scan.data.size() - POINT_BYTE_LEN;

                std::memcpy(point_data, &point, POINT_CONTINUOUS_BYTE_LEN);
#if POINT_FIELDS_HAVE_TIMESTAMP(MS_DRIVER_POINT_TYPE_FIELDS)
                std::memcpy(point_data + POINT_CONTINUOUS_BYTE_LEN, &point.lidar_timestamp_microsec,
                            sizeof(point.lidar_timestamp_microsec));
#endif
#if POINT_FIELDS_HAVE_REFLECTOR(MS_DRIVER_POINT_TYPE_FIELDS)
                const float refl = static_cast<float>(point.reflectorbit);
                std::memcpy(point_data + POINT_CONTINUOUS_BYTE_LEN +
                                POINT_FIELDS_HAVE_TIMESTAMP(MS_DRIVER_POINT_TYPE_FIELDS) * 8,
                            &refl, sizeof(refl));
#endif
            }
        }
        s->samples[seg_i].clear();
    }
    s->sample_fill_mask = 0;

    scan.point_step = POINT_BYTE_LEN;
    scan.width = static_cast<uint32_t>(scan.data.size() / POINT_BYTE_LEN);
    scan.stamp_sec = static_cast<uint32_t>(earliest_ts / 1000000000UL);
    scan.stamp_nsec = static_cast<uint32_t>(earliest_ts % 1000000000UL);
    clouds.push_back(std::move(scan));
}

}  // namespace ms_test::legacy
