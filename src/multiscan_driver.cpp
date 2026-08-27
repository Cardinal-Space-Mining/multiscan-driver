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

#include <array>
#include <deque>
#include <mutex>
#include <atomic>
#include <memory>
#include <limits>
#include <thread>
#include <vector>
// #include <iostream>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <csm_metrics/stats.hpp>
#include <csm_metrics/profiling.hpp>

#include <multiscan_driver/multiscan_spec.hpp>

#include "sick_scan_xd/udp_sockets.h"
#include "sick_scan_xd/msgpack_parser.h"
#include "sick_scan_xd/compact_parser.h"
#include "sick_scan_xd/scansegment_parser_output.h"
#include "sick_scan_xd/sick_scan_common_tcp.h"
#include "sick_scan_xd/sopas_services.h"

#include <csm_utils/ros_utils.hpp>
#include "point_fields.hpp"
#ifndef MS_DRIVER_POINT_TYPE_FIELDS
    #define MS_DRIVER_POINT_TYPE_FIELDS MS_POINT_FIELD_ENABLE_ALL
#endif
#include "point_type.hpp"


#define STATS_PUB_FREQUNCY      10U
#define STATS_PUB_DELTA_TIME_MS (1000U / STATS_PUB_FREQUNCY)

#ifndef PUBLISH_PROCESS_METRICS
    #define PUBLISH_PROCESS_METRICS 1
#endif

#if PUBLISH_PROCESS_METRICS
    #define IF_PUBLISH_PROCESS_METRICS(...) __VA_ARGS__
#else
    #define IF_PUBLISH_PROCESS_METRICS(...)
#endif


using namespace util::ros_aliases;
using namespace sick_scan_xd;
using namespace sick_scansegment_xd;

#define ssxd     sick_scan_xd
#define ssgmt_xd sick_scansegment_xd


class MultiscanNode : public rclcpp::Node
{
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using ProcessStatsMsg = csm_metrics::msg::ProcessStats;

public:
    MultiscanNode(bool autostart = true);
    ~MultiscanNode();

    void start();
    void shutdown();

protected:
    bool initConnection();
    int recvSegmentData(
        double udp_recv_timeout_s,
        chrono_system_time& recv_start_time);
    bool parseSegment(
        ScanSegmentParserOutput& seg,
        const chrono_system_time& recv_start_time);
    void publishImu(const ScanSegmentParserOutput& seg);
    void addSegment(ScanSegmentParserOutput& seg);
    void publishScan();

    void runReceiver();
#if PUBLISH_PROCESS_METRICS
    void publishStats();
#endif

private:
    // constant from sick_scansegment_xd
    static constexpr size_t RECV_BUFFER_N_BYTES = 64 * 1024;

    using SegmentQueue = std::deque<ScanSegmentParserOutput>;
    using SampleBuffer = std::array<SegmentQueue, ms136::SEGMENTS_PER_FRAME>;

    struct
    {
        std::string lidar_frame_id;

        std::string lidar_hostname = "";
        std::string driver_hostname = "";
        int lidar_udp_port = 2115;
        // int imu_udp_port = 2115;
        int sopas_tcp_port = 2111;
        bool use_msgpack = false;
        bool use_cola_binary = true;
        double udp_dropout_reset_thresh = 2.;
        double udp_receive_timeout = 1.;
        double sopas_read_timeout = 3.;
        double error_restart_timeout = 3.;
        int max_segment_buffering = 3;
        int echo_selection = 3;
    } config;

    SharedPub<PointCloudMsg> scan_pub;
    SharedPub<ImuMsg> imu_pub;

    PointCloudMsg::_fields_type scan_fields;

    UdpReceiverSocketImpl udp_recv_socket;
    std::unique_ptr<SickScanCommonTcp> sopas_tcp;
    std::unique_ptr<SopasServices> sopas_service;

    std::vector<uint8_t> udp_buffer;
    SampleBuffer samples;
    size_t sample_fill_mask = 0;

    std::thread recv_thread;
    std::atomic<bool> is_running = true;

#if PUBLISH_PROCESS_METRICS
    SharedPub<ProcessStatsMsg> proc_stats_pub;
    RclTimer stats_pub_timer;
    csm::metrics::ProcessStats process_stats;
#endif
};


// --- Implementation ----------------------------------------------------------

void moveSegmentsNoIMU(ScanSegmentParserOutput& a, ScanSegmentParserOutput& b)
{
    a.scandata = std::move(b.scandata);
    a.timestamp = std::move(b.timestamp);
    a.timestamp_sec = b.timestamp_sec;
    a.timestamp_nsec = b.timestamp_nsec;
    a.segmentIndex = b.segmentIndex;
    a.telegramCnt = b.telegramCnt;
}

// void runTests(
//     const std::vector<ScanSegmentParserOutput::LidarPoint>& all_points)
// {
//     ms136::redux::DenseBuffer dense_buff;
//     dense_buff.fill(0);

//     for (const auto& pt : all_points)
//     {
//         ms136::redux::addPointToBuffer(
//             dense_buff,
//             pt.layerIdx,
//             pt.pointIdx,
//             pt.range,
//             pt.reflectorbit);
//     }

//     ms136::redux::PackedBuffer packed;
//     ms136::redux::packBuffer(packed, dense_buff);

//     ms136::redux::DenseBuffer unpacked;
//     ms136::redux::unpackBuffer(unpacked, packed);

//     size_t n_empty_pts = 0;
//     double total_error = 0.;
//     float max_error = 0.f;
//     for (const auto& pt : all_points)
//     {
//         const size_t dense_i =
//             ms136::redux::computeDenseIdx(pt.layerIdx, pt.pointIdx);
//         assert(dense_i < ms136::POINTS_PER_LR_SCAN);
//         const uint16_t range_mm = unpacked[dense_i] & ms136::redux::RANGE_MASK;
//         if (range_mm)
//         {
//             const float range_m = static_cast<float>(range_mm) / 1000.f;
//             const Eigen::Vector3f proj_v3f =
//                 ms136::redux::projectPoint(dense_i, range_m);
//             const Eigen::Vector3f ref_v3f{pt.x, pt.y, pt.z};
//             const float error = (proj_v3f - ref_v3f).norm();
//             total_error += static_cast<double>(error);
//             max_error = std::max(max_error, error);
//         }
//         else
//         {
//             n_empty_pts++;
//         }
//     }
//     const size_t n_tested_pts = (all_points.size() - n_empty_pts);

//     std::cout << "FRAME\n"
//                  "\tInput points : "
//               << all_points.size()
//               << "\n"
//                  "\tEmpty points : "
//               << n_empty_pts
//               << "\n"
//                  "\tTotal repojection error : "
//               << total_error << " (" << n_tested_pts << " pts // avg : "
//               << (total_error / static_cast<double>(n_tested_pts))
//               << ")\n"
//                  "\tMax reprojection error : "
//               << max_error
//               << "\n"
//                  "\tPack/Unpack equal? : "
//               << (dense_buff == unpacked)
//               << "\n"
//                  "\tReduction % : "
//               << static_cast<double>(dense_buff.size() - packed.size()) /
//                      dense_buff.size()
//               << "\n"
//               << std::endl;

//     // if (this->prev_points.empty())
//     // {
//     //     std::ostringstream ss;
//     //     ss << "/home/hoodi/multiscan_pt_coords_"
//     //        << std::chrono::duration_cast<std::chrono::seconds>(
//     //               std::chrono::system_clock::now().time_since_epoch())
//     //               .count()
//     //        << ".csv";
//     //     std::ofstream f;
//     //     f.open(ss.str(), std::ios::out);
//     //     for (const LidarPoint& p : all_points)
//     //     {
//     //         f << p.elevation << ", "
//     //             << p.azimuth << ", "
//     //             << p.segmentIdx << ", "
//     //             << p.layerIdx << ", "
//     //             << p.pointIdx << ", "
//     //             << p.echoIdx << "\n";
//     //     }
//     //     f.flush();
//     //     f.close();
//     // }
// }


MultiscanNode::MultiscanNode(bool autostart) : Node("multiscan_driver")
{
    util::declare_param(
        this,
        "lidar_frame",
        this->config.lidar_frame_id,
        "lidar_link");
    util::declare_param(
        this,
        "lidar_hostname",
        this->config.lidar_hostname,
        "");
    util::declare_param(
        this,
        "driver_hostname",
        this->config.driver_hostname,
        "");
    util::declare_param(
        this,
        "lidar_udp_port",
        this->config.lidar_udp_port,
        2115);
    // util::declare_param(this, "imu_udp_port", this->config.imu_udp_port, 2115);
    util::declare_param(
        this,
        "sopas_tcp_port",
        this->config.sopas_tcp_port,
        2111);
    util::declare_param(this, "use_msgpack", this->config.use_msgpack, false);
    util::declare_param(
        this,
        "use_cola_binary",
        this->config.use_cola_binary,
        true);
    util::declare_param(
        this,
        "udp_reset_timeout",
        this->config.udp_dropout_reset_thresh,
        2.);
    util::declare_param(
        this,
        "udp_receive_timeout",
        this->config.udp_receive_timeout,
        1.);
    util::declare_param(
        this,
        "sopas_read_timeout",
        this->config.sopas_read_timeout,
        3.);
    util::declare_param(
        this,
        "error_restart_timeout",
        this->config.error_restart_timeout,
        3.);
    util::declare_param(
        this,
        "max_segment_buffers",
        this->config.max_segment_buffering,
        3);
    util::declare_param(this, "echo_selection", this->config.echo_selection, 3);

    this->scan_pub = this->create_publisher<PointCloudMsg>(
        "lidar_scan",
        rclcpp::SensorDataQoS{});
    this->imu_pub =
        this->create_publisher<ImuMsg>("lidar_imu", rclcpp::SensorDataQoS{});

#if PUBLISH_PROCESS_METRICS
    this->proc_stats_pub = this->create_publisher<ProcessStatsMsg>(
        "multiscan_driver/process_stats",
        rclcpp::SensorDataQoS{});
    this->stats_pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(STATS_PUB_DELTA_TIME_MS),
        [this]()
        {
            this->process_stats.update();
            this->proc_stats_pub->publish(this->process_stats.toMsg());
        });
#endif

    this->scan_fields = MS_DRIVER_POINT_FIELD_LIST;

    if (autostart)
    {
        this->start();
    }
}

MultiscanNode::~MultiscanNode() { this->shutdown(); }


void MultiscanNode::start()
{
    this->is_running = true;
    if (!this->recv_thread.joinable())
    {
        this->recv_thread = std::thread{&MultiscanNode::runReceiver, this};
    }
}

void MultiscanNode::shutdown()
{
    this->is_running = false;
    if (this->recv_thread.joinable())
    {
        this->udp_recv_socket.ForceStop();
        this->recv_thread.join();
    }
}


bool MultiscanNode::initConnection()
{
    this->sopas_tcp = std::make_unique<SickScanCommonTcp>(
        this->config.lidar_hostname,
        this->config.sopas_tcp_port,
        this->config.use_cola_binary ? 'B' : 'A');
    this->sopas_service = std::make_unique<SopasServices>(
        this->sopas_tcp.get(),
        this->config.use_cola_binary);
    this->sopas_tcp->init_device(3);
    this->sopas_tcp->setReadTimeOutInMs(
        static_cast<size_t>(this->config.sopas_read_timeout * 1e3));

    if (!this->sopas_tcp->isConnected())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Failed to setup TCP SOPAS connection to "
            "initialize device using parameters:\n"
            "\tDevice Hostname: %s\n"
            "\tSOPAS TCP Port: %d\n"
            "\tCoLa Mode: %s\n",
            this->config.lidar_hostname.c_str(),
            this->config.sopas_tcp_port,
            this->config.use_cola_binary ? "Binary" : "Ascii");
        return false;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "[MULTISCAN DRIVER]: TCP connected! Sending startup commands...");

    if (!this->sopas_service->sendAuthorization())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: SOPAS authorization command failed.");
        return false;
    }

    if (!this->sopas_service->sendMultiScanStartCmd(
            this->config.driver_hostname,
            this->config.lidar_udp_port,
            (2 -
             static_cast<int>(
                 this->config.use_msgpack)),  // 1 for msgpack, 2 for compact
            true,                             // enable imu data
            this->config.lidar_udp_port))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: SOPAS startup sequence failed.");
        return false;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "[MULTISCAN DRIVER]: Successfully sent all startup commands.");

    if (!this->udp_recv_socket.Init("", this->config.lidar_udp_port))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Failed to initialize UDP socket using port %d.",
            this->config.lidar_udp_port);
        return false;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "[MULTISCAN DRIVER]: Successfully initialized UDP connection!");

    return true;
}

int MultiscanNode::recvSegmentData(
    double udp_recv_timeout_s,
    chrono_system_time& recv_start_time)
{
    static const std::vector<uint8_t> UDP_MSG_START_SEQ(
        {0x02, 0x02, 0x02, 0x02});

    size_t bytes_received = this->udp_recv_socket.Receive(
        this->udp_buffer,
        udp_recv_timeout_s,
        UDP_MSG_START_SEQ);

    const bool recv_ok =
        (bytes_received > UDP_MSG_START_SEQ.size() + 8) &&
        std::equal(
            this->udp_buffer.begin(),
            this->udp_buffer.begin() + UDP_MSG_START_SEQ.size(),
            UDP_MSG_START_SEQ.begin());
    if (!recv_ok)
    {
        return -1;
    }

    uint32_t payload_len_bytes = 0;
    uint32_t bytes_to_receive = 0;
    uint32_t udp_payload_offset = 0;

    recv_start_time = chrono_system_clock::now();
    if (this->config.use_msgpack)
    {
        payload_len_bytes = ssgmt_xd::Convert4Byte(
            udp_buffer.data() + UDP_MSG_START_SEQ.size());
        bytes_to_receive = static_cast<uint32_t>(
            payload_len_bytes + UDP_MSG_START_SEQ.size() +
            2 * sizeof(uint32_t));
        // payload starts after (4 byte \x02\x02\x02\x02) + (4 byte payload length)
        udp_payload_offset = UDP_MSG_START_SEQ.size() + sizeof(uint32_t);
    }
    else
    {
        bool parse_success = false;
        uint32_t n_bytes_req = 0;
        while (this->is_running &&
               !(parse_success = CompactDataParser::ParseSegment(
                     this->udp_buffer.data(),
                     bytes_received,
                     0,
                     payload_len_bytes,
                     n_bytes_req)) &&
               (udp_recv_timeout_s < 0 ||
                ssgmt_xd::Seconds(recv_start_time, chrono_system_clock::now()) <
                    udp_recv_timeout_s))
        {
            if (n_bytes_req > (1 << 20))
            {
                parse_success = false;
                CompactDataParser::ParseSegment(
                    this->udp_buffer.data(),
                    bytes_received,
                    0,
                    payload_len_bytes,
                    n_bytes_req,
                    0.f,
                    1);
                break;
            }

            while (this->is_running &&
                   (bytes_received < n_bytes_req + sizeof(uint32_t)) &&
                   (udp_recv_timeout_s < 0 ||
                    ssgmt_xd::Seconds(
                        recv_start_time,
                        chrono_system_clock::now()) < udp_recv_timeout_s))
            {
                std::vector<uint8_t> chunk_buffer(RECV_BUFFER_N_BYTES, 0);
                size_t chunk_bytes_received =
                    this->udp_recv_socket.Receive(chunk_buffer);
                this->udp_buffer.insert(
                    this->udp_buffer.begin() + bytes_received,
                    chunk_buffer.begin(),
                    chunk_buffer.begin() + chunk_bytes_received);
                bytes_received += chunk_bytes_received;
            }
        }

        if (!parse_success)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Failed to parse compact payload from bytes received over UDP.");
            return -2;
        }

        // payload + (4 byte CRC)
        bytes_to_receive =
            static_cast<uint32_t>(payload_len_bytes + sizeof(uint32_t));
        // compact format calculates CRC over complete message (incl. header)
        udp_payload_offset = 0;
    }

    size_t bytes_valid =
        std::min<size_t>(bytes_received, static_cast<size_t>(bytes_to_receive));
    const uint32_t u32_payload_crc = ssgmt_xd::Convert4Byte(
        this->udp_buffer.data() + bytes_valid - sizeof(uint32_t));
    const std::vector<uint8_t> msgpack_payload{
        this->udp_buffer.begin() + udp_payload_offset,
        this->udp_buffer.begin() + bytes_valid - sizeof(uint32_t)};
    const uint32_t u32_msgpack_crc =
        ssgmt_xd::crc32(0, msgpack_payload.data(), msgpack_payload.size());

    if (u32_payload_crc != u32_msgpack_crc)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVEr]: CRC payload check failed!");
        return -3;
    }

    return static_cast<int>(bytes_received);
}

bool MultiscanNode::parseSegment(
    ScanSegmentParserOutput& seg,
    const chrono_system_time& recv_start_time)
{
    if (this->config.use_msgpack)
    {
        if (!MsgPackParser::Parse(
                this->udp_buffer,
                recv_start_time,
                seg,
                true,
                false))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Msgpack parse failed.");
            return false;
        }
    }
    else
    {
        if (!CompactDataParser::Parse(
                this->udp_buffer,
                recv_start_time,
                seg,
                0,
                true,
                false))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Compact parse failed.");
            return false;
        }
    }

    return true;
}

void MultiscanNode::publishImu(const ScanSegmentParserOutput& seg)
{
    sensor_msgs::msg::Imu msg;

    msg.header.stamp.sec = seg.timestamp_sec;
    msg.header.stamp.nanosec = seg.timestamp_nsec;
    msg.header.frame_id = this->config.lidar_frame_id;

    msg.angular_velocity.x = seg.imudata.angular_velocity_x;
    msg.angular_velocity.y = seg.imudata.angular_velocity_y;
    msg.angular_velocity.z = seg.imudata.angular_velocity_z;

    msg.linear_acceleration.x = seg.imudata.acceleration_x;
    msg.linear_acceleration.y = seg.imudata.acceleration_y;
    msg.linear_acceleration.z = seg.imudata.acceleration_z;

    msg.orientation.w = seg.imudata.orientation_w;
    msg.orientation.x = seg.imudata.orientation_x;
    msg.orientation.y = seg.imudata.orientation_y;
    msg.orientation.z = seg.imudata.orientation_z;

    this->imu_pub->publish(msg);
}

void MultiscanNode::addSegment(ScanSegmentParserOutput& seg)
{
    const size_t idx = static_cast<size_t>(seg.segmentIndex);
    samples[idx].emplace_front();
    if (samples[idx].size() >
        static_cast<size_t>(this->config.max_segment_buffering))
    {
        samples[idx].resize(this->config.max_segment_buffering);
    }
    ::moveSegmentsNoIMU(samples[idx].front(), seg);
    this->sample_fill_mask |= (1 << idx);
}

void MultiscanNode::publishScan()
{
    using ScanLayer = ScanSegmentParserOutput::Scangroup;
    using ScanLine = ScanSegmentParserOutput::Scanline;
    using LidarPoint = ScanSegmentParserOutput::LidarPoint;

    constexpr size_t POINT_CONTINUOUS_BYTE_LEN =
        COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS) * 4;
    constexpr size_t POINT_BYTE_LEN =
        COMPUTE_NUM_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS) * 4;

    sensor_msgs::msg::PointCloud2 scan;
    scan.data.reserve(ms136::MAX_POINTS_PER_SCAN * POINT_BYTE_LEN);
    scan.data.resize(0);

    // std::vector<ScanSegmentParserOutput::LidarPoint> all_points;
    // all_points.reserve(ms136::POINTS_PER_LR_SCAN);

    uint64_t earliest_ts = std::numeric_limits<uint64_t>::max();
    // segments are groups which points come in from the network (think slice of a pie)
    for (size_t seg_i = 0; seg_i < ms136::SEGMENTS_PER_FRAME; seg_i++)
    {
        ScanSegmentParserOutput& seg = this->samples[seg_i].front();
        const uint64_t ts =
            static_cast<uint64_t>(seg.timestamp_sec) * 1000000000UL +
            static_cast<uint64_t>(seg.timestamp_nsec);
        earliest_ts = std::min(ts, earliest_ts);

        for (ScanLayer& layer : seg.scandata)
        {
            if (layer.scanlines.empty())
            {
                continue;
            }
            const int echo_i =
                (static_cast<size_t>(this->config.echo_selection - 1) <
                 layer.scanlines.size())
                    ? this->config.echo_selection
                    : 0;
            ScanLine& line = layer.scanlines[echo_i];
            const size_t points_per_seg =
                (line.points.size() <= ms136::POINTS_PER_SEGMENT_LR_LAYER)
                    ? ms136::POINTS_PER_SEGMENT_LR_LAYER
                    : ms136::POINTS_PER_SEGMENT_HD_LAYER;

            for (LidarPoint& point : line.points)
            {
                // rescale point idx to be per-layer instead of per-segment
                point.pointIdx = (seg_i * points_per_seg) + point.pointIdx;

                scan.data.resize(scan.data.size() + POINT_BYTE_LEN);
                uint8_t* const point_data =
                    scan.data.end().base() - POINT_BYTE_LEN;

                memcpy(point_data, &point, POINT_CONTINUOUS_BYTE_LEN);

#if POINT_FIELDS_HAVE_TIMESTAMP(MS_DRIVER_POINT_TYPE_FIELDS)
                memcpy(
                    point_data + POINT_CONTINUOUS_BYTE_LEN,
                    &point.lidar_timestamp_microsec,
                    sizeof(point.lidar_timestamp_microsec));
#endif
#if POINT_FIELDS_HAVE_REFLECTOR(MS_DRIVER_POINT_TYPE_FIELDS)
    #define POINT_RB_F32_IDX                                             \
        ((POINT_CONTINUOUS_BYTE_LEN / sizeof(float)) +                   \
         (POINT_FIELDS_HAVE_TIMESTAMP(MS_DRIVER_POINT_TYPE_FIELDS) * 2))
                reinterpret_cast<float*>(point_data)[POINT_RB_F32_IDX] =
                    static_cast<float>(point.reflectorbit);
#endif
            }

            // all_points.insert(
            //     all_points.end(),
            //     line.points.begin(),
            //     line.points.end());
        }
        this->samples[seg_i].clear();
    }
    this->sample_fill_mask = 0;

    scan.fields = this->scan_fields;
    scan.is_bigendian = false;
    scan.point_step = POINT_BYTE_LEN;
    scan.row_step = scan.data.size();
    scan.height = 1;
    scan.width = (scan.data.size() / POINT_BYTE_LEN);
    scan.is_dense = true;
    scan.header.frame_id = this->config.lidar_frame_id;
    scan.header.stamp.sec = (earliest_ts / 1000000000UL);
    scan.header.stamp.nanosec = (earliest_ts % 1000000000UL);

    this->scan_pub->publish(scan);

    // runTests(all_points);
}

void MultiscanNode::runReceiver()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[MULTISCAN DRIVER]: Initializing connections using the following parameters:"
        "\n\tLidar IP address: %s"
        "\n\tDriver IP address: %s"
        "\n\tLidar UDP port: %d"
        "\n\tSOPAS TCP port: %d"
        "\n\tData format: %s"
        "\n\tCoLa configuration: %s",
        this->config.lidar_hostname.c_str(),
        this->config.driver_hostname.c_str(),
        this->config.lidar_udp_port,
        this->config.sopas_tcp_port,
        this->config.use_msgpack ? "MsgPack" : "Compact",
        this->config.use_cola_binary ? "Binary" : "ASCII");

    while (this->is_running)
    {
        PROFILING_NOTIFY(init_connection);
        if (!this->initConnection())
        {
            goto END_L;
        }
        PROFILING_NOTIFY(init_connection);

        try
        {
            this->udp_buffer.clear();
            this->udp_buffer.resize(RECV_BUFFER_N_BYTES, 0);

            int n_recv_bytes = 0;
            double udp_recv_timeout = 5.;
            chrono_system_time recv_start_time;
            chrono_system_time last_udp_recv_time = chrono_system_clock::now();

            while (this->is_running && sopas_tcp->isConnected())
            {
                PROFILING_SYNC();
                PROFILING_FLUSH();

                PROFILING_NOTIFY(receive_bytes);
                if ((n_recv_bytes = this->recvSegmentData(
                         udp_recv_timeout,
                         recv_start_time)) < 0)
                {
                    PROFILING_NOTIFY(receive_bytes);
                    continue;
                }

                PROFILING_NOTIFY2(receive_bytes, parse_segment);

                ScanSegmentParserOutput segment;
                if (!this->parseSegment(segment, recv_start_time))
                {
                    PROFILING_NOTIFY(parse_segment);
                    continue;
                }
                PROFILING_NOTIFY(parse_segment);

                if (segment.imudata.valid)
                {
                    PROFILING_NOTIFY(export_imu);
                    this->publishImu(segment);
                    PROFILING_NOTIFY(export_imu);
                }
                if (!segment.scandata.empty())
                {
                    PROFILING_NOTIFY(queue_sample);
                    this->addSegment(segment);
                    PROFILING_NOTIFY(queue_sample);
                }

                if (this->sample_fill_mask >=
                    (1 << ms136::SEGMENTS_PER_FRAME) - 1)
                {
                    PROFILING_NOTIFY(export_cloud);
                    this->publishScan();
                    PROFILING_NOTIFY(export_cloud);
                }

                if (n_recv_bytes > 0)
                {
                    last_udp_recv_time = chrono_system_clock::now();
                }
                if (ssgmt_xd::Seconds(
                        last_udp_recv_time,
                        chrono_system_clock::now()) >
                    this->config.udp_dropout_reset_thresh)
                {
                    udp_recv_timeout = -1;
                }
                else
                {
                    // receive non-blocking with timeout
                    udp_recv_timeout = this->config.udp_receive_timeout;
                }
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MULTISCAN DRIVER]: UDP decode loop encountered an exception - what():\n\t%s",
                e.what());
        }

    END_L:

        if (this->sopas_tcp->isConnected())
        {
            this->sopas_service->sendAuthorization();
            this->sopas_service->sendMultiScanStopCmd(true);
        }

        if (this->is_running)
        {
            if (!this->sopas_tcp->isConnected())
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[MULTISCAN DRIVER]: Lost connection to SOPAS service. "
                    "Restarting connections after timeout...");
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[MULTISCAN DRIVER]: Encountered decode error. "
                    "Restarting connections after timeout...");
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<size_t>(
                        this->config.error_restart_timeout * 1e3)));
        }
    }
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MultiscanNode>(false);
    PROFILING_INIT(*node);
    node->start();

    rclcpp::spin(node);

    node->shutdown();
    PROFILING_DEINIT();
    rclcpp::shutdown();

    return 0;
}
