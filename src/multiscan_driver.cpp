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

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <ctime>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <csm_metrics/stats.hpp>
#include <csm_metrics/profiling.hpp>

#include <csm_utils/ros_utils.hpp>

#include "compact/decoder.hpp"
#include "compact/point_layout.hpp"
#include "frame_assembler.hpp"
#include "net/udp_receiver.hpp"

#include "sick_scan_xd/softwarePLL.h"
#include "sick_scan_xd/sick_scan_common_tcp.h"
#include "sick_scan_xd/sopas_services.h"


#define STATS_PUB_FREQUNCY      10U
#define STATS_PUB_DELTA_TIME_MS (1000U / STATS_PUB_FREQUNCY)

#ifndef PUBLISH_PROCESS_METRICS
    #define PUBLISH_PROCESS_METRICS 1
#endif


using namespace util::ros_aliases;
using namespace sick_scan_xd;

namespace compact = multiscan::compact;


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
    void handleDatagram(std::span<const uint8_t> datagram, uint64_t recv_ns);
    void handleImu(std::span<const uint8_t> datagram, uint64_t recv_ns);
    void publishFrame(multiscan::FrameAssembler::Frame& frame);
    void reportStreamHealth();
    void runReceiver();

    /* Sensor clock (us) -> host clock (ns) via the vendored software PLL, as
     * the old parser did. Replaced by the time sync rewrite. */
    uint64_t toHostTime(uint64_t sensor_us, uint64_t recv_ns);

private:
    // the scanner can take a few seconds to start streaming after (re)start
    static constexpr double INITIAL_UDP_TIMEOUT_S = 5.;
    static constexpr int RECEIVE_POLL_MS = 20;

    struct
    {
        std::string lidar_frame_id;

        std::string lidar_hostname = "";
        std::string driver_hostname = "";
        int lidar_udp_port = 2115;
        int imu_udp_port = 2116;
        int udp_receive_buffer_bytes = 4 << 20;
        int sopas_tcp_port = 2111;
        bool use_cola_binary =
            false;  // port 2111 only speaks CoLa A (firmware 2.3)
        // restart the connection after this long without valid lidar data
        double udp_dropout_reset_thresh = 2.;
        double sopas_read_timeout = 3.;
        double error_restart_timeout = 3.;
        // 1-based echo to publish. Falls back to the first echo the sensor
        // sends (with its echo filter on first or last it sends only one).
        int echo_selection = 1;
        std::vector<std::string> point_fields;
        bool publish_partial_frames = true;
    } config;

    SharedPub<PointCloudMsg> scan_pub;
    SharedPub<ImuMsg> imu_pub;

    std::vector<PointCloudMsg::_fields_type::value_type> scan_fields;
    std::unique_ptr<compact::ScanDecoder> decoder;
    std::unique_ptr<multiscan::FrameAssembler> assembler;
    std::vector<multiscan::FrameAssembler::Frame> frames;
    multiscan::FrameAssembler::Stats reported_stats;

    multiscan::UdpReceiver receiver;
    std::unique_ptr<SickScanCommonTcp> sopas_tcp;
    std::unique_ptr<SopasServices> sopas_service;

    std::thread recv_thread;
    std::atomic<bool> is_running = true;

#if PUBLISH_PROCESS_METRICS
    SharedPub<ProcessStatsMsg> proc_stats_pub;
    RclTimer stats_pub_timer;
    csm::metrics::ProcessStats process_stats;
#endif
};


// --- Implementation ----------------------------------------------------------

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
    util::declare_param(this, "imu_udp_port", this->config.imu_udp_port, 2116);
    util::declare_param(
        this,
        "udp_receive_buffer_bytes",
        this->config.udp_receive_buffer_bytes,
        4 << 20);
    util::declare_param(
        this,
        "sopas_tcp_port",
        this->config.sopas_tcp_port,
        2111);
    util::declare_param(
        this,
        "use_cola_binary",
        this->config.use_cola_binary,
        false);
    util::declare_param(
        this,
        "udp_reset_timeout",
        this->config.udp_dropout_reset_thresh,
        2.);
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
    util::declare_param(this, "echo_selection", this->config.echo_selection, 1);
    util::declare_param(
        this,
        "output.point_fields",
        this->config.point_fields,
        std::vector<std::string>{
            "x",
            "y",
            "z",
            "intensity",
            "range",
            "azimuth",
            "elevation",
            "layer",
            "echo",
            "index",
            "tl",
            "th",
            "reflective"});
    util::declare_param(
        this,
        "output.publish_partial_frames",
        this->config.publish_partial_frames,
        true);

    bool use_msgpack = false;
    util::declare_param(this, "use_msgpack", use_msgpack, false);
    if (use_msgpack)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[MULTISCAN DRIVER]: MSGPACK is no longer supported (it never worked in this "
            "driver); using the Compact format.");
    }

    std::string layout_error;
    auto layout = multiscan::PointLayout::fromNames(
        this->config.point_fields,
        layout_error);
    if (!layout)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Invalid output.point_fields (%s); publishing all fields.",
            layout_error.c_str());
        layout = multiscan::PointLayout::all();
    }
    for (const auto& f : layout->fields())
    {
        this->scan_fields.push_back(
            sensor_msgs::msg::PointField{}
                .set__name(f.name)
                .set__offset(f.offset)
                .set__datatype(f.datatype)
                .set__count(1));
    }
    this->decoder = std::make_unique<compact::ScanDecoder>(
        *layout,
        compact::DecodeOptions{
            .echo_selection = static_cast<uint32_t>(
                std::max(0, this->config.echo_selection))});
    this->assembler = std::make_unique<multiscan::FrameAssembler>(
        *this->decoder,
        multiscan::FrameAssembler::Config{
            .publish_partial_frames = this->config.publish_partial_frames},
        [this](uint64_t sensor_us, uint64_t recv_ns)
        { return this->toHostTime(sensor_us, recv_ns); });

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
    this->receiver.stop();
    if (this->recv_thread.joinable())
    {
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
            2,     // compact
            true,  // enable imu data
            this->config.imu_udp_port))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: SOPAS startup sequence failed.");
        return false;
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "[MULTISCAN DRIVER]: Successfully sent all startup commands.");

    return true;
}

uint64_t MultiscanNode::toHostTime(uint64_t sensor_us, uint64_t recv_ns)
{
    const uint64_t sys_ns = recv_ns;
    const uint32_t curtick = static_cast<uint32_t>(sensor_us & 0xFFFFFFFF);
    SoftwarePLL& pll = SoftwarePLL::instance();
    pll.updatePLL(
        static_cast<uint32_t>(sys_ns / 1000000000),
        static_cast<uint32_t>(sys_ns % 1000000000),
        curtick);
    uint32_t sec = static_cast<uint32_t>(sensor_us / 1000000);
    uint32_t nsec = static_cast<uint32_t>(1000 * (sensor_us % 1000000));
    if (pll.IsInitialized())
    {
        pll.getCorrectedTimeStamp(sec, nsec, curtick);
    }
    return static_cast<uint64_t>(sec) * 1000000000ULL + nsec;
}

void MultiscanNode::handleDatagram(
    std::span<const uint8_t> datagram,
    uint64_t recv_ns)
{
    switch (compact::telegramType(datagram))
    {
        case compact::TelegramType::Scan:
        {
            const compact::Error err =
                this->assembler->add(datagram, recv_ns, this->frames);
            if (err != compact::Error::None)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "[MULTISCAN DRIVER]: Dropping scan telegram: %s.",
                    compact::toString(err));
            }
            break;
        }
        case compact::TelegramType::Imu:
            this->handleImu(datagram, recv_ns);
            break;
        default:
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[MULTISCAN DRIVER]: Ignoring unknown %zu byte datagram.",
                datagram.size());
            break;
    }
}

void MultiscanNode::handleImu(
    std::span<const uint8_t> datagram,
    uint64_t recv_ns)
{
    compact::ImuSample s;
    const compact::Error err = compact::decodeImu(datagram, s);
    if (err != compact::Error::None)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[MULTISCAN DRIVER]: Dropping IMU telegram: %s.",
            compact::toString(err));
        return;
    }
    const uint64_t stamp_ns = this->toHostTime(s.time_us, recv_ns);

    auto msg = std::make_unique<ImuMsg>();
    msg->header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000ULL);
    msg->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);
    msg->header.frame_id = this->config.lidar_frame_id;
    msg->angular_velocity.x = s.angular_velocity[0];
    msg->angular_velocity.y = s.angular_velocity[1];
    msg->angular_velocity.z = s.angular_velocity[2];
    msg->linear_acceleration.x = s.acceleration[0];
    msg->linear_acceleration.y = s.acceleration[1];
    msg->linear_acceleration.z = s.acceleration[2];
    msg->orientation.w = s.orientation[0];
    msg->orientation.x = s.orientation[1];
    msg->orientation.y = s.orientation[2];
    msg->orientation.z = s.orientation[3];
    this->imu_pub->publish(std::move(msg));
}

void MultiscanNode::publishFrame(multiscan::FrameAssembler::Frame& frame)
{
    const uint32_t step = this->decoder->layout().pointStep();
    auto scan = std::make_unique<PointCloudMsg>();
    scan->data = std::move(frame.data);  // no copy
    scan->fields = this->scan_fields;
    scan->is_bigendian = false;
    scan->point_step = step;
    scan->row_step = static_cast<uint32_t>(scan->data.size());
    scan->height = 1;
    scan->width = frame.points;
    scan->is_dense = true;
    scan->header.frame_id = this->config.lidar_frame_id;
    scan->header.stamp.sec =
        static_cast<int32_t>(frame.stamp_ns / 1000000000ULL);
    scan->header.stamp.nanosec =
        static_cast<uint32_t>(frame.stamp_ns % 1000000000ULL);

    this->scan_pub->publish(std::move(scan));
}

void MultiscanNode::reportStreamHealth()
{
    const auto& now = this->assembler->stats();
    auto& prev = this->reported_stats;
    const uint64_t lost = now.telegrams_lost - prev.telegrams_lost;
    const uint64_t partial = now.frames_partial - prev.frames_partial;
    const uint64_t discarded = now.frames_discarded - prev.frames_discarded;
    const uint64_t missed = now.frames_missed - prev.frames_missed;
    const uint64_t dup_late = (now.segments_duplicate + now.segments_late) -
                              (prev.segments_duplicate + prev.segments_late);
    if (now.expected_mask_changes != prev.expected_mask_changes)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Sensor sends %d of %u segments per frame (mask 0x%03x).",
            std::popcount(this->assembler->expectedMask()),
            12u,
            this->assembler->expectedMask());
    }
    if (lost || partial || discarded || missed || dup_late)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Stream degraded since last report: %lu telegrams lost, "
            "%lu partial frames published, %lu discarded, %lu frames missed, %lu duplicate/late segments.",
            static_cast<unsigned long>(lost),
            static_cast<unsigned long>(partial),
            static_cast<unsigned long>(discarded),
            static_cast<unsigned long>(missed),
            static_cast<unsigned long>(dup_late));
    }
    prev = now;
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
        "\n\tCoLa configuration: %s",
        this->config.lidar_hostname.c_str(),
        this->config.driver_hostname.c_str(),
        this->config.lidar_udp_port,
        this->config.sopas_tcp_port,
        this->config.use_cola_binary ? "Binary" : "ASCII");

    {
        std::string error;
        std::vector<uint16_t> ports{
            static_cast<uint16_t>(this->config.lidar_udp_port),
            static_cast<uint16_t>(this->config.imu_udp_port)};
        if (!this->receiver
                 .open(ports, this->config.udp_receive_buffer_bytes, error))
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Cannot open UDP sockets: %s",
                error.c_str());
            return;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Listening on UDP %d (scan) and %d (IMU), %d KiB receive buffer per socket.",
            this->config.lidar_udp_port,
            this->config.imu_udp_port,
            this->receiver.receiveBufferBytes() / 1024);
        if (this->receiver.receiveBufferBytes() <
            this->config.udp_receive_buffer_bytes / 2)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[MULTISCAN DRIVER]: The kernel limited the UDP receive buffer to %d KiB "
                "(requested %d KiB); raise net.core.rmem_max to buffer more.",
                this->receiver.receiveBufferBytes() / 1024,
                this->config.udp_receive_buffer_bytes / 1024);
        }
    }

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
            std::vector<multiscan::UdpReceiver::Datagram> batch;
            bool received_data = false;
            auto last_udp_recv_time = std::chrono::steady_clock::now();
            auto last_report_time = last_udp_recv_time;
            this->assembler->reset();

            while (this->is_running && sopas_tcp->isConnected())
            {
                PROFILING_SYNC();
                PROFILING_FLUSH();

                PROFILING_NOTIFY(receive_bytes);
                const int n = this->receiver.receive(batch, RECEIVE_POLL_MS);
                PROFILING_NOTIFY(receive_bytes);
                if (n < 0)
                {
                    break;  // stopped or socket error
                }
                const auto now = std::chrono::steady_clock::now();

                PROFILING_NOTIFY(parse_segment);
                for (const auto& d : batch)
                {
                    this->handleDatagram(d.data, d.recv_ns);
                }
                timespec rt;
                clock_gettime(CLOCK_REALTIME, &rt);
                this->assembler->flushStale(
                    static_cast<uint64_t>(rt.tv_sec) * 1000000000ULL +
                        static_cast<uint64_t>(rt.tv_nsec),
                    this->frames);
                PROFILING_NOTIFY(parse_segment);

                PROFILING_NOTIFY(export_cloud);
                for (auto& f : this->frames)
                {
                    this->publishFrame(f);
                }
                this->frames.clear();
                PROFILING_NOTIFY(export_cloud);

                if (now - last_report_time > std::chrono::seconds(5))
                {
                    this->reportStreamHealth();
                    last_report_time = now;
                }

                if (n > 0)
                {
                    received_data = true;
                    last_udp_recv_time = now;
                    continue;
                }
                // The TCP link can look healthy after the lidar reboots or
                // loses power, so lack of UDP data is what triggers a reset.
                const double reset_after_s =
                    received_data ? this->config.udp_dropout_reset_thresh
                                  : std::max(
                                        this->config.udp_dropout_reset_thresh,
                                        INITIAL_UDP_TIMEOUT_S);
                const double silent_s =
                    std::chrono::duration<double>(now - last_udp_recv_time)
                        .count();
                if (silent_s > reset_after_s)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[MULTISCAN DRIVER]: No valid lidar data for %.1f seconds.",
                        silent_s);
                    break;
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
                    "[MULTISCAN DRIVER]: Restarting connections after timeout...");
            }
            // sleep in short steps so shutdown() isn't held up
            const auto restart_time =
                std::chrono::steady_clock::now() +
                std::chrono::milliseconds(
                    static_cast<int64_t>(
                        this->config.error_restart_timeout * 1e3));
            while (this->is_running &&
                   std::chrono::steady_clock::now() < restart_time)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
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
