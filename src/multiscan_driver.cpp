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
#include <chrono>
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
#include "legacy_slot_assembler.hpp"

#include "sick_scan_xd/udp_sockets.h"
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
using namespace sick_scansegment_xd;

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
    void handleDatagram(std::span<const uint8_t> datagram, const chrono_system_time& recv_time);
    void handleScan(std::span<const uint8_t> datagram, const chrono_system_time& recv_time);
    void handleImu(std::span<const uint8_t> datagram, const chrono_system_time& recv_time);
    void publishScan();
    void runReceiver();

    /* Sensor clock (us) -> host clock (ns) via the vendored software PLL, as
     * the old parser did. Replaced by the time sync rewrite. */
    uint64_t toHostTime(uint64_t sensor_us, const chrono_system_time& recv_time);

private:
    // a whole datagram; compact segments are at most ~15 KB
    static constexpr size_t RECV_BUFFER_N_BYTES = 64 * 1024;
    // the scanner can take a few seconds to start streaming after (re)start
    static constexpr double INITIAL_UDP_TIMEOUT_S = 5.;
    static constexpr size_t SEGMENT_POINT_CAPACITY = 4096;

    struct
    {
        std::string lidar_frame_id;

        std::string lidar_hostname = "";
        std::string driver_hostname = "";
        int lidar_udp_port = 2115;
        int sopas_tcp_port = 2111;
        bool use_cola_binary = false;  // port 2111 only speaks CoLa A (firmware 2.3)
        // restart the connection after this long without valid lidar data
        double udp_dropout_reset_thresh = 2.;
        double sopas_read_timeout = 3.;
        double error_restart_timeout = 3.;
        // 1-based echo to publish. Falls back to the first echo the sensor
        // sends (with its echo filter on first or last it sends only one).
        int echo_selection = 1;
        std::vector<std::string> point_fields;
    } config;

    SharedPub<PointCloudMsg> scan_pub;
    SharedPub<ImuMsg> imu_pub;

    std::vector<PointCloudMsg::_fields_type::value_type> scan_fields;
    std::unique_ptr<compact::ScanDecoder> decoder;
    multiscan::LegacySlotAssembler assembler;

    UdpReceiverSocketImpl udp_recv_socket;
    std::unique_ptr<SickScanCommonTcp> sopas_tcp;
    std::unique_ptr<SopasServices> sopas_service;

    std::vector<uint8_t> udp_buffer = std::vector<uint8_t>(RECV_BUFFER_N_BYTES);

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
    util::declare_param(this, "lidar_frame", this->config.lidar_frame_id, "lidar_link");
    util::declare_param(this, "lidar_hostname", this->config.lidar_hostname, "");
    util::declare_param(this, "driver_hostname", this->config.driver_hostname, "");
    util::declare_param(this, "lidar_udp_port", this->config.lidar_udp_port, 2115);
    util::declare_param(this, "sopas_tcp_port", this->config.sopas_tcp_port, 2111);
    util::declare_param(this, "use_cola_binary", this->config.use_cola_binary, false);
    util::declare_param(this, "udp_reset_timeout", this->config.udp_dropout_reset_thresh, 2.);
    util::declare_param(this, "sopas_read_timeout", this->config.sopas_read_timeout, 3.);
    util::declare_param(this, "error_restart_timeout", this->config.error_restart_timeout, 3.);
    util::declare_param(this, "echo_selection", this->config.echo_selection, 1);
    util::declare_param(
        this,
        "output.point_fields",
        this->config.point_fields,
        std::vector<std::string>{"x", "y", "z", "intensity", "range", "azimuth", "elevation",
                                 "layer", "echo", "index", "tl", "th", "reflective"});

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
    auto layout = multiscan::PointLayout::fromNames(this->config.point_fields, layout_error);
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
        this->scan_fields.push_back(sensor_msgs::msg::PointField{}
                                        .set__name(f.name)
                                        .set__offset(f.offset)
                                        .set__datatype(f.datatype)
                                        .set__count(1));
    }
    this->decoder = std::make_unique<compact::ScanDecoder>(
        *layout,
        compact::DecodeOptions{
            .echo_selection = static_cast<uint32_t>(std::max(0, this->config.echo_selection))});

    this->scan_pub = this->create_publisher<PointCloudMsg>("lidar_scan", rclcpp::SensorDataQoS{});
    this->imu_pub = this->create_publisher<ImuMsg>("lidar_imu", rclcpp::SensorDataQoS{});

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
    this->sopas_service = std::make_unique<SopasServices>(this->sopas_tcp.get(), this->config.use_cola_binary);
    this->sopas_tcp->init_device(3);
    this->sopas_tcp->setReadTimeOutInMs(static_cast<size_t>(this->config.sopas_read_timeout * 1e3));

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

    RCLCPP_DEBUG(this->get_logger(), "[MULTISCAN DRIVER]: TCP connected! Sending startup commands...");

    if (!this->sopas_service->sendAuthorization())
    {
        RCLCPP_ERROR(this->get_logger(), "[MULTISCAN DRIVER]: SOPAS authorization command failed.");
        return false;
    }

    if (!this->sopas_service->sendMultiScanStartCmd(
            this->config.driver_hostname,
            this->config.lidar_udp_port,
            2,     // compact
            true,  // enable imu data
            this->config.lidar_udp_port))
    {
        RCLCPP_ERROR(this->get_logger(), "[MULTISCAN DRIVER]: SOPAS startup sequence failed.");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "[MULTISCAN DRIVER]: Successfully sent all startup commands.");

    if (!this->udp_recv_socket.Init("", this->config.lidar_udp_port))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Failed to initialize UDP socket using port %d.",
            this->config.lidar_udp_port);
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "[MULTISCAN DRIVER]: Successfully initialized UDP connection!");

    return true;
}

uint64_t MultiscanNode::toHostTime(uint64_t sensor_us, const chrono_system_time& recv_time)
{
    const int64_t sys_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(recv_time.time_since_epoch()).count();
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

void MultiscanNode::handleDatagram(std::span<const uint8_t> datagram, const chrono_system_time& recv_time)
{
    switch (compact::telegramType(datagram))
    {
        case compact::TelegramType::Scan:
            this->handleScan(datagram, recv_time);
            break;
        case compact::TelegramType::Imu:
            this->handleImu(datagram, recv_time);
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

void MultiscanNode::handleScan(std::span<const uint8_t> datagram, const chrono_system_time& recv_time)
{
    // the segment counter selects the slot to decode into; decode() validates it
    uint64_t segment = std::numeric_limits<uint64_t>::max();
    if (datagram.size() >= compact::SCAN_HEADER_BYTES + 8)
    {
        std::memcpy(&segment, datagram.data() + compact::SCAN_HEADER_BYTES, sizeof(segment));
    }
    if (segment >= multiscan::LegacySlotAssembler::SEGMENTS)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[MULTISCAN DRIVER]: Dropping segment with out-of-range index %lu.",
            static_cast<unsigned long>(segment));
        return;
    }

    const size_t step = this->decoder->layout().pointStep();
    size_t capacity = SEGMENT_POINT_CAPACITY;
    compact::ScanInfo info;
    compact::Error err = this->decoder->decode(
        datagram,
        this->assembler.slotBuffer(segment, capacity, step),
        capacity,
        info);
    if (err == compact::Error::BufferTooSmall)
    {
        capacity = info.points;
        err = this->decoder->decode(
            datagram,
            this->assembler.slotBuffer(segment, capacity, step),
            capacity,
            info);
    }
    if (err != compact::Error::None)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[MULTISCAN DRIVER]: Dropping scan telegram: %s.",
            compact::toString(err));
        return;
    }

    const uint64_t stamp_ns = this->toHostTime(info.start_time_us, recv_time);
    if (this->assembler.commit(segment, info.points, stamp_ns))
    {
        PROFILING_NOTIFY(export_cloud);
        this->publishScan();
        PROFILING_NOTIFY(export_cloud);
    }
}

void MultiscanNode::handleImu(std::span<const uint8_t> datagram, const chrono_system_time& recv_time)
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
    const uint64_t stamp_ns = this->toHostTime(s.time_us, recv_time);

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

void MultiscanNode::publishScan()
{
    const uint32_t step = this->decoder->layout().pointStep();
    auto scan = std::make_unique<PointCloudMsg>();
    const uint64_t earliest_ns = this->assembler.take(scan->data, step);

    scan->fields = this->scan_fields;
    scan->is_bigendian = false;
    scan->point_step = step;
    scan->row_step = static_cast<uint32_t>(scan->data.size());
    scan->height = 1;
    scan->width = static_cast<uint32_t>(scan->data.size() / step);
    scan->is_dense = true;
    scan->header.frame_id = this->config.lidar_frame_id;
    scan->header.stamp.sec = static_cast<int32_t>(earliest_ns / 1000000000ULL);
    scan->header.stamp.nanosec = static_cast<uint32_t>(earliest_ns % 1000000000ULL);

    this->scan_pub->publish(std::move(scan));
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
            bool received_data = false;
            chrono_system_time last_udp_recv_time = chrono_system_clock::now();

            while (this->is_running && sopas_tcp->isConnected())
            {
                PROFILING_SYNC();
                PROFILING_FLUSH();

                PROFILING_NOTIFY(receive_bytes);
                // one datagram, or 0 after the socket's 100 ms receive timeout
                const size_t n = this->udp_recv_socket.Receive(this->udp_buffer);
                const chrono_system_time recv_time = chrono_system_clock::now();
                PROFILING_NOTIFY(receive_bytes);

                if (n == 0)
                {
                    // The TCP link can look healthy after the lidar reboots or
                    // loses power, so lack of UDP data is what triggers a reset.
                    const double reset_after_s =
                        received_data
                            ? this->config.udp_dropout_reset_thresh
                            : std::max(this->config.udp_dropout_reset_thresh, INITIAL_UDP_TIMEOUT_S);
                    const double silent_s = sick_scansegment_xd::Seconds(last_udp_recv_time, recv_time);
                    if (silent_s > reset_after_s)
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "[MULTISCAN DRIVER]: No valid lidar data for %.1f seconds.",
                            silent_s);
                        break;
                    }
                    continue;
                }
                received_data = true;
                last_udp_recv_time = recv_time;

                PROFILING_NOTIFY(parse_segment);
                this->handleDatagram(std::span<const uint8_t>(this->udp_buffer.data(), n), recv_time);
                PROFILING_NOTIFY(parse_segment);
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
                RCLCPP_ERROR(this->get_logger(), "[MULTISCAN DRIVER]: Restarting connections after timeout...");
            }
            // sleep in short steps so shutdown() isn't held up
            const auto restart_time = std::chrono::steady_clock::now() +
                                      std::chrono::milliseconds(static_cast<int64_t>(
                                          this->config.error_restart_timeout * 1e3));
            while (this->is_running && std::chrono::steady_clock::now() < restart_time)
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
