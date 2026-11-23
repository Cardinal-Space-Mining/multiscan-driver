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
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <csm_metrics/stats.hpp>
#include <csm_metrics/profiling.hpp>

#include <csm_utils/ros_utils.hpp>

#include "compact/decoder.hpp"
#include "compact/point_layout.hpp"
#include "frame_assembler.hpp"
#include "net/udp_receiver.hpp"

#include "sopas/cola_client.hpp"
#include "sopas/sensor_session.hpp"
#include "sopas/sensor_settings.hpp"
#include "time_sync.hpp"


#define STATS_PUB_FREQUNCY      10U
#define STATS_PUB_DELTA_TIME_MS (1000U / STATS_PUB_FREQUNCY)

#ifndef PUBLISH_PROCESS_METRICS
    #define PUBLISH_PROCESS_METRICS 1
#endif


using namespace util::ros_aliases;

namespace compact = multiscan::compact;
namespace sopas = multiscan::sopas;


class MultiscanNode : public rclcpp::Node
{
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
    using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
    using ProcessStatsMsg = csm_metrics::msg::ProcessStats;

public:
    MultiscanNode(bool autostart = true);
    ~MultiscanNode();

    void start();
    void shutdown();

protected:
    void declareSensorSettings();
    bool initConnection();
    void stopStreaming();
    void handleDatagram(std::span<const uint8_t> datagram, uint64_t recv_ns);
    void handleImu(std::span<const uint8_t> datagram, uint64_t recv_ns);
    void publishFrame(multiscan::FrameAssembler::Frame& frame);
    void reportStreamHealth();
    void runReceiver();
    bool pollSensorStatus();
    void updateDiagnosticsSnapshot(bool streaming);
    void publishDiagnostics();

    /* host - sensor clock offset (ns) for a scan telegram, per time.source;
     * every telegram also feeds the clock estimator. */
    int64_t clockOffset(uint64_t transmit_us, uint64_t recv_ns);

private:
    // the scanner can take a few seconds to start streaming after (re)start
    static constexpr double INITIAL_UDP_TIMEOUT_S = 5.;
    static constexpr int RECEIVE_POLL_MS = 20;
    // IMU position relative to the optical origin (operating instructions)
    static constexpr double IMU_OFFSET_M[3] = {-0.017, 0.0235, -0.030};

    enum class TimeSource
    {
        Estimated,  // sensor clock mapped to the host clock (ClockEstimator)
        Sensor,     // sensor clock as is (synchronised via NTP/PTP)
        Receive,    // host receive time of each telegram
    };

    struct
    {
        std::string lidar_frame_id;

        std::string lidar_hostname = "";
        std::string driver_hostname = "";
        int lidar_udp_port = 2115;
        int imu_udp_port = 2116;
        int udp_receive_buffer_bytes = 4 << 20;
        int sopas_tcp_port = 2111;  // CoLa A (firmware 2.3: 2111 = CoLa A only)
        std::string sopas_password = "F4724744";  // Authorized Client hash
        // restart the connection after this long without valid lidar data
        double udp_dropout_reset_thresh = 2.;
        double sopas_read_timeout = 3.;
        double error_restart_timeout = 3.;
        // 1-based echo to publish. Falls back to the first echo the sensor
        // sends (with its echo filter on first or last it sends only one).
        int echo_selection = 1;
        std::vector<std::string> point_fields;
        bool publish_partial_frames = true;
        // applied on every connect (never persisted) unless apply is false
        bool apply_sensor_settings = true;
        sopas::SensorSettings sensor;
        TimeSource time_source = TimeSource::Estimated;
        std::string imu_frame_id;
        std::array<double, 3> imu_orientation_cov{};
        std::array<double, 3> imu_angular_velocity_cov{};
        std::array<double, 3> imu_linear_acceleration_cov{};
        double status_period_s = 2.;
    } config;

    /* What /diagnostics reports. Written by the receive thread, read by the
     * diagnostics timer. */
    struct DiagnosticsSnapshot
    {
        bool connected = false;
        bool streaming = false;  // data arrived within udp_reset_timeout
        uint64_t connects = 0;
        std::string last_error;
        sopas::DeviceInfo device;
        std::string stream_host;
        size_t settings_total = 0;
        size_t settings_verified = 0;
        std::vector<std::string> setting_problems;

        bool have_status = false;
        sopas::SensorStatus status;

        multiscan::FrameAssembler::Stats stream;
        uint64_t frames_published = 0;
        uint64_t imu_published = 0;
        uint32_t expected_mask = 0;

        bool clock_valid = false;
        int64_t clock_offset_ns = 0;
        double clock_drift = 0.;
        multiscan::ClockEstimator::Stats clock;
    };

    SharedPub<PointCloudMsg> scan_pub;
    SharedPub<ImuMsg> imu_pub;
    SharedPub<DiagnosticArrayMsg> diag_pub;
    RclTimer diag_timer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> imu_tf;

    std::vector<PointCloudMsg::_fields_type::value_type> scan_fields;
    std::unique_ptr<compact::ScanDecoder> decoder;
    std::unique_ptr<multiscan::FrameAssembler> assembler;
    std::vector<multiscan::FrameAssembler::Frame> frames;
    multiscan::FrameAssembler::Stats reported_stats;

    multiscan::UdpReceiver receiver;
    sopas::ColaClient cola;
    multiscan::ClockEstimator clock;
    sopas::SensorStatus last_status;
    bool have_last_status = false;
    uint64_t frames_published = 0;
    uint64_t imu_published = 0;

    std::mutex diag_mtx;
    DiagnosticsSnapshot diag;           // guarded by diag_mtx
    DiagnosticsSnapshot diag_previous;  // timer thread only
    std::chrono::steady_clock::time_point diag_previous_time;

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
        "sopas_password",
        this->config.sopas_password,
        "F4724744");
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

    {
        std::string source = "estimated";
        util::declare_param(
            this,
            "time.source",
            source,
            std::string{"estimated"});
        if (source == "estimated")
        {
            this->config.time_source = TimeSource::Estimated;
        }
        else if (source == "sensor")
        {
            this->config.time_source = TimeSource::Sensor;
        }
        else if (source == "receive")
        {
            this->config.time_source = TimeSource::Receive;
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MULTISCAN DRIVER]: time.source must be estimated, sensor or receive "
                "(got '%s'); using estimated.",
                source.c_str());
        }
    }

    util::declare_param(this, "imu.frame_id", this->config.imu_frame_id, "");
    bool imu_publish_tf = true;
    util::declare_param(this, "imu.publish_tf", imu_publish_tf, true);
    {
        auto cov = [this](const char* name, std::array<double, 3>& out)
        {
            std::vector<double> v{0., 0., 0.};
            util::declare_param(this, name, v, std::vector<double>{0., 0., 0.});
            if (v.size() == 3)
            {
                std::copy(v.begin(), v.end(), out.begin());
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[MULTISCAN DRIVER]: %s needs 3 values (x, y, z variances); using 0 (unknown).",
                    name);
            }
        };
        cov("imu.orientation_covariance", this->config.imu_orientation_cov);
        cov("imu.angular_velocity_covariance",
            this->config.imu_angular_velocity_cov);
        cov("imu.linear_acceleration_covariance",
            this->config.imu_linear_acceleration_cov);
    }
    util::declare_param(
        this,
        "diagnostics.status_period",
        this->config.status_period_s,
        2.);

    this->declareSensorSettings();

    if (this->config.time_source == TimeSource::Sensor &&
        this->config.sensor.time_sync.mode == sopas::TimeSyncMode::Off)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[MULTISCAN DRIVER]: time.source is 'sensor' but sensor.time_sync.mode is "
            "'off': the sensor clock counts from power-up, so stamps will not be "
            "host time unless the sensor is synchronised some other way.");
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
        [this](const compact::ScanInfo& info, uint64_t recv_ns)
        { return this->clockOffset(info.transmit_time_us, recv_ns); });

    this->scan_pub = this->create_publisher<PointCloudMsg>(
        "lidar_scan",
        rclcpp::SensorDataQoS{});
    this->imu_pub =
        this->create_publisher<ImuMsg>("lidar_imu", rclcpp::SensorDataQoS{});

    if (this->config.imu_frame_id.empty())
    {
        this->config.imu_frame_id = this->config.lidar_frame_id;
    }
    else if (
        imu_publish_tf &&
        this->config.imu_frame_id != this->config.lidar_frame_id)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = this->config.lidar_frame_id;
        tf.child_frame_id = this->config.imu_frame_id;
        tf.transform.translation.x = IMU_OFFSET_M[0];
        tf.transform.translation.y = IMU_OFFSET_M[1];
        tf.transform.translation.z = IMU_OFFSET_M[2];
        tf.transform.rotation.w = 1.;  // IMU axes are the device axes
        this->imu_tf =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        this->imu_tf->sendTransform(tf);
    }

    this->diag_pub = this->create_publisher<DiagnosticArrayMsg>(
        "/diagnostics",
        rclcpp::SystemDefaultsQoS{});
    this->diag_previous_time = std::chrono::steady_clock::now();
    this->diag_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { this->publishDiagnostics(); });

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


void MultiscanNode::declareSensorSettings()
{
    sopas::SensorSettings& s = this->config.sensor;
    std::vector<std::string> errors;

    auto flag = [this](const std::string& name, bool def)
    {
        bool v = def;
        util::declare_param(this, name, v, def);
        return v;
    };
    auto integer = [this](const std::string& name, int64_t def)
    {
        int64_t v = def;
        util::declare_param(this, name, v, def);
        return v;
    };
    auto text = [this](const std::string& name, const std::string& def)
    {
        std::string v = def;
        util::declare_param(this, name, v, def);
        return v;
    };
    auto integers =
        [this, &errors](const std::string& name, std::vector<int64_t> def)
    {
        const size_t n = def.size();
        std::vector<int64_t> v = def;
        util::declare_param(this, name, v, def);
        if (v.size() != n)
        {
            errors.push_back(name + " needs " + std::to_string(n) + " values");
            v = def;
        }
        return v;
    };
    auto toBools = [](const std::vector<int64_t>& v, auto& out)
    {
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i] = v[i] != 0;
        }
    };

    util::declare_param(
        this,
        "sensor.apply",
        this->config.apply_sensor_settings,
        true);

    const std::string echo = text("sensor.echo_filter", "last");
    if (auto e = sopas::echoFilterFromString(echo))
    {
        s.echo_filter = *e;
    }
    else
    {
        errors.push_back(
            "sensor.echo_filter must be first, last or all (got '" + echo +
            "')");
    }

    toBools(
        integers(
            "sensor.layers",
            {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1}),
        s.layers);
    {
        std::vector<double> range{-180., 180.};
        util::declare_param(
            this,
            "sensor.output_range_deg",
            range,
            std::vector<double>{-180., 180.});
        if (range.size() == 2)
        {
            s.output_range_start_deg = range[0];
            s.output_range_stop_deg = range[1];
        }
        else
        {
            errors.push_back("sensor.output_range_deg needs [start, stop]");
        }
    }
    s.interlaced = flag("sensor.interlaced", false);

    s.range_filter.enabled = flag("sensor.range_filter.enabled", false);
    s.range_filter.min_mm = integer("sensor.range_filter.min_mm", 0);
    s.range_filter.max_mm = integer("sensor.range_filter.max_mm", 200000);

    s.cubic_filter.enabled = flag("sensor.cubic_filter.enabled", false);
    {
        const auto lo =
            integers("sensor.cubic_filter.min_mm", {-20000, -20000, -20000});
        const auto hi =
            integers("sensor.cubic_filter.max_mm", {20000, 20000, 20000});
        std::copy(lo.begin(), lo.end(), s.cubic_filter.min_mm.begin());
        std::copy(hi.begin(), hi.end(), s.cubic_filter.max_mm.begin());
    }

    s.particle_filter = flag("sensor.particle_filter", false);
    s.fog_filter = flag("sensor.fog_filter", false);
    s.moving_average.enabled = flag("sensor.moving_average.enabled", false);
    s.moving_average.depth = integer("sensor.moving_average.depth", 2);
    s.interval_filter.enabled = flag("sensor.interval_filter.enabled", false);
    s.interval_filter.factor = integer("sensor.interval_filter.factor", 4);

    auto& c = s.contamination;
    const std::string strategy =
        text("sensor.contamination.strategy", "inactive");
    if (auto v = sopas::contaminationStrategyFromString(strategy))
    {
        c.strategy = *v;
    }
    else
    {
        errors.push_back(
            "sensor.contamination.strategy must be inactive, high_availability or sensitive");
    }
    c.response_time_s = integer("sensor.contamination.response_time_s", 3);
    const std::string sensitivity =
        text("sensor.contamination.sensitivity", "low");
    if (auto v = sopas::contaminationSensitivityFromString(sensitivity))
    {
        c.sensitivity = *v;
    }
    else
    {
        errors.push_back(
            "sensor.contamination.sensitivity must be high, medium or low");
    }
    c.custom_sectors = flag("sensor.contamination.custom_sectors", true);
    toBools(
        integers("sensor.contamination.sectors", std::vector<int64_t>(12, 1)),
        c.sectors);
    c.warning = flag("sensor.contamination.warning", true);
    c.error = flag("sensor.contamination.error", true);

    const std::string sync = text("sensor.time_sync.mode", "off");
    if (auto v = sopas::timeSyncModeFromString(sync))
    {
        s.time_sync.mode = *v;
    }
    else
    {
        errors.push_back("sensor.time_sync.mode must be off, ntp or ptp");
    }
    s.time_sync.ntp_server = text("sensor.time_sync.ntp_server", "");
    s.time_sync.ntp_update_s = integer("sensor.time_sync.ntp_update_s", 10);

    if (const std::string v = s.validate(); !v.empty())
    {
        errors.push_back(v);
    }
    if (!errors.empty())
    {
        std::string all;
        for (const auto& e : errors)
        {
            all += "\n\t" + e;
        }
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Invalid sensor settings; they will NOT be applied, "
            "only compared with the sensor:%s",
            all.c_str());
        this->config.apply_sensor_settings = false;
    }
}

bool MultiscanNode::initConnection()
{
    const int timeout_ms =
        static_cast<int>(this->config.sopas_read_timeout * 1e3);
    std::string err;
    if (!this->cola.connect(
            this->config.lidar_hostname,
            static_cast<uint16_t>(this->config.sopas_tcp_port),
            timeout_ms,
            err))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: SOPAS connection failed: %s",
            err.c_str());
        std::lock_guard l(this->diag_mtx);
        this->diag.connected = false;
        this->diag.last_error = "SOPAS connection failed: " + err;
        return false;
    }

    const std::string host = this->config.driver_hostname.empty()
                                 ? this->cola.localAddress()
                                 : this->config.driver_hostname;
    sopas::SensorSettings settings = this->config.sensor;
    if (settings.time_sync.mode == sopas::TimeSyncMode::Ntp &&
        settings.time_sync.ntp_server.empty())
    {
        settings.time_sync.ntp_server = host;  // this host serves NTP
    }
    sopas::SensorSession session(
        this->cola,
        this->config.sopas_password,
        timeout_ms);
    sopas::StartReport report;
    const bool ok = session.start(
        {host,
         static_cast<uint16_t>(this->config.lidar_udp_port),
         static_cast<uint16_t>(this->config.imu_udp_port),
         true},
        settings,
        this->config.apply_sensor_settings,
        report);

    {
        std::lock_guard l(this->diag_mtx);
        this->diag.connected = this->cola.connected();
        this->diag.device = report.device;
        this->diag.stream_host = host;
        this->diag.settings_total = report.settings.size();
        this->diag.settings_verified = 0;
        this->diag.setting_problems.clear();
        for (const auto& st : report.settings)
        {
            this->diag.settings_verified += st.verified;
            if (!st.problem.empty())
            {
                this->diag.setting_problems.push_back(
                    st.name + ": " + st.problem);
            }
        }
        if (ok)
        {
            this->diag.connects++;
            this->diag.last_error.clear();
        }
        else
        {
            this->diag.last_error = "sensor start failed: " + report.error;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[MULTISCAN DRIVER]: Connected to %s (serial %s, firmware %s); stream to %s:%d (scan) / %d (IMU).",
        report.device.type.c_str(),
        report.device.serial.c_str(),
        report.device.firmware.c_str(),
        host.c_str(),
        this->config.lidar_udp_port,
        this->config.imu_udp_port);
    size_t verified = 0;
    for (const auto& st : report.settings)
    {
        verified += st.verified;
        if (!st.problem.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Sensor setting %s: %s",
                st.name.c_str(),
                st.problem.c_str());
        }
    }
    if (!ok)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Sensor start failed: %s",
            report.error.c_str());
        return false;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "[MULTISCAN DRIVER]: %s; %zu of %zu sensor settings verified, %zu problems.",
        this->config.apply_sensor_settings
            ? "Settings applied"
            : "Settings compared only (sensor.apply is false)",
        verified,
        report.settings.size(),
        report.problems());
    return true;
}

void MultiscanNode::stopStreaming()
{
    if (this->cola.connected())
    {
        sopas::SensorSession session(
            this->cola,
            this->config.sopas_password,
            static_cast<int>(this->config.sopas_read_timeout * 1e3));
        std::string err;
        if (!session.stop(err))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Stopping the sensor output failed: %s",
                err.c_str());
        }
    }
    this->cola.disconnect();
}

int64_t MultiscanNode::clockOffset(uint64_t transmit_us, uint64_t recv_ns)
{
    this->clock.add(transmit_us, recv_ns);
    switch (this->config.time_source)
    {
        case TimeSource::Sensor:
            return 0;
        case TimeSource::Receive:
            return static_cast<int64_t>(recv_ns) -
                   static_cast<int64_t>(transmit_us * 1000);
        case TimeSource::Estimated:
        default:
            return this->clock.offsetNs(recv_ns);
    }
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
    // IMU telegrams carry no transmit time, only the sample time: they use
    // the offset estimated from the scan telegrams
    uint64_t stamp_ns = recv_ns;
    if (this->config.time_source == TimeSource::Sensor)
    {
        stamp_ns = s.time_us * 1000;
    }
    else if (
        this->config.time_source == TimeSource::Estimated &&
        this->clock.valid())
    {
        stamp_ns = this->clock.toHostNs(s.time_us, recv_ns);
    }

    auto msg = std::make_unique<ImuMsg>();
    msg->header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000ULL);
    msg->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);
    msg->header.frame_id = this->config.imu_frame_id;
    for (int i = 0; i < 3; i++)
    {
        msg->orientation_covariance[i * 4] =
            this->config.imu_orientation_cov[i];
        msg->angular_velocity_covariance[i * 4] =
            this->config.imu_angular_velocity_cov[i];
        msg->linear_acceleration_covariance[i * 4] =
            this->config.imu_linear_acceleration_cov[i];
    }
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
    this->imu_published++;
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
    this->frames_published++;
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

bool MultiscanNode::pollSensorStatus()
{
    sopas::SensorSession session(
        this->cola,
        this->config.sopas_password,
        static_cast<int>(this->config.sopas_read_timeout * 1e3));
    sopas::SensorStatus st;
    std::string err;
    if (!session.readStatus(st, err))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[MULTISCAN DRIVER]: Reading the sensor status failed: %s",
            err.c_str());
        return false;
    }

    // log changes (and anything abnormal on the first read)
    const sopas::SensorStatus& prev = this->last_status;
    const bool first = !this->have_last_status;
    constexpr int NORMAL =
        static_cast<int>(sopas::SensorStatus::DeviceState::Normal);
    if (st.have_state && (first ? st.state != NORMAL : st.state != prev.state))
    {
        if (st.state == NORMAL)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Sensor state back to normal operation.");
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Sensor state: %s (%d).",
                sopas::deviceStateName(st.state),
                st.state);
        }
    }
    if (st.have_contamination &&
        (first ? (st.contamination_warning || st.contamination_error)
               : (st.contamination_warning != prev.contamination_warning ||
                  st.contamination_error != prev.contamination_error)))
    {
        if (st.contamination_error)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Optics cover contamination ERROR: clean the sensor window.");
        }
        else if (st.contamination_warning)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Optics cover contamination warning.");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[MULTISCAN DRIVER]: Optics cover contamination cleared.");
        }
    }
    this->last_status = st;
    this->have_last_status = true;

    std::lock_guard l(this->diag_mtx);
    this->diag.have_status = true;
    this->diag.status = st;
    return true;
}

void MultiscanNode::updateDiagnosticsSnapshot(bool streaming)
{
    timespec rt;
    clock_gettime(CLOCK_REALTIME, &rt);
    const uint64_t now_ns = static_cast<uint64_t>(rt.tv_sec) * 1000000000ULL +
                            static_cast<uint64_t>(rt.tv_nsec);

    std::lock_guard l(this->diag_mtx);
    this->diag.connected = this->cola.connected();
    this->diag.streaming = streaming;
    if (streaming)
    {
        this->diag.last_error.clear();
    }
    this->diag.stream = this->assembler->stats();
    this->diag.expected_mask = this->assembler->expectedMask();
    this->diag.frames_published = this->frames_published;
    this->diag.imu_published = this->imu_published;
    this->diag.clock_valid = this->clock.valid();
    this->diag.clock_offset_ns = this->clock.offsetNs(now_ns);
    this->diag.clock_drift = this->clock.drift();
    this->diag.clock = this->clock.stats();
}

void MultiscanNode::publishDiagnostics()
{
    using Status = DiagnosticStatusMsg;
    DiagnosticsSnapshot d;
    {
        std::lock_guard l(this->diag_mtx);
        d = this->diag;
    }
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::max(
        1e-3,
        std::chrono::duration<double>(now - this->diag_previous_time).count());
    const DiagnosticsSnapshot& p = this->diag_previous;

    const std::string hw = d.device.serial.empty()
                               ? this->config.lidar_hostname
                               : d.device.type + " " + d.device.serial;
    auto make = [&hw](const char* name, uint8_t level, std::string message)
    {
        Status st;
        st.name = std::string("multiscan_driver: ") + name;
        st.hardware_id = hw;
        st.level = level;
        st.message = std::move(message);
        return st;
    };
    auto kv = [](Status& st, const std::string& key, const std::string& value)
    {
        st.values.push_back(
            diagnostic_msgs::msg::KeyValue{}.set__key(key).set__value(value));
    };
    auto fmt = [](const char* f, double v)
    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), f, v);
        return std::string(buf);
    };

    auto msg = std::make_unique<DiagnosticArrayMsg>();
    msg->header.stamp = this->now();

    // --- connection
    {
        Status st =
            d.streaming ? make("Connection", Status::OK, "streaming")
            : d.connected
                ? make("Connection", Status::WARN, "connected, no data")
                : make(
                      "Connection",
                      Status::ERROR,
                      d.last_error.empty() ? "not connected" : d.last_error);
        kv(st, "Sensor", this->config.lidar_hostname);
        kv(st, "Stream destination", d.stream_host);
        kv(st, "Device", d.device.type);
        kv(st, "Serial", d.device.serial);
        kv(st, "Firmware", d.device.firmware);
        kv(st, "Connects", std::to_string(d.connects));
        if (!d.last_error.empty())
        {
            kv(st, "Last error", d.last_error);
        }
        msg->status.push_back(std::move(st));
    }

    // --- device state and temperature
    {
        Status st = make("Sensor", Status::STALE, "no status read yet");
        if (d.have_status && d.status.have_state)
        {
            using DS = sopas::SensorStatus::DeviceState;
            const int s = d.status.state;
            st.level = s == static_cast<int>(DS::Normal) ? Status::OK
                       : (s == static_cast<int>(DS::RecoverableError) ||
                          s == static_cast<int>(DS::FatalError))
                           ? Status::ERROR
                           : Status::WARN;
            st.message = sopas::deviceStateName(s);
            kv(st,
               "Device state",
               std::to_string(s) + " (" + sopas::deviceStateName(s) + ")");
        }
        if (d.have_status && d.status.have_temperature)
        {
            kv(st,
               "Internal temperature [C]",
               fmt("%.1f", d.status.temperature_c));
        }
        msg->status.push_back(std::move(st));
    }

    // --- contamination
    {
        const bool monitoring = this->config.sensor.contamination.strategy !=
                                sopas::ContaminationStrategy::Inactive;
        Status st = make("Contamination", Status::STALE, "no status read yet");
        if (d.have_status && d.status.have_contamination)
        {
            if (d.status.contamination_error)
            {
                st.level = Status::ERROR;
                st.message = "contaminated (error): clean the optics cover";
            }
            else if (d.status.contamination_warning)
            {
                st.level = Status::WARN;
                st.message = "contaminated (warning)";
            }
            else
            {
                st.level = Status::OK;
                st.message = monitoring ? "clean" : "monitoring inactive";
            }
            kv(st,
               "Warning",
               d.status.contamination_warning ? "true" : "false");
            kv(st, "Error", d.status.contamination_error ? "true" : "false");
        }
        if (d.have_status && d.status.have_sectors)
        {
            for (size_t i = 0; i < d.status.sectors.size(); i++)
            {
                kv(st,
                   "Sector " + std::to_string(i + 1),
                   sopas::sectorName(d.status.sectors[i]));
            }
        }
        msg->status.push_back(std::move(st));
    }

    // --- stream (rates over the last period)
    {
        const double fps = (d.frames_published - p.frames_published) / dt;
        const uint64_t lost = d.stream.telegrams_lost - p.stream.telegrams_lost;
        const uint64_t partial =
            d.stream.frames_partial - p.stream.frames_partial;
        const uint64_t discarded =
            d.stream.frames_discarded - p.stream.frames_discarded;
        const uint64_t errors = d.stream.decode_errors - p.stream.decode_errors;
        Status st = make("Stream", Status::OK, fmt("%.1f frames/s", fps));
        if (!d.streaming)
        {
            st.level = Status::ERROR;
            st.message = "no data";
        }
        else if (lost || partial || discarded || errors)
        {
            st.level = Status::WARN;
            st.message += ", degraded";
        }
        kv(st, "Frames/s", fmt("%.1f", fps));
        kv(st,
           "IMU samples/s",
           fmt("%.1f", (d.imu_published - p.imu_published) / dt));
        kv(st,
           "Segments per frame",
           std::to_string(std::popcount(d.expected_mask)));
        kv(st, "Telegrams lost (last period)", std::to_string(lost));
        kv(st, "Partial frames (last period)", std::to_string(partial));
        kv(st, "Discarded frames (last period)", std::to_string(discarded));
        kv(st, "Decode errors (last period)", std::to_string(errors));
        kv(st, "Frames published", std::to_string(d.frames_published));
        kv(st, "Frames complete", std::to_string(d.stream.frames_complete));
        kv(st, "Frames partial", std::to_string(d.stream.frames_partial));
        kv(st, "Frames missed", std::to_string(d.stream.frames_missed));
        kv(st, "Telegrams lost", std::to_string(d.stream.telegrams_lost));
        kv(st,
           "Duplicate/late segments",
           std::to_string(
               d.stream.segments_duplicate + d.stream.segments_late));
        kv(st, "Decode errors", std::to_string(d.stream.decode_errors));
        kv(st,
           "Truncated datagrams",
           std::to_string(this->receiver.truncatedDatagrams()));
        msg->status.push_back(std::move(st));
    }

    // --- time
    {
        static constexpr const char* SOURCES[] = {
            "estimated",
            "sensor",
            "receive"};
        static constexpr const char* SYNC[] = {"off", "ntp", "?", "ptp"};
        Status st = make(
            "Time",
            Status::OK,
            SOURCES[static_cast<int>(this->config.time_source)]);
        const double offset_s = d.clock_offset_ns * 1e-9;
        if (!d.clock_valid)
        {
            st.level = Status::STALE;
            st.message += ", no data yet";
        }
        else if (
            this->config.time_source == TimeSource::Sensor &&
            std::abs(d.clock_offset_ns) > 50'000'000)
        {
            st.level = Status::WARN;
            st.message += ": sensor clock is " + fmt("%.3f", offset_s) +
                          " s behind the host; is it synchronised?";
        }
        else if (
            d.clock.restarts != p.clock.restarts ||
            d.clock.steps != p.clock.steps)
        {
            st.level = Status::WARN;
            st.message += ", clock jumped";
        }
        kv(st, "Source", SOURCES[static_cast<int>(this->config.time_source)]);
        kv(st,
           "Sensor time sync",
           SYNC[static_cast<int>(this->config.sensor.time_sync.mode)]);
        kv(st, "Host - sensor offset [s]", fmt("%.6f", offset_s));
        kv(st, "Drift [ppm]", fmt("%.2f", d.clock_drift * 1e6));
        kv(st,
           "Delay above estimate, min [us]",
           fmt("%.0f", d.clock.block_min_delay_ns * 1e-3));
        kv(st,
           "Delay above estimate, max [us]",
           fmt("%.0f", d.clock.block_max_delay_ns * 1e-3));
        kv(st, "Clock restarts", std::to_string(d.clock.restarts));
        kv(st, "Clock steps", std::to_string(d.clock.steps));
        msg->status.push_back(std::move(st));
    }

    // --- configuration
    {
        Status st = make("Configuration", Status::OK, "");
        if (d.settings_total == 0)
        {
            st.level = Status::STALE;
            st.message = "not checked yet";
        }
        else
        {
            st.message = std::to_string(d.settings_verified) + " of " +
                         std::to_string(d.settings_total) +
                         " settings verified";
            if (!d.setting_problems.empty())
            {
                st.level = Status::WARN;
                st.message += ", " + std::to_string(d.setting_problems.size()) +
                              " problems";
            }
        }
        kv(st,
           "Mode",
           this->config.apply_sensor_settings ? "apply" : "compare only");
        for (const auto& pr : d.setting_problems)
        {
            const size_t colon = pr.find(':');
            kv(st,
               pr.substr(0, colon),
               colon == std::string::npos ? "" : pr.substr(colon + 2));
        }
        msg->status.push_back(std::move(st));
    }

    this->diag_pub->publish(std::move(msg));
    this->diag_previous = std::move(d);
    this->diag_previous_time = now;
}

void MultiscanNode::runReceiver()
{
    RCLCPP_INFO(
        this->get_logger(),
        "[MULTISCAN DRIVER]: Initializing connections using the following parameters:"
        "\n\tLidar IP address: %s"
        "\n\tDriver IP address: %s"
        "\n\tLidar UDP port: %d"
        "\n\tSOPAS TCP port: %d (CoLa A)",
        this->config.lidar_hostname.c_str(),
        this->config.driver_hostname.empty()
            ? "(auto)"
            : this->config.driver_hostname.c_str(),
        this->config.lidar_udp_port,
        this->config.sopas_tcp_port);

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
            auto last_sopas_check = last_udp_recv_time;
            auto last_status_poll =
                last_udp_recv_time - std::chrono::hours(1);  // poll at once
            this->assembler->reset();

            while (this->is_running && this->cola.connected())
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
                if (now - last_sopas_check > std::chrono::seconds(1))
                {
                    // handles events and notices a closed connection
                    this->cola.poll(0);
                    this->updateDiagnosticsSnapshot(
                        received_data &&
                        std::chrono::duration<double>(now - last_udp_recv_time)
                                .count() <
                            this->config.udp_dropout_reset_thresh);
                    last_sopas_check = now;
                }
                if (this->config.status_period_s > 0. &&
                    std::chrono::duration<double>(now - last_status_poll)
                            .count() >= this->config.status_period_s)
                {
                    if (!this->pollSensorStatus())
                    {
                        break;
                    }
                    last_status_poll = now;
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

        const bool sopas_lost = !this->cola.connected();
        this->stopStreaming();
        {
            std::lock_guard l(this->diag_mtx);
            this->diag.connected = false;
            this->diag.streaming = false;
            if (this->diag.last_error.empty())
            {
                this->diag.last_error =
                    sopas_lost ? "lost the SOPAS connection" : "no lidar data";
            }
        }

        if (this->is_running)
        {
            if (sopas_lost)
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
