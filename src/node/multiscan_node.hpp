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

/*  The multiscan_driver ROS node.
 *
 *  - multiscan_node.cpp: setup, publishing, diagnostics
 *  - receive_loop.cpp: the receive thread, which connects to the sensor,
 *    starts the stream, receives and assembles frames, and reconnects after
 *    errors
 *  - driver_config.cpp: the parameters
 *  - diagnostics.cpp: the /diagnostics statuses
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <csm_metrics/stats.hpp>

#include <csm_utils/ros_utils.hpp>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "ghost/ghost_filter_worker.hpp"
#include "net/udp_receiver.hpp"
#include "node/diagnostics.hpp"
#include "node/driver_config.hpp"
#include "sopas/cola_client.hpp"
#include "sopas/sensor_session.hpp"
#include "time_sync.hpp"


#ifndef PUBLISH_PROCESS_METRICS
    #define PUBLISH_PROCESS_METRICS 1
#endif


namespace multiscan
{

class MultiscanNode : public rclcpp::Node
{
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PointFieldMsg = sensor_msgs::msg::PointField;
    using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
    using ProcessStatsMsg = csm_metrics::msg::ProcessStats;
    template<typename T>
    using SharedPub = util::ros_aliases::SharedPub<T>;
    using RclTimer = util::ros_aliases::RclTimer;
    using Clock = std::chrono::steady_clock;

public:
    explicit MultiscanNode(bool autostart = true);
    ~MultiscanNode();

public:
    void start();
    void shutdown();

protected:
    // The scanner can take a few seconds to start streaming after a
    // (re)start.
    static constexpr double INITIAL_UDP_TIMEOUT_S = 5.;
    static constexpr int RECEIVE_POLL_MS = 20;
    static constexpr auto HEALTH_REPORT_PERIOD = std::chrono::seconds(5);
    static constexpr auto CONNECTION_CHECK_PERIOD = std::chrono::seconds(1);
    static constexpr auto PROCESS_STATS_PERIOD = std::chrono::milliseconds(100);
    /* IMU position relative to the optical origin (operating
     * instructions). */
    static constexpr double IMU_OFFSET_M[3] = {-0.017, 0.0235, -0.030};

protected:
    // --- Setup (multiscan_node.cpp) ------------------------------------------
    void createPipeline();
    void createPublishers();
    void publishImuTransform();
    void startGhostFilter();
    std::unique_ptr<GhostFilter> loadGhostFilter(std::string& model_path);
    void failGhostFilter(const std::string& reason);

    // --- Publishing (multiscan_node.cpp) -------------------------------------
    void publishFrame(FrameAssembler::Frame& frame);
    void publishImu(const compact::ImuSample& sample, uint64_t receive_time_ns);
    void publishDiagnostics();

    // --- Receive thread (receive_loop.cpp) -----------------------------------
    void runReceiver();
    bool openUdpSockets();
    void streamUntilError();
    void handleDatagrams(const std::vector<UdpReceiver::Datagram>& batch);
    void handleScan(
        std::span<const uint8_t> datagram,
        uint64_t receive_time_ns);
    void handleImu(std::span<const uint8_t> datagram, uint64_t receive_time_ns);
    void forwardFrames();
    /* host - sensor clock offset (ns) for a scan telegram, per time.source;
     * every telegram also feeds the clock estimator. */
    int64_t clockOffset(uint64_t transmit_time_us, uint64_t receive_time_ns);

    // --- Sensor connection (receive_loop.cpp) --------------------------------
    bool connectAndStart();
    void recordStartReport(
        const sopas::StartReport& report,
        const std::string& stream_host,
        bool started);
    void stopStreaming();
    void recordDisconnect(bool sopas_lost);
    void waitBeforeRestart();
    bool pollSensorStatus();
    void logStatusChanges(const sopas::SensorStatus& status);
    void reportStreamHealth();
    void updateDiagnosticsSnapshot(bool streaming);
    int sopasTimeoutMs() const;

protected:
    DriverConfig config;

    // --- Decoding ------------------------------------------------------------
    std::vector<PointFieldMsg> point_fields;
    std::unique_ptr<compact::ScanDecoder> decoder;
    std::unique_ptr<FrameAssembler> assembler;
    std::vector<FrameAssembler::Frame> assembled_frames;
    FrameAssembler::Stats reported_stream_stats;

    // --- Sensor --------------------------------------------------------------
    UdpReceiver receiver;
    sopas::ColaClient sopas_client;
    ClockEstimator clock_estimator;
    sopas::SensorStatus last_sensor_status;
    bool has_last_sensor_status{false};

    // --- Publishing ----------------------------------------------------------
    SharedPub<PointCloudMsg> scan_publisher;
    SharedPub<ImuMsg> imu_publisher;
    SharedPub<DiagnosticArrayMsg> diagnostics_publisher;
    RclTimer diagnostics_timer;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> imu_transform;
    std::atomic<uint64_t> frames_published{0};  // also the filter thread
    uint64_t imu_samples_published{0};

    // --- Diagnostics ---------------------------------------------------------
    std::mutex diagnostics_mutex;
    DiagnosticsSnapshot diagnostics;  // guarded by diagnostics_mutex
    std::unique_ptr<DiagnosticsReporter> diagnostics_reporter;  // timer only

    // --- Receive thread ------------------------------------------------------
    std::thread receive_thread;
    std::atomic<bool> is_running{true};

    // --- Ghost filter (ghost_filter.enabled) ---------------------------------
    // Publishes frames on its own thread. Declared after the publishers so
    // it is destroyed (stopped) first.
    std::unique_ptr<GhostFilterWorker> ghost_filter;
    std::string ghost_filter_error;  // why it isn't running although enabled
    std::string ghost_filter_model;  // description for /diagnostics

#if PUBLISH_PROCESS_METRICS
    SharedPub<ProcessStatsMsg> process_stats_publisher;
    RclTimer process_stats_timer;
    csm::metrics::ProcessStats process_stats;
#endif
};

}  // namespace multiscan
