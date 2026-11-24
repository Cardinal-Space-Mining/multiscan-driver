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

#include "node/multiscan_node.hpp"

#include <algorithm>
#include <filesystem>

#include <ament_index_cpp/get_package_share_path.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>


namespace multiscan
{

namespace
{

constexpr const char* LOG_PREFIX = "[MULTISCAN DRIVER]: ";

builtin_interfaces::msg::Time toStamp(uint64_t time_ns)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(time_ns / 1000000000ULL);
    stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000ULL);
    return stamp;
}

/* A model file parameter is a path, or the name of a model the driver
 * installs: "artifact_full" -> share/multiscan_driver/models/
 * artifact_full.model.txt. */
std::string resolveModelPath(const std::string& value)
{
    const bool is_name = !value.empty() &&
                         value.find('/') == std::string::npos &&
                         std::filesystem::path(value).extension().empty();
    if (!is_name)
    {
        return value;
    }
    try
    {
        return (ament_index_cpp::get_package_share_path("multiscan_driver") /
                "models" / (value + ".model.txt"))
            .string();
    }
    catch (const std::exception&)
    {
        return value;  // not installed: try it as a path
    }
}

}  // namespace



// --- Setup -------------------------------------------------------------------

MultiscanNode::MultiscanNode(bool autostart) :
    Node("multiscan_driver"),
    config{loadDriverConfig(*this)}
{
    this->createPipeline();
    this->startGhostFilter();
    this->createPublishers();
    if (autostart)
    {
        this->start();
    }
}

MultiscanNode::~MultiscanNode() { this->shutdown(); }

void MultiscanNode::start()
{
    this->is_running = true;
    if (!this->receive_thread.joinable())
    {
        this->receive_thread = std::thread{&MultiscanNode::runReceiver, this};
    }
}

void MultiscanNode::shutdown()
{
    this->is_running = false;
    this->receiver.stop();
    if (this->receive_thread.joinable())
    {
        this->receive_thread.join();
    }
}

/* Point layout, decoder and frame assembler. */
void MultiscanNode::createPipeline()
{
    std::string layout_error;
    std::optional<PointLayout> layout =
        PointLayout::fromNames(this->config.output.point_fields, layout_error);
    if (!layout)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "%sInvalid output.point_fields (%s); publishing all fields.",
            LOG_PREFIX,
            layout_error.c_str());
        layout = PointLayout::all();
    }
    for (const FieldDescriptor& field : layout->fields())
    {
        this->point_fields.push_back(
            PointFieldMsg{}
                .set__name(field.name)
                .set__offset(field.offset)
                .set__datatype(field.datatype)
                .set__count(1));
    }

    const uint32_t echo_selection =
        static_cast<uint32_t>(std::max(0, this->config.output.echo_selection));
    this->decoder = std::make_unique<compact::ScanDecoder>(
        *layout,
        compact::DecodeOptions{.echo_selection = echo_selection});
    this->assembler = std::make_unique<FrameAssembler>(
        *this->decoder,
        FrameAssembler::Config{
            .publish_partial_frames =
                this->config.output.publish_partial_frames},
        [this](const compact::ScanInfo& info, uint64_t receive_time_ns)
        { return this->clockOffset(info.transmit_time_us, receive_time_ns); });
}

void MultiscanNode::createPublishers()
{
    this->scan_publisher = this->create_publisher<PointCloudMsg>(
        "lidar_scan",
        rclcpp::SensorDataQoS{});
    this->imu_publisher =
        this->create_publisher<ImuMsg>("lidar_imu", rclcpp::SensorDataQoS{});
    this->publishImuTransform();

    this->diagnostics_reporter =
        std::make_unique<DiagnosticsReporter>(this->config);
    this->diagnostics_publisher = this->create_publisher<DiagnosticArrayMsg>(
        "/diagnostics",
        rclcpp::SystemDefaultsQoS{});
    this->diagnostics_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { this->publishDiagnostics(); });

#if PUBLISH_PROCESS_METRICS
    this->process_stats_publisher = this->create_publisher<ProcessStatsMsg>(
        "multiscan_driver/process_stats",
        rclcpp::SensorDataQoS{});
    this->process_stats_timer = this->create_wall_timer(
        PROCESS_STATS_PERIOD,
        [this]()
        {
            this->process_stats.update();
            this->process_stats_publisher->publish(this->process_stats.toMsg());
        });
#endif
}

/* With its own frame, the IMU gets a static transform from the lidar
 * frame (unless imu.publish_tf is off). */
void MultiscanNode::publishImuTransform()
{
    DriverConfig::Imu& imu = this->config.imu;
    const std::string& lidar_frame = this->config.output.lidar_frame_id;
    if (imu.frame_id.empty())
    {
        imu.frame_id = lidar_frame;
        return;
    }
    if (!imu.publish_tf || imu.frame_id == lidar_frame)
    {
        return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = lidar_frame;
    transform.child_frame_id = imu.frame_id;
    transform.transform.translation.x = IMU_OFFSET_M[0];
    transform.transform.translation.y = IMU_OFFSET_M[1];
    transform.transform.translation.z = IMU_OFFSET_M[2];
    transform.transform.rotation.w = 1.;  // IMU axes are the device axes
    this->imu_transform =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    this->imu_transform->sendTransform(transform);
}



// --- Ghost filter ------------------------------------------------------------

void MultiscanNode::startGhostFilter()
{
    const DriverConfig::GhostFilterSettings& settings =
        this->config.ghost_filter;
    if (!settings.enabled)
    {
        return;
    }
    if (this->config.sensor.interlaced)
    {
        this->failGhostFilter(
            "sensor.interlaced is on; the model assumes the same beam "
            "directions in every frame");
        return;
    }
    std::string model_path;
    std::unique_ptr<GhostFilter> filter = this->loadGhostFilter(model_path);
    if (!filter)
    {
        return;
    }

    const artifact_filter::Model& model = filter->model();
    const bool dropping = settings.mode == GhostFilter::Mode::Drop;
    RCLCPP_INFO(
        this->get_logger(),
        "%sGhost filter: %s model (%s, %zu trees, trained %s) from %s; %s "
        "points from probability %.2f%s.",
        LOG_PREFIX,
        model.variant.c_str(),
        model.use_intensity ? "range and intensity" : "range only",
        model.tree_roots.size(),
        model.trained.c_str(),
        model_path.c_str(),
        dropping ? "dropping" : "annotating",
        filter->threshold(),
        dropping ? ""
                 : " (field 'artifact': probability, -1 = not classified)");
    if (!filter->earlyExitActive())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "%sGhost filter runs without early exit (%s): on one core a frame "
            "can take longer than the 50 ms scan period, and frames are then "
            "published unfiltered.",
            LOG_PREFIX,
            settings.early_exit ? "the model has no checkpoints"
                                : "ghost_filter.early_exit is false");
    }
    this->ghost_filter_model = model.variant + " (" + model_path + ")";

    this->ghost_filter = std::make_unique<GhostFilterWorker>(
        std::move(filter),
        this->decoder->layout(),
        [this](FrameAssembler::Frame& frame, const GhostFilter::Result&)
        { this->publishFrame(frame); });
    this->assembler->setSegmentHook(
        [this](
            const compact::ScanInfo& info,
            std::span<const uint8_t> datagram,
            uint64_t frame_number,
            uint32_t first_point)
        {
            this->ghost_filter
                ->segment(info, datagram, frame_number, first_point);
        });
}

/* The model file, or else the fallback. */
std::unique_ptr<GhostFilter> MultiscanNode::loadGhostFilter(
    std::string& model_path)
{
    const DriverConfig::GhostFilterSettings& settings =
        this->config.ghost_filter;
    const GhostFilter::Config filter_config{
        .mode = settings.mode,
        .threshold = settings.threshold,
        .early_exit = settings.early_exit,
        .echo_selection = static_cast<uint32_t>(
            std::max(0, this->config.output.echo_selection))};

    std::string errors;
    for (const std::string& parameter :
         {settings.model_file, settings.fallback_model_file})
    {
        if (parameter.empty())
        {
            continue;
        }
        const std::string path = resolveModelPath(parameter);
        try
        {
            auto filter = std::make_unique<GhostFilter>(
                artifact_filter::Model::load(path),
                filter_config);
            model_path = path;
            return filter;
        }
        catch (const std::exception& error)
        {
            errors += (errors.empty() ? "" : "; ") + std::string(error.what());
            RCLCPP_WARN(
                this->get_logger(),
                "%sGhost filter: cannot use %s: %s",
                LOG_PREFIX,
                path.c_str(),
                error.what());
        }
    }
    this->failGhostFilter(
        errors.empty() ? "ghost_filter.model_file is not set" : errors);
    return nullptr;
}

void MultiscanNode::failGhostFilter(const std::string& reason)
{
    this->ghost_filter_error = reason;
    RCLCPP_ERROR(
        this->get_logger(),
        "%sGhost filter disabled: %s",
        LOG_PREFIX,
        reason.c_str());
}



// --- Publishing --------------------------------------------------------------

void MultiscanNode::publishFrame(FrameAssembler::Frame& frame)
{
    auto scan = std::make_unique<PointCloudMsg>();
    scan->data = std::move(frame.data);  // no copy
    scan->fields = this->point_fields;
    scan->is_bigendian = false;
    scan->point_step = this->decoder->layout().pointStep();
    scan->row_step = static_cast<uint32_t>(scan->data.size());
    scan->height = 1;
    scan->width = frame.num_points;
    scan->is_dense = true;
    scan->header.frame_id = this->config.output.lidar_frame_id;
    scan->header.stamp = toStamp(frame.stamp_ns);

    this->scan_publisher->publish(std::move(scan));
    this->frames_published++;
}

void MultiscanNode::publishImu(
    const compact::ImuSample& sample,
    uint64_t receive_time_ns)
{
    // IMU telegrams carry no transmit time, only the sample time: they use
    // the offset estimated from the scan telegrams
    uint64_t stamp_ns = receive_time_ns;
    if (this->config.time_source == TimeSource::Sensor)
    {
        stamp_ns = sample.time_us * 1000;
    }
    else if (
        this->config.time_source == TimeSource::Estimated &&
        this->clock_estimator.valid())
    {
        stamp_ns =
            this->clock_estimator.toHostNs(sample.time_us, receive_time_ns);
    }

    const DriverConfig::Imu& imu = this->config.imu;
    auto message = std::make_unique<ImuMsg>();
    message->header.stamp = toStamp(stamp_ns);
    message->header.frame_id = imu.frame_id;
    for (int axis = 0; axis < 3; axis++)
    {
        // diagonal of the 3 x 3 covariance matrices
        message->orientation_covariance[axis * 4] =
            imu.orientation_covariance[axis];
        message->angular_velocity_covariance[axis * 4] =
            imu.angular_velocity_covariance[axis];
        message->linear_acceleration_covariance[axis * 4] =
            imu.linear_acceleration_covariance[axis];
    }
    message->angular_velocity.x = sample.angular_velocity[0];
    message->angular_velocity.y = sample.angular_velocity[1];
    message->angular_velocity.z = sample.angular_velocity[2];
    message->linear_acceleration.x = sample.acceleration[0];
    message->linear_acceleration.y = sample.acceleration[1];
    message->linear_acceleration.z = sample.acceleration[2];
    message->orientation.w = sample.orientation[0];
    message->orientation.x = sample.orientation[1];
    message->orientation.y = sample.orientation[2];
    message->orientation.z = sample.orientation[3];
    this->imu_publisher->publish(std::move(message));
    this->imu_samples_published++;
}

void MultiscanNode::publishDiagnostics()
{
    DiagnosticsSnapshot snapshot;
    {
        std::lock_guard lock{this->diagnostics_mutex};
        snapshot = this->diagnostics;
    }
    snapshot.truncated_datagrams = this->receiver.truncatedDatagrams();

    DiagnosticsSnapshot::GhostFilterState& ghost = snapshot.ghost_filter;
    ghost.running = this->ghost_filter != nullptr;
    ghost.error = this->ghost_filter_error;
    if (this->ghost_filter)
    {
        const GhostFilter& filter = this->ghost_filter->filter();
        ghost.model = this->ghost_filter_model;
        ghost.mode = filter.config().mode;
        ghost.threshold = filter.threshold();
        ghost.early_exit_active = filter.earlyExitActive();
        ghost.model_has_checkpoints = filter.model().hasEarlyExit();
        ghost.stats = this->ghost_filter->takeStats();
    }

    this->diagnostics_publisher->publish(
        this->diagnostics_reporter->build(snapshot, this->now()));
}

}  // namespace multiscan
