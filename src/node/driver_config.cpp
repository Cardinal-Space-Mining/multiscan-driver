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

#include "node/driver_config.hpp"

#include <algorithm>
#include <optional>

#include <csm_utils/ros_utils.hpp>


namespace multiscan
{

namespace
{

constexpr const char* LOG_PREFIX = "[MULTISCAN DRIVER]: ";

/* Declares and reads parameters, collecting what is wrong with them. */
class ParameterReader
{
public:
    explicit ParameterReader(rclcpp::Node& node) : node{node} {}

public:
    bool flag(const std::string& name, bool default_value)
    {
        return util::declare_and_get_param(this->node, name, default_value);
    }
    int64_t integer(const std::string& name, int64_t default_value)
    {
        return util::declare_and_get_param(this->node, name, default_value);
    }
    double real(const std::string& name, double default_value)
    {
        return util::declare_and_get_param(this->node, name, default_value);
    }
    std::string text(const std::string& name, const std::string& default_value)
    {
        return util::declare_and_get_param(this->node, name, default_value);
    }
    std::vector<std::string> texts(
        const std::string& name,
        const std::vector<std::string>& default_value)
    {
        return util::declare_and_get_param(this->node, name, default_value);
    }

    /* A list that must have as many values as the default. */
    template<typename T>
    std::vector<T> list(const std::string& name, std::vector<T> default_value)
    {
        std::vector<T> value =
            util::declare_and_get_param(this->node, name, default_value);
        if (value.size() != default_value.size())
        {
            this->problems.push_back(
                name + " needs " + std::to_string(default_value.size()) +
                " values");
            return default_value;
        }
        return value;
    }

    /* A text parameter that must be one of a set of words. */
    template<typename T, typename Parse>
    T choice(
        const std::string& name,
        const std::string& default_value,
        Parse parse,
        const std::string& allowed)
    {
        const std::string value = this->text(name, default_value);
        if (const std::optional<T> parsed = parse(value))
        {
            return *parsed;
        }
        this->problems.push_back(
            name + " must be " + allowed + " (got '" + value + "')");
        return *parse(default_value);
    }

    void report(const std::string& problem)
    {
        this->problems.push_back(problem);
    }

public:
    rclcpp::Node& node;
    std::vector<std::string> problems;
};

template<size_t N>
void toFlags(const std::vector<int64_t>& values, std::array<bool, N>& flags)
{
    for (size_t i = 0; i < N; i++)
    {
        flags[i] = values[i] != 0;
    }
}

std::string joinLines(const std::vector<std::string>& lines)
{
    std::string joined;
    for (const std::string& line : lines)
    {
        joined += "\n\t" + line;
    }
    return joined;
}


// --- Driver parameters -------------------------------------------------------

void readConnection(ParameterReader& reader, DriverConfig::Connection& out)
{
    out.lidar_hostname = reader.text("lidar_hostname", "");
    out.driver_hostname = reader.text("driver_hostname", "");
    out.lidar_udp_port = reader.integer("lidar_udp_port", 2115);
    out.imu_udp_port = reader.integer("imu_udp_port", 2116);
    out.udp_receive_buffer_bytes =
        reader.integer("udp_receive_buffer_bytes", 4 << 20);
    out.sopas_tcp_port = reader.integer("sopas_tcp_port", 2111);
    out.sopas_password = reader.text("sopas_password", "F4724744");
    out.udp_reset_timeout_s = reader.real("udp_reset_timeout", 2.);
    out.sopas_read_timeout_s = reader.real("sopas_read_timeout", 3.);
    out.error_restart_timeout_s = reader.real("error_restart_timeout", 3.);
}

void readOutput(ParameterReader& reader, DriverConfig::Output& out)
{
    out.lidar_frame_id = reader.text("lidar_frame", "lidar_link");
    out.echo_selection = reader.integer("echo_selection", 1);
    out.point_fields = reader.texts(
        "output.point_fields",
        {"x",
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
    out.publish_partial_frames =
        reader.flag("output.publish_partial_frames", true);
}

void readImu(ParameterReader& reader, DriverConfig::Imu& out)
{
    out.frame_id = reader.text("imu.frame_id", "");
    out.publish_tf = reader.flag("imu.publish_tf", true);
    const auto covariance =
        [&reader](const std::string& name, std::array<double, 3>& variances)
    {
        const std::vector<double> values =
            reader.list<double>(name, {0., 0., 0.});
        std::copy(values.begin(), values.end(), variances.begin());
    };
    covariance("imu.orientation_covariance", out.orientation_covariance);
    covariance(
        "imu.angular_velocity_covariance",
        out.angular_velocity_covariance);
    covariance(
        "imu.linear_acceleration_covariance",
        out.linear_acceleration_covariance);
}

void readGhostFilter(
    ParameterReader& reader,
    DriverConfig::GhostFilterSettings& out)
{
    out.enabled = reader.flag("ghost_filter.enabled", false);
    out.model_file = reader.text("ghost_filter.model_file", "artifact_full");
    out.fallback_model_file =
        reader.text("ghost_filter.fallback_model_file", "artifact_xyz");
    out.mode = reader.choice<GhostFilter::Mode>(
        "ghost_filter.mode",
        "drop",
        [](const std::string& value) -> std::optional<GhostFilter::Mode>
        {
            if (value == "drop")
            {
                return GhostFilter::Mode::Drop;
            }
            if (value == "annotate")
            {
                return GhostFilter::Mode::Annotate;
            }
            return std::nullopt;
        },
        "drop or annotate");
    out.threshold = reader.real("ghost_filter.threshold", -1.);
    out.early_exit = reader.flag("ghost_filter.early_exit", true);
}

TimeSource readTimeSource(ParameterReader& reader)
{
    return reader.choice<TimeSource>(
        "time.source",
        "estimated",
        [](const std::string& value) -> std::optional<TimeSource>
        {
            for (const TimeSource source :
                 {TimeSource::Estimated,
                  TimeSource::Sensor,
                  TimeSource::Receive})
            {
                if (value == timeSourceName(source))
                {
                    return source;
                }
            }
            return std::nullopt;
        },
        "estimated, sensor or receive");
}


// --- Sensor settings ---------------------------------------------------------

void readScanSettings(ParameterReader& reader, sopas::SensorSettings& out)
{
    out.echo_filter = reader.choice<sopas::EchoFilter>(
        "sensor.echo_filter",
        "last",
        sopas::echoFilterFromString,
        "first, last or all");
    toFlags(
        reader.list<int64_t>(
            "sensor.layers",
            {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1}),
        out.layers);
    const std::vector<double> range =
        reader.list<double>("sensor.output_range_deg", {-180., 180.});
    out.output_range_start_deg = range[0];
    out.output_range_stop_deg = range[1];
    out.interlaced = reader.flag("sensor.interlaced", false);
}

void readFilterSettings(ParameterReader& reader, sopas::SensorSettings& out)
{
    out.range_filter.enabled =
        reader.flag("sensor.range_filter.enabled", false);
    out.range_filter.min_mm = reader.integer("sensor.range_filter.min_mm", 0);
    out.range_filter.max_mm =
        reader.integer("sensor.range_filter.max_mm", 200000);

    out.cubic_filter.enabled =
        reader.flag("sensor.cubic_filter.enabled", false);
    const std::vector<int64_t> cubic_min = reader.list<int64_t>(
        "sensor.cubic_filter.min_mm",
        {-20000, -20000, -20000});
    const std::vector<int64_t> cubic_max = reader.list<int64_t>(
        "sensor.cubic_filter.max_mm",
        {20000, 20000, 20000});
    std::copy(
        cubic_min.begin(),
        cubic_min.end(),
        out.cubic_filter.min_mm.begin());
    std::copy(
        cubic_max.begin(),
        cubic_max.end(),
        out.cubic_filter.max_mm.begin());

    out.particle_filter = reader.flag("sensor.particle_filter", false);
    out.fog_filter = reader.flag("sensor.fog_filter", false);
    out.moving_average.enabled =
        reader.flag("sensor.moving_average.enabled", false);
    out.moving_average.depth = reader.integer("sensor.moving_average.depth", 2);
    out.interval_filter.enabled =
        reader.flag("sensor.interval_filter.enabled", false);
    out.interval_filter.factor =
        reader.integer("sensor.interval_filter.factor", 4);
}

void readContaminationSettings(
    ParameterReader& reader,
    sopas::SensorSettings::Contamination& out)
{
    out.strategy = reader.choice<sopas::ContaminationStrategy>(
        "sensor.contamination.strategy",
        "inactive",
        sopas::contaminationStrategyFromString,
        "inactive, high_availability or sensitive");
    out.response_time_s =
        reader.integer("sensor.contamination.response_time_s", 3);
    out.sensitivity = reader.choice<sopas::ContaminationSensitivity>(
        "sensor.contamination.sensitivity",
        "low",
        sopas::contaminationSensitivityFromString,
        "high, medium or low");
    out.custom_sectors =
        reader.flag("sensor.contamination.custom_sectors", true);
    toFlags(
        reader.list<int64_t>(
            "sensor.contamination.sectors",
            std::vector<int64_t>(12, 1)),
        out.sectors);
    out.warning = reader.flag("sensor.contamination.warning", true);
    out.error = reader.flag("sensor.contamination.error", true);
}

void readTimeSyncSettings(
    ParameterReader& reader,
    sopas::SensorSettings::TimeSync& out)
{
    out.mode = reader.choice<sopas::TimeSyncMode>(
        "sensor.time_sync.mode",
        "off",
        sopas::timeSyncModeFromString,
        "off, ntp or ptp");
    out.ntp_server = reader.text("sensor.time_sync.ntp_server", "");
    out.ntp_update_s = reader.integer("sensor.time_sync.ntp_update_s", 10);
}

/* Reads the sensor.* parameters. Problems switch apply_sensor_settings
 * off: the sensor's settings are then only compared. */
void readSensorSettings(rclcpp::Node& node, DriverConfig& config)
{
    ParameterReader reader{node};
    config.apply_sensor_settings = reader.flag("sensor.apply", true);
    readScanSettings(reader, config.sensor);
    readFilterSettings(reader, config.sensor);
    readContaminationSettings(reader, config.sensor.contamination);
    readTimeSyncSettings(reader, config.sensor.time_sync);
    if (const std::string problem = config.sensor.validate(); !problem.empty())
    {
        reader.report(problem);
    }

    if (!reader.problems.empty())
    {
        RCLCPP_ERROR(
            node.get_logger(),
            "%sInvalid sensor settings; they will NOT be applied, only "
            "compared with the sensor:%s",
            LOG_PREFIX,
            joinLines(reader.problems).c_str());
        config.apply_sensor_settings = false;
    }
}

}  // namespace



const char* timeSourceName(TimeSource source)
{
    switch (source)
    {
        case TimeSource::Estimated:
            return "estimated";
        case TimeSource::Sensor:
            return "sensor";
        case TimeSource::Receive:
            return "receive";
    }
    return "?";
}

DriverConfig loadDriverConfig(rclcpp::Node& node)
{
    DriverConfig config;
    ParameterReader reader{node};
    readConnection(reader, config.connection);
    readOutput(reader, config.output);
    readImu(reader, config.imu);
    readGhostFilter(reader, config.ghost_filter);
    config.time_source = readTimeSource(reader);
    config.status_period_s = reader.real("diagnostics.status_period", 2.);
    if (!reader.problems.empty())
    {
        RCLCPP_ERROR(
            node.get_logger(),
            "%sInvalid parameters, using their defaults:%s",
            LOG_PREFIX,
            joinLines(reader.problems).c_str());
    }

    readSensorSettings(node, config);

    // annotating needs somewhere to put the probabilities
    std::vector<std::string>& fields = config.output.point_fields;
    if (config.ghost_filter.enabled &&
        config.ghost_filter.mode == GhostFilter::Mode::Annotate &&
        std::find(fields.begin(), fields.end(), "artifact") == fields.end())
    {
        fields.push_back("artifact");
    }
    if (config.time_source == TimeSource::Sensor &&
        config.sensor.time_sync.mode == sopas::TimeSyncMode::Off)
    {
        RCLCPP_WARN(
            node.get_logger(),
            "%stime.source is 'sensor' but sensor.time_sync.mode is 'off': "
            "the sensor clock counts from power-up, so stamps will not be "
            "host time unless the sensor is synchronised some other way.",
            LOG_PREFIX);
    }
    return config;
}

}  // namespace multiscan
