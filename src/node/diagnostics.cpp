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

#include "node/diagnostics.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>


namespace multiscan
{

namespace
{

using StatusMsg = diagnostic_msgs::msg::DiagnosticStatus;

void addValue(
    StatusMsg& status,
    const std::string& key,
    const std::string& value)
{
    status.values.push_back(
        diagnostic_msgs::msg::KeyValue{}.set__key(key).set__value(value));
}

std::string formatNumber(const char* format, double value)
{
    char text[64];
    std::snprintf(text, sizeof(text), format, value);
    return std::string(text);
}

const char* yesNo(bool value) { return value ? "true" : "false"; }

const char* timeSyncName(sopas::TimeSyncMode mode)
{
    switch (mode)
    {
        case sopas::TimeSyncMode::Off:
            return "off";
        case sopas::TimeSyncMode::Ntp:
            return "ntp";
        case sopas::TimeSyncMode::Ptp:
            return "ptp";
    }
    return "?";
}

/* A per-frame mean over the window of the ghost filter stats. */
double perFrame(uint64_t total, uint64_t frames)
{
    return frames ? static_cast<double>(total) / static_cast<double>(frames)
                  : 0.;
}

}  // namespace



DiagnosticsReporter::DiagnosticsReporter(const DriverConfig& config) :
    config{config},
    previous_time{std::chrono::steady_clock::now()}
{
}

DiagnosticsReporter::DiagnosticArrayMsg DiagnosticsReporter::build(
    const DiagnosticsSnapshot& current,
    const builtin_interfaces::msg::Time& stamp)
{
    const auto now = std::chrono::steady_clock::now();
    const Period period{
        .current = current,
        .previous = this->previous,
        .seconds = std::max(
            1e-3,
            std::chrono::duration<double>(now - this->previous_time).count())};
    this->hardware_id = current.device.serial.empty()
                            ? this->config.connection.lidar_hostname
                            : current.device.type + " " + current.device.serial;

    DiagnosticArrayMsg message;
    message.header.stamp = stamp;
    message.status.push_back(this->connectionStatus(period));
    message.status.push_back(this->sensorStatus(period));
    message.status.push_back(this->contaminationStatus(period));
    message.status.push_back(this->streamStatus(period));
    message.status.push_back(this->timeStatus(period));
    if (this->config.ghost_filter.enabled)
    {
        message.status.push_back(this->ghostFilterStatus(period));
    }
    message.status.push_back(this->configurationStatus(period));

    this->previous = current;
    this->previous_time = now;
    return message;
}

StatusMsg DiagnosticsReporter::makeStatus(
    const char* name,
    uint8_t level,
    std::string message) const
{
    StatusMsg status;
    status.name = std::string("multiscan_driver: ") + name;
    status.hardware_id = this->hardware_id;
    status.level = level;
    status.message = std::move(message);
    return status;
}



// --- Statuses ----------------------------------------------------------------

StatusMsg DiagnosticsReporter::connectionStatus(const Period& period) const
{
    const DiagnosticsSnapshot& state = period.current;
    StatusMsg status =
        this->makeStatus("Connection", StatusMsg::OK, "streaming");
    if (!state.streaming && state.connected)
    {
        status.level = StatusMsg::WARN;
        status.message = "connected, no data";
    }
    else if (!state.streaming)
    {
        status.level = StatusMsg::ERROR;
        status.message =
            state.last_error.empty() ? "not connected" : state.last_error;
    }
    addValue(status, "Sensor", this->config.connection.lidar_hostname);
    addValue(status, "Stream destination", state.stream_host);
    addValue(status, "Device", state.device.type);
    addValue(status, "Serial", state.device.serial);
    addValue(status, "Firmware", state.device.firmware);
    addValue(status, "Connects", std::to_string(state.connects));
    if (!state.last_error.empty())
    {
        addValue(status, "Last error", state.last_error);
    }
    return status;
}

/* Device state and temperature. */
StatusMsg DiagnosticsReporter::sensorStatus(const Period& period) const
{
    using DeviceState = sopas::SensorStatus::DeviceState;
    const DiagnosticsSnapshot& state = period.current;
    StatusMsg status =
        this->makeStatus("Sensor", StatusMsg::STALE, "no status read yet");
    if (state.has_status && state.status.has_state)
    {
        const int device_state = state.status.state;
        const bool is_error =
            device_state == static_cast<int>(DeviceState::RecoverableError) ||
            device_state == static_cast<int>(DeviceState::FatalError);
        status.level = device_state == static_cast<int>(DeviceState::Normal)
                           ? StatusMsg::OK
                       : is_error ? StatusMsg::ERROR
                                  : StatusMsg::WARN;
        status.message = sopas::deviceStateName(device_state);
        addValue(
            status,
            "Device state",
            std::to_string(device_state) + " (" +
                sopas::deviceStateName(device_state) + ")");
    }
    if (state.has_status && state.status.has_temperature)
    {
        addValue(
            status,
            "Internal temperature [C]",
            formatNumber("%.1f", state.status.temperature_c));
    }
    return status;
}

StatusMsg DiagnosticsReporter::contaminationStatus(const Period& period) const
{
    const DiagnosticsSnapshot& state = period.current;
    const bool monitoring = this->config.sensor.contamination.strategy !=
                            sopas::ContaminationStrategy::Inactive;
    StatusMsg status = this->makeStatus(
        "Contamination",
        StatusMsg::STALE,
        "no status read yet");
    if (state.has_status && state.status.has_contamination)
    {
        if (state.status.contamination_error)
        {
            status.level = StatusMsg::ERROR;
            status.message = "contaminated (error): clean the optics cover";
        }
        else if (state.status.contamination_warning)
        {
            status.level = StatusMsg::WARN;
            status.message = "contaminated (warning)";
        }
        else
        {
            status.level = StatusMsg::OK;
            status.message = monitoring ? "clean" : "monitoring inactive";
        }
        addValue(status, "Warning", yesNo(state.status.contamination_warning));
        addValue(status, "Error", yesNo(state.status.contamination_error));
    }
    if (state.has_status && state.status.has_sectors)
    {
        for (size_t i = 0; i < state.status.sectors.size(); i++)
        {
            addValue(
                status,
                "Sector " + std::to_string(i + 1),
                sopas::sectorName(state.status.sectors[i]));
        }
    }
    return status;
}

/* Rates over the last period, and totals. */
StatusMsg DiagnosticsReporter::streamStatus(const Period& period) const
{
    const DiagnosticsSnapshot& state = period.current;
    const FrameAssembler::Stats& stream = state.stream;
    const FrameAssembler::Stats& before = period.previous.stream;
    const double frames_per_second =
        (state.frames_published - period.previous.frames_published) /
        period.seconds;
    const double imu_per_second =
        (state.imu_samples_published - period.previous.imu_samples_published) /
        period.seconds;
    const uint64_t lost = stream.telegrams_lost - before.telegrams_lost;
    const uint64_t partial = stream.frames_partial - before.frames_partial;
    const uint64_t discarded =
        stream.frames_discarded - before.frames_discarded;
    const uint64_t errors = stream.decode_errors - before.decode_errors;

    StatusMsg status = this->makeStatus(
        "Stream",
        StatusMsg::OK,
        formatNumber("%.1f frames/s", frames_per_second));
    if (!state.streaming)
    {
        status.level = StatusMsg::ERROR;
        status.message = "no data";
    }
    else if (lost || partial || discarded || errors)
    {
        status.level = StatusMsg::WARN;
        status.message += ", degraded";
    }

    addValue(status, "Frames/s", formatNumber("%.1f", frames_per_second));
    addValue(status, "IMU samples/s", formatNumber("%.1f", imu_per_second));
    addValue(
        status,
        "Segments per frame",
        std::to_string(std::popcount(state.expected_mask)));
    addValue(status, "Telegrams lost (last period)", std::to_string(lost));
    addValue(status, "Partial frames (last period)", std::to_string(partial));
    addValue(
        status,
        "Discarded frames (last period)",
        std::to_string(discarded));
    addValue(status, "Decode errors (last period)", std::to_string(errors));
    addValue(
        status,
        "Frames published",
        std::to_string(state.frames_published));
    addValue(status, "Frames complete", std::to_string(stream.frames_complete));
    addValue(status, "Frames partial", std::to_string(stream.frames_partial));
    addValue(status, "Frames missed", std::to_string(stream.frames_missed));
    addValue(status, "Telegrams lost", std::to_string(stream.telegrams_lost));
    addValue(
        status,
        "Duplicate/late segments",
        std::to_string(stream.segments_duplicate + stream.segments_late));
    addValue(status, "Decode errors", std::to_string(stream.decode_errors));
    addValue(
        status,
        "Truncated datagrams",
        std::to_string(state.truncated_datagrams));
    return status;
}

StatusMsg DiagnosticsReporter::timeStatus(const Period& period) const
{
    const DiagnosticsSnapshot& state = period.current;
    const char* source = timeSourceName(this->config.time_source);
    const double offset_s = state.clock_offset_ns * 1e-9;
    StatusMsg status = this->makeStatus("Time", StatusMsg::OK, source);
    if (!state.clock_valid)
    {
        status.level = StatusMsg::STALE;
        status.message += ", no data yet";
    }
    else if (
        this->config.time_source == TimeSource::Sensor &&
        std::abs(state.clock_offset_ns) > 50'000'000)
    {
        status.level = StatusMsg::WARN;
        status.message += ": sensor clock is " +
                          formatNumber("%.3f", offset_s) +
                          " s behind the host; is it synchronised?";
    }
    else if (
        state.clock.restarts != period.previous.clock.restarts ||
        state.clock.steps != period.previous.clock.steps)
    {
        status.level = StatusMsg::WARN;
        status.message += ", clock jumped";
    }

    addValue(status, "Source", source);
    addValue(
        status,
        "Sensor time sync",
        timeSyncName(this->config.sensor.time_sync.mode));
    addValue(
        status,
        "Host - sensor offset [s]",
        formatNumber("%.6f", offset_s));
    addValue(
        status,
        "Drift [ppm]",
        formatNumber("%.2f", state.clock_drift * 1e6));
    addValue(
        status,
        "Delay above estimate, min [us]",
        formatNumber("%.0f", state.clock.block_min_delay_ns * 1e-3));
    addValue(
        status,
        "Delay above estimate, max [us]",
        formatNumber("%.0f", state.clock.block_max_delay_ns * 1e-3));
    addValue(status, "Clock restarts", std::to_string(state.clock.restarts));
    addValue(status, "Clock steps", std::to_string(state.clock.steps));
    return status;
}

StatusMsg DiagnosticsReporter::ghostFilterStatus(const Period& period) const
{
    const DiagnosticsSnapshot::GhostFilterState& ghost =
        period.current.ghost_filter;
    StatusMsg status = this->makeStatus("Ghost filter", StatusMsg::OK, "");
    if (!ghost.running)
    {
        status.level = StatusMsg::ERROR;
        status.message = "not running: " + ghost.error;
        return status;
    }

    const GhostFilterWorker::Stats& stats = ghost.stats;
    const GhostFilterWorker::Stats& before = period.previous.ghost_filter.stats;
    const uint64_t frames = stats.window_frames;
    const uint64_t unfiltered =
        stats.frames_unfiltered - before.frames_unfiltered;
    const uint64_t overloaded =
        stats.frames_overloaded - before.frames_overloaded;
    const double flagged_per_frame =
        perFrame(stats.points_flagged - before.points_flagged, frames);
    const double classified_per_frame =
        perFrame(stats.points_classified - before.points_classified, frames);
    const double delay_ms = perFrame(stats.window_delay_sum_ns, frames) * 1e-6;
    const double work_ms = perFrame(stats.window_work_sum_ns, frames) * 1e-6;

    status.message =
        formatNumber("%.1f points flagged per frame", flagged_per_frame) +
        formatNumber(", +%.1f ms", delay_ms);
    if (overloaded)
    {
        status.level = StatusMsg::WARN;
        status.message += ", too slow: " + std::to_string(overloaded) +
                          " frames published unfiltered";
    }
    else if (unfiltered)
    {
        status.level = StatusMsg::WARN;
        status.message +=
            ", " + std::to_string(unfiltered) + " frames unfiltered";
    }
    else if (!frames && period.current.streaming)
    {
        status.level = StatusMsg::WARN;
        status.message = "no frames filtered";
    }

    addValue(status, "Model", ghost.model);
    addValue(
        status,
        "Mode",
        ghost.mode == GhostFilter::Mode::Drop ? "drop" : "annotate");
    addValue(status, "Threshold", formatNumber("%.3f", ghost.threshold));
    addValue(
        status,
        "Early exit",
        !ghost.model_has_checkpoints
            ? "no (model has no checkpoints)"
            : (ghost.early_exit_active ? "yes" : "no"));
    addValue(status, "Frames filtered (last period)", std::to_string(frames));
    addValue(
        status,
        "Points flagged per frame",
        formatNumber("%.1f", flagged_per_frame));
    addValue(
        status,
        "Points classified per frame",
        formatNumber("%.0f", classified_per_frame));
    addValue(status, "Delay added, mean [ms]", formatNumber("%.2f", delay_ms));
    addValue(
        status,
        "Delay added, max [ms]",
        formatNumber("%.2f", stats.window_delay_max_ns * 1e-6));
    addValue(
        status,
        "Work per frame, mean [ms]",
        formatNumber("%.2f", work_ms));
    addValue(
        status,
        "Work per frame, max [ms]",
        formatNumber("%.2f", stats.window_work_max_ns * 1e-6));
    addValue(status, "Frames filtered", std::to_string(stats.frames_filtered));
    addValue(
        status,
        "Frames unfiltered",
        std::to_string(stats.frames_unfiltered));
    addValue(
        status,
        "Frames unfiltered (too slow)",
        std::to_string(stats.frames_overloaded));
    return status;
}

StatusMsg DiagnosticsReporter::configurationStatus(const Period& period) const
{
    const DiagnosticsSnapshot& state = period.current;
    StatusMsg status = this->makeStatus("Configuration", StatusMsg::OK, "");
    if (state.settings_total == 0)
    {
        status.level = StatusMsg::STALE;
        status.message = "not checked yet";
    }
    else
    {
        status.message = std::to_string(state.settings_verified) + " of " +
                         std::to_string(state.settings_total) +
                         " settings verified";
        if (!state.setting_problems.empty())
        {
            status.level = StatusMsg::WARN;
            status.message += ", " +
                              std::to_string(state.setting_problems.size()) +
                              " problems";
        }
    }
    addValue(
        status,
        "Mode",
        this->config.apply_sensor_settings ? "apply" : "compare only");
    // problems read "<setting>: <what>"
    for (const std::string& problem : state.setting_problems)
    {
        const size_t colon = problem.find(':');
        addValue(
            status,
            problem.substr(0, colon),
            colon == std::string::npos ? "" : problem.substr(colon + 2));
    }
    return status;
}

}  // namespace multiscan
