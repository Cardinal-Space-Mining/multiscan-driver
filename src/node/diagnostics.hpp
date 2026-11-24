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

/*  What the driver reports on /diagnostics: one status per topic
 *  (connection, sensor, contamination, stream, time, ghost filter,
 *  configuration), built once a second from a snapshot of the driver state.
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "frame_assembler.hpp"
#include "ghost/ghost_filter_worker.hpp"
#include "node/driver_config.hpp"
#include "sopas/sensor_session.hpp"
#include "time_sync.hpp"


namespace multiscan
{

/* The driver state the diagnostics are built from. The receive thread keeps
 * it up to date; the diagnostics timer copies it. */
struct DiagnosticsSnapshot
{
public:
    struct GhostFilterState
    {
        bool running{false};
        std::string error;  // why it isn't running
        std::string model;  // description, e.g. "full (<path>)"
        GhostFilter::Mode mode{GhostFilter::Mode::Drop};
        double threshold{0.};
        bool early_exit_active{false};
        bool model_has_checkpoints{false};
        GhostFilterWorker::Stats stats;
    };

public:
    // --- Connection ---
    bool connected{false};
    bool streaming{false};  // data arrived within udp_reset_timeout
    uint64_t connects{0};
    std::string last_error;
    sopas::DeviceInfo device;
    std::string stream_host;

    // --- Configuration ---
    size_t settings_total{0};
    size_t settings_verified{0};
    std::vector<std::string> setting_problems;

    // --- Sensor status ---
    bool has_status{false};
    sopas::SensorStatus status;

    // --- Stream ---
    FrameAssembler::Stats stream;
    uint64_t frames_published{0};
    uint64_t imu_samples_published{0};
    uint32_t expected_mask{0};
    uint64_t truncated_datagrams{0};

    // --- Time ---
    bool clock_valid{false};
    int64_t clock_offset_ns{0};
    double clock_drift{0.};
    ClockEstimator::Stats clock;

    GhostFilterState ghost_filter;
};


class DiagnosticsReporter
{
public:
    using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;
    using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;

public:
    explicit DiagnosticsReporter(const DriverConfig& config);

public:
    /* The statuses for `current`. Rates and "last period" values are
     * against the snapshot of the previous call. */
    DiagnosticArrayMsg build(
        const DiagnosticsSnapshot& current,
        const builtin_interfaces::msg::Time& stamp);

protected:
    /* The current and the previous snapshot, and the time between them. */
    struct Period
    {
        const DiagnosticsSnapshot& current;
        const DiagnosticsSnapshot& previous;
        double seconds;
    };

protected:
    DiagnosticStatusMsg connectionStatus(const Period& period) const;
    DiagnosticStatusMsg sensorStatus(const Period& period) const;
    DiagnosticStatusMsg contaminationStatus(const Period& period) const;
    DiagnosticStatusMsg streamStatus(const Period& period) const;
    DiagnosticStatusMsg timeStatus(const Period& period) const;
    DiagnosticStatusMsg ghostFilterStatus(const Period& period) const;
    DiagnosticStatusMsg configurationStatus(const Period& period) const;

    DiagnosticStatusMsg
        makeStatus(const char* name, uint8_t level, std::string message) const;

protected:
    const DriverConfig& config;
    std::string hardware_id;
    DiagnosticsSnapshot previous;
    std::chrono::steady_clock::time_point previous_time;
};

}  // namespace multiscan
