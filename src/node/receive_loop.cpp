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

#include <bit>
#include <ctime>

#include <csm_metrics/profiling.hpp>

#include "node/multiscan_node.hpp"


namespace multiscan
{

namespace
{

constexpr const char* LOG_PREFIX = "[MULTISCAN DRIVER]: ";

uint64_t realtimeNowNs()
{
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(now.tv_nsec);
}

double secondsBetween(
    std::chrono::steady_clock::time_point earlier,
    std::chrono::steady_clock::time_point later)
{
    return std::chrono::duration<double>(later - earlier).count();
}

}  // namespace



// --- Receive thread ----------------------------------------------------------

void MultiscanNode::runReceiver()
{
    const DriverConfig::Connection& connection = this->config.connection;
    RCLCPP_INFO(
        this->get_logger(),
        "%sInitializing connections using the following parameters:"
        "\n\tLidar IP address: %s"
        "\n\tDriver IP address: %s"
        "\n\tLidar UDP port: %d"
        "\n\tSOPAS TCP port: %d (CoLa A)",
        LOG_PREFIX,
        connection.lidar_hostname.c_str(),
        connection.driver_hostname.empty() ? "(auto)"
                                           : connection.driver_hostname.c_str(),
        connection.lidar_udp_port,
        connection.sopas_tcp_port);
    if (!this->openUdpSockets())
    {
        return;
    }

    while (this->is_running)
    {
        PROFILING_NOTIFY(init_connection);
        const bool started = this->connectAndStart();
        PROFILING_NOTIFY(init_connection);
        if (started)
        {
            try
            {
                this->streamUntilError();
            }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "%sUDP decode loop encountered an exception - "
                    "what():\n\t%s",
                    LOG_PREFIX,
                    error.what());
            }
        }

        const bool sopas_lost = !this->sopas_client.connected();
        this->stopStreaming();
        this->recordDisconnect(sopas_lost);
        if (!this->is_running)
        {
            break;
        }
        RCLCPP_ERROR(
            this->get_logger(),
            "%s%sRestarting connections after timeout...",
            LOG_PREFIX,
            sopas_lost ? "Lost connection to SOPAS service. " : "");
        this->waitBeforeRestart();
    }
}

bool MultiscanNode::openUdpSockets()
{
    const DriverConfig::Connection& connection = this->config.connection;
    std::string error;
    const std::vector<uint16_t> ports{
        static_cast<uint16_t>(connection.lidar_udp_port),
        static_cast<uint16_t>(connection.imu_udp_port)};
    if (!this->receiver.open(ports, connection.udp_receive_buffer_bytes, error))
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "%sCannot open UDP sockets: %s",
            LOG_PREFIX,
            error.c_str());
        return false;
    }

    const int granted_kib = this->receiver.receiveBufferBytes() / 1024;
    RCLCPP_INFO(
        this->get_logger(),
        "%sListening on UDP %d (scan) and %d (IMU), %d KiB receive buffer per "
        "socket.",
        LOG_PREFIX,
        connection.lidar_udp_port,
        connection.imu_udp_port,
        granted_kib);
    if (this->receiver.receiveBufferBytes() <
        connection.udp_receive_buffer_bytes / 2)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "%sThe kernel limited the UDP receive buffer to %d KiB (requested "
            "%d KiB); raise net.core.rmem_max to buffer more.",
            LOG_PREFIX,
            granted_kib,
            connection.udp_receive_buffer_bytes / 1024);
    }
    return true;
}

/* Receives until the stream or the SOPAS connection fails, or the node
 * stops. The TCP link can look healthy after the lidar reboots or loses
 * power, so lack of UDP data is what ends a healthy-looking stream. */
void MultiscanNode::streamUntilError()
{
    const double reset_timeout_s = this->config.connection.udp_reset_timeout_s;
    std::vector<UdpReceiver::Datagram> batch;
    bool received_data = false;
    Clock::time_point last_data_time = Clock::now();
    Clock::time_point last_health_report = last_data_time;
    Clock::time_point last_connection_check = last_data_time;
    // poll the status at once
    Clock::time_point last_status_poll = last_data_time - std::chrono::hours(1);

    this->assembler->reset();
    if (this->ghost_filter)
    {
        this->ghost_filter->reset();
    }

    while (this->is_running && this->sopas_client.connected())
    {
        PROFILING_SYNC();
        PROFILING_FLUSH();

        PROFILING_NOTIFY(receive_bytes);
        const int received = this->receiver.receive(batch, RECEIVE_POLL_MS);
        PROFILING_NOTIFY(receive_bytes);
        if (received < 0)
        {
            break;  // stopped or socket error
        }
        const Clock::time_point now = Clock::now();

        PROFILING_NOTIFY(parse_segment);
        this->handleDatagrams(batch);
        PROFILING_NOTIFY(parse_segment);

        PROFILING_NOTIFY(export_cloud);
        this->forwardFrames();
        PROFILING_NOTIFY(export_cloud);

        // --- Periodic work ---
        if (now - last_health_report > HEALTH_REPORT_PERIOD)
        {
            this->reportStreamHealth();
            last_health_report = now;
        }
        if (now - last_connection_check > CONNECTION_CHECK_PERIOD)
        {
            // handles events and notices a closed connection
            this->sopas_client.poll(0);
            this->updateDiagnosticsSnapshot(
                received_data &&
                secondsBetween(last_data_time, now) < reset_timeout_s);
            last_connection_check = now;
        }
        if (this->config.status_period_s > 0. &&
            secondsBetween(last_status_poll, now) >=
                this->config.status_period_s)
        {
            if (!this->pollSensorStatus())
            {
                break;
            }
            last_status_poll = now;
        }

        // --- Data timeout ---
        if (received > 0)
        {
            received_data = true;
            last_data_time = now;
            continue;
        }
        const double allowed_silence_s =
            received_data ? reset_timeout_s
                          : std::max(reset_timeout_s, INITIAL_UDP_TIMEOUT_S);
        const double silence_s = secondsBetween(last_data_time, now);
        if (silence_s > allowed_silence_s)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "%sNo valid lidar data for %.1f seconds.",
                LOG_PREFIX,
                silence_s);
            break;
        }
    }
}

void MultiscanNode::handleDatagrams(
    const std::vector<UdpReceiver::Datagram>& batch)
{
    for (const UdpReceiver::Datagram& datagram : batch)
    {
        switch (compact::telegramType(datagram.data))
        {
            case compact::TelegramType::Scan:
                this->handleScan(datagram.data, datagram.receive_time_ns);
                break;
            case compact::TelegramType::Imu:
                this->handleImu(datagram.data, datagram.receive_time_ns);
                break;
            default:
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "%sIgnoring unknown %zu byte datagram.",
                    LOG_PREFIX,
                    datagram.data.size());
                break;
        }
    }
    this->assembler->flushStale(realtimeNowNs(), this->assembled_frames);
}

void MultiscanNode::handleScan(
    std::span<const uint8_t> datagram,
    uint64_t receive_time_ns)
{
    const compact::Error error =
        this->assembler->add(datagram, receive_time_ns, this->assembled_frames);
    if (error != compact::Error::None)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "%sDropping scan telegram: %s.",
            LOG_PREFIX,
            compact::toString(error));
    }
}

void MultiscanNode::handleImu(
    std::span<const uint8_t> datagram,
    uint64_t receive_time_ns)
{
    compact::ImuSample sample;
    const compact::Error error = compact::decodeImu(datagram, sample);
    if (error != compact::Error::None)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "%sDropping IMU telegram: %s.",
            LOG_PREFIX,
            compact::toString(error));
        return;
    }
    this->publishImu(sample, receive_time_ns);
}

/* Assembled frames go to the ghost filter, which publishes them, or are
 * published directly. */
void MultiscanNode::forwardFrames()
{
    for (FrameAssembler::Frame& frame : this->assembled_frames)
    {
        if (this->ghost_filter)
        {
            this->ghost_filter->frame(std::move(frame));
        }
        else
        {
            this->publishFrame(frame);
        }
    }
    this->assembled_frames.clear();
}

int64_t MultiscanNode::clockOffset(
    uint64_t transmit_time_us,
    uint64_t receive_time_ns)
{
    this->clock_estimator.add(transmit_time_us, receive_time_ns);
    switch (this->config.time_source)
    {
        case TimeSource::Sensor:
            return 0;
        case TimeSource::Receive:
            return static_cast<int64_t>(receive_time_ns) -
                   static_cast<int64_t>(transmit_time_us * 1000);
        case TimeSource::Estimated:
        default:
            return this->clock_estimator.offsetNs(receive_time_ns);
    }
}



// --- Sensor connection -------------------------------------------------------

int MultiscanNode::sopasTimeoutMs() const
{
    return static_cast<int>(this->config.connection.sopas_read_timeout_s * 1e3);
}

/* Connects to SOPAS and starts the stream (sopas::SensorSession::start). */
bool MultiscanNode::connectAndStart()
{
    const DriverConfig::Connection& connection = this->config.connection;
    std::string error;
    if (!this->sopas_client.connect(
            connection.lidar_hostname,
            static_cast<uint16_t>(connection.sopas_tcp_port),
            this->sopasTimeoutMs(),
            error))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "%sSOPAS connection failed: %s",
            LOG_PREFIX,
            error.c_str());
        std::lock_guard lock{this->diagnostics_mutex};
        this->diagnostics.connected = false;
        this->diagnostics.last_error = "SOPAS connection failed: " + error;
        return false;
    }

    const std::string stream_host = connection.driver_hostname.empty()
                                        ? this->sopas_client.localAddress()
                                        : connection.driver_hostname;
    sopas::SensorSettings settings = this->config.sensor;
    if (settings.time_sync.mode == sopas::TimeSyncMode::Ntp &&
        settings.time_sync.ntp_server.empty())
    {
        settings.time_sync.ntp_server = stream_host;  // this host serves NTP
    }
    sopas::SensorSession session(
        this->sopas_client,
        connection.sopas_password,
        this->sopasTimeoutMs());
    sopas::StartReport report;
    const bool started = session.start(
        {.host = stream_host,
         .scan_port = static_cast<uint16_t>(connection.lidar_udp_port),
         .imu_port = static_cast<uint16_t>(connection.imu_udp_port),
         .imu_enabled = true},
        settings,
        this->config.apply_sensor_settings,
        report);
    this->recordStartReport(report, stream_host, started);
    return started;
}

/* Logs how the start went and keeps it for /diagnostics. */
void MultiscanNode::recordStartReport(
    const sopas::StartReport& report,
    const std::string& stream_host,
    bool started)
{
    size_t verified = 0;
    {
        std::lock_guard lock{this->diagnostics_mutex};
        DiagnosticsSnapshot& state = this->diagnostics;
        state.connected = this->sopas_client.connected();
        state.device = report.device;
        state.stream_host = stream_host;
        state.settings_total = report.settings.size();
        state.settings_verified = 0;
        state.setting_problems.clear();
        for (const sopas::SettingStatus& setting : report.settings)
        {
            state.settings_verified += setting.verified;
            if (!setting.problem.empty())
            {
                state.setting_problems.push_back(
                    setting.name + ": " + setting.problem);
            }
        }
        verified = state.settings_verified;
        if (started)
        {
            state.connects++;
            state.last_error.clear();
        }
        else
        {
            state.last_error = "sensor start failed: " + report.error;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "%sConnected to %s (serial %s, firmware %s); stream to %s:%d (scan) / "
        "%d (IMU).",
        LOG_PREFIX,
        report.device.type.c_str(),
        report.device.serial.c_str(),
        report.device.firmware.c_str(),
        stream_host.c_str(),
        this->config.connection.lidar_udp_port,
        this->config.connection.imu_udp_port);
    for (const sopas::SettingStatus& setting : report.settings)
    {
        if (!setting.problem.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "%sSensor setting %s: %s",
                LOG_PREFIX,
                setting.name.c_str(),
                setting.problem.c_str());
        }
    }
    if (!started)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "%sSensor start failed: %s",
            LOG_PREFIX,
            report.error.c_str());
        return;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "%s%s; %zu of %zu sensor settings verified, %zu problems.",
        LOG_PREFIX,
        this->config.apply_sensor_settings
            ? "Settings applied"
            : "Settings compared only (sensor.apply is false)",
        verified,
        report.settings.size(),
        report.problems());
}

void MultiscanNode::stopStreaming()
{
    if (this->sopas_client.connected())
    {
        sopas::SensorSession session(
            this->sopas_client,
            this->config.connection.sopas_password,
            this->sopasTimeoutMs());
        std::string error;
        if (!session.stop(error))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "%sStopping the sensor output failed: %s",
                LOG_PREFIX,
                error.c_str());
        }
    }
    this->sopas_client.disconnect();
}

void MultiscanNode::recordDisconnect(bool sopas_lost)
{
    std::lock_guard lock{this->diagnostics_mutex};
    this->diagnostics.connected = false;
    this->diagnostics.streaming = false;
    if (this->diagnostics.last_error.empty())
    {
        this->diagnostics.last_error =
            sopas_lost ? "lost the SOPAS connection" : "no lidar data";
    }
}

/* Sleeps error_restart_timeout in short steps, so shutdown() isn't held
 * up. */
void MultiscanNode::waitBeforeRestart()
{
    const Clock::time_point restart_time =
        Clock::now() +
        std::chrono::milliseconds(
            static_cast<int64_t>(
                this->config.connection.error_restart_timeout_s * 1e3));
    while (this->is_running && Clock::now() < restart_time)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool MultiscanNode::pollSensorStatus()
{
    sopas::SensorSession session(
        this->sopas_client,
        this->config.connection.sopas_password,
        this->sopasTimeoutMs());
    sopas::SensorStatus status;
    std::string error;
    if (!session.readStatus(status, error))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "%sReading the sensor status failed: %s",
            LOG_PREFIX,
            error.c_str());
        return false;
    }
    this->logStatusChanges(status);
    this->last_sensor_status = status;
    this->has_last_sensor_status = true;

    std::lock_guard lock{this->diagnostics_mutex};
    this->diagnostics.has_status = true;
    this->diagnostics.status = status;
    return true;
}

/* Logs changes of the device state and contamination (and anything
 * abnormal on the first read). */
void MultiscanNode::logStatusChanges(const sopas::SensorStatus& status)
{
    const sopas::SensorStatus& previous = this->last_sensor_status;
    const bool first = !this->has_last_sensor_status;
    constexpr int NORMAL =
        static_cast<int>(sopas::SensorStatus::DeviceState::Normal);

    const bool state_changed =
        first ? status.state != NORMAL : status.state != previous.state;
    if (status.has_state && state_changed)
    {
        if (status.state == NORMAL)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "%sSensor state back to normal operation.",
                LOG_PREFIX);
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "%sSensor state: %s (%d).",
                LOG_PREFIX,
                sopas::deviceStateName(status.state),
                status.state);
        }
    }

    const bool contamination_changed =
        first
            ? (status.contamination_warning || status.contamination_error)
            : (status.contamination_warning != previous.contamination_warning ||
               status.contamination_error != previous.contamination_error);
    if (!status.has_contamination || !contamination_changed)
    {
        return;
    }
    if (status.contamination_error)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "%sOptics cover contamination ERROR: clean the sensor window.",
            LOG_PREFIX);
    }
    else if (status.contamination_warning)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "%sOptics cover contamination warning.",
            LOG_PREFIX);
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%sOptics cover contamination cleared.",
            LOG_PREFIX);
    }
}

/* Logs what went wrong in the stream since the last report. */
void MultiscanNode::reportStreamHealth()
{
    const FrameAssembler::Stats& now = this->assembler->stats();
    FrameAssembler::Stats& before = this->reported_stream_stats;
    const uint64_t lost = now.telegrams_lost - before.telegrams_lost;
    const uint64_t partial = now.frames_partial - before.frames_partial;
    const uint64_t discarded = now.frames_discarded - before.frames_discarded;
    const uint64_t missed = now.frames_missed - before.frames_missed;
    const uint64_t duplicate_or_late =
        (now.segments_duplicate + now.segments_late) -
        (before.segments_duplicate + before.segments_late);
    if (now.expected_mask_changes != before.expected_mask_changes)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "%sSensor sends %d of %u segments per frame (mask 0x%03x).",
            LOG_PREFIX,
            std::popcount(this->assembler->expectedMask()),
            12u,
            this->assembler->expectedMask());
    }
    if (lost || partial || discarded || missed || duplicate_or_late)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "%sStream degraded since last report: %lu telegrams lost, %lu "
            "partial frames published, %lu discarded, %lu frames missed, %lu "
            "duplicate/late segments.",
            LOG_PREFIX,
            static_cast<unsigned long>(lost),
            static_cast<unsigned long>(partial),
            static_cast<unsigned long>(discarded),
            static_cast<unsigned long>(missed),
            static_cast<unsigned long>(duplicate_or_late));
    }
    before = now;
}

void MultiscanNode::updateDiagnosticsSnapshot(bool streaming)
{
    const uint64_t now_ns = realtimeNowNs();
    std::lock_guard lock{this->diagnostics_mutex};
    DiagnosticsSnapshot& state = this->diagnostics;
    state.connected = this->sopas_client.connected();
    state.streaming = streaming;
    if (streaming)
    {
        state.last_error.clear();
    }
    state.stream = this->assembler->stats();
    state.expected_mask = this->assembler->expectedMask();
    state.frames_published = this->frames_published.load();
    state.imu_samples_published = this->imu_samples_published;
    state.clock_valid = this->clock_estimator.valid();
    state.clock_offset_ns = this->clock_estimator.offsetNs(now_ns);
    state.clock_drift = this->clock_estimator.drift();
    state.clock = this->clock_estimator.stats();
}

}  // namespace multiscan
