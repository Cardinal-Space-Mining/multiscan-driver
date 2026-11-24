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

/*  The driver's ROS parameters (see config/params.yaml for what each one
 *  does), read once at startup.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ghost/ghost_filter.hpp"
#include "sopas/sensor_settings.hpp"


namespace multiscan
{

enum class TimeSource
{
    Estimated,  // sensor clock mapped to the host clock (ClockEstimator)
    Sensor,     // sensor clock as is (synchronised via NTP/PTP)
    Receive,    // host receive time of each telegram
};

const char* timeSourceName(TimeSource source);


struct DriverConfig
{
public:
    struct Connection
    {
        std::string lidar_hostname;
        std::string driver_hostname;  // "": the interface that reaches it
        int lidar_udp_port{2115};
        int imu_udp_port{2116};
        int udp_receive_buffer_bytes{4 << 20};
        int sopas_tcp_port{2111};  // CoLa A (firmware 2.3: 2111 = CoLa A only)
        std::string sopas_password{"F4724744"};  // Authorized Client hash
        /* Restart the connection after this long without valid lidar
         * data. */
        double udp_reset_timeout_s{2.};
        double sopas_read_timeout_s{3.};
        double error_restart_timeout_s{3.};
    };

    struct Output
    {
        std::string lidar_frame_id;
        /* 1-based echo to publish. Falls back to the first echo the sensor
         * sends (with its echo filter on first or last it sends only one). */
        int echo_selection{1};
        std::vector<std::string> point_fields;
        bool publish_partial_frames{true};
    };

    struct Imu
    {
        std::string frame_id;  // "": the lidar frame
        bool publish_tf{true};
        std::array<double, 3> orientation_covariance{};
        std::array<double, 3> angular_velocity_covariance{};
        std::array<double, 3> linear_acceleration_covariance{};
    };

    struct GhostFilterSettings
    {
        bool enabled{false};
        /* A path, or the name of a model the driver installs (models/). */
        std::string model_file;
        std::string fallback_model_file;  // used if model_file fails
        GhostFilter::Mode mode{GhostFilter::Mode::Drop};
        double threshold{-1.};  // < 0: the model's
        bool early_exit{true};
    };

public:
    Connection connection;
    Output output;
    Imu imu;
    GhostFilterSettings ghost_filter;
    TimeSource time_source{TimeSource::Estimated};
    double status_period_s{2.};

    /* Applied on every connect (never persisted) unless apply is false. */
    bool apply_sensor_settings{true};
    sopas::SensorSettings sensor;
};


/* Declares every parameter and reads it. Invalid values are logged and
 * replaced by their defaults; invalid sensor settings switch
 * apply_sensor_settings off. */
DriverConfig loadDriverConfig(rclcpp::Node& node);

}  // namespace multiscan
