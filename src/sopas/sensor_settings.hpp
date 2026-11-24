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

/*  multiScan136 settings the driver applies on every connect, and their
 *  CoLa A telegrams. Units, ranges and read-back formats were checked on
 *  firmware 2.3.0 (test/data/sopas_transcript_fw2.3.txt); where the operating
 *  instructions disagree with the device, the device wins:
 *    - LFPcubicareafilter bounds are mm (the manual says 1/10 mm)
 *    - temperatureAlarmConfiguration doesn't exist on firmware 2.3
 *    - interlaced mode (a method) has no read-back
 *    - LFPintervalFilter rejects a factor of 1 (sFA 30); the manual allows 1..50
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>


namespace multiscan::sopas
{

enum class EchoFilter : int
{
    First = 0,
    All = 1,
    Last = 2,
};

enum class ContaminationStrategy : int
{
    Inactive = 0,
    HighAvailability = 1,  // warn/error when all sectors are contaminated
    Sensitive = 2,         // warn/error when any sector is contaminated
};

enum class ContaminationSensitivity : int
{
    High = 0,
    Medium = 1,
    Low = 2,
};

/* What the sensor synchronises its clock to (TSCRole). Without it the clock
 * counts from power-up. */
enum class TimeSyncMode : int
{
    Off = 0,
    Ntp = 1,  // polls ntp_server
    Ptp = 3,  // IEEE 1588 slave; needs a PTP master on the network
};

/* TSCTCtimezone value for UTC (COORD_WORLD_TIME), written with NTP. */
constexpr int64_t TIMEZONE_UTC = 34;

struct SensorSettings
{
public:
    struct RangeFilter
    {
        bool enabled{false};
        int64_t min_mm{0};
        int64_t max_mm{200000};
    };

    struct CubicFilter
    {
        bool enabled{false};
        std::array<int64_t, 3> min_mm{-20000, -20000, -20000};  // x, y, z
        std::array<int64_t, 3> max_mm{20000, 20000, 20000};
    };

    struct MovingAverage
    {
        bool enabled{false};
        int64_t depth{2};  // scans averaged
    };

    struct IntervalFilter
    {
        bool enabled{false};
        /* Output every n-th scan; the device rejects 1 (sFA 30). */
        int64_t factor{4};
    };

    struct Contamination
    {
        ContaminationStrategy strategy{ContaminationStrategy::Inactive};
        int64_t response_time_s{3};
        ContaminationSensitivity sensitivity{ContaminationSensitivity::Low};
        /* Use `sectors` (else: no weather protection hood). */
        bool custom_sectors{true};
        std::array<bool, 12> sectors{
            true,
            true,
            true,
            true,
            true,
            true,
            true,
            true,
            true,
            true,
            true,
            true};
        bool warning{true};
        bool error{true};
    };

    struct TimeSync
    {
        TimeSyncMode mode{TimeSyncMode::Off};
        /* IPv4 of the NTP server. Must be set before settingItems() when
         * mode is Ntp (the driver fills in its own address if empty). */
        std::string ntp_server;
        int64_t ntp_update_s{10};  // 1..3600
    };

public:
    /* Empty if valid, else what is wrong. */
    std::string validate() const;

public:
    EchoFilter echo_filter{EchoFilter::Last};
    /* Layers 1..16 (index 0..15); layers 6 and 14 are the high-res ones. */
    std::array<bool, 16> layers{
        true,
        true,
        true,
        true,
        true,
        false,
        true,
        true,
        true,
        true,
        true,
        true,
        true,
        false,
        true,
        true};
    double output_range_start_deg{-180.};
    double output_range_stop_deg{180.};
    bool interlaced{false};

    RangeFilter range_filter;
    CubicFilter cubic_filter;
    bool particle_filter{false};  // adds one scan of latency
    bool fog_filter{false};
    MovingAverage moving_average;
    IntervalFilter interval_filter;
    Contamination contamination;
    TimeSync time_sync;
};


/* One setting: the telegram that applies it and how to check it. */
struct SettingItem
{
    std::string name;
    std::string write_telegram;  // full telegram, e.g. "sWN FREchoFilter 2"
    std::string read_telegram;   // "sRN FREchoFilter", or "" for methods
    /* Read-back arguments (or the reply for methods). */
    std::vector<int64_t> expected_values;
};

std::vector<SettingItem> settingItems(const SensorSettings& settings);

/* "" if the read-back arguments equal `expected` (compared as 32-bit two's
 * complement, which is how the sensor prints negative values), else a
 * description of the difference. */
std::string compareReadback(
    const std::vector<std::string>& arguments,
    const std::vector<int64_t>& expected);

/* CoLa A argument format for numbers: signed decimal ("+5", "-5"), which the
 * sensor never confuses with hex. */
std::string signedDecimal(int64_t value);


// --- Parameter values --------------------------------------------------------

std::optional<EchoFilter> echoFilterFromString(const std::string& text);
std::optional<ContaminationStrategy> contaminationStrategyFromString(
    const std::string& text);
std::optional<ContaminationSensitivity> contaminationSensitivityFromString(
    const std::string& text);
std::optional<TimeSyncMode> timeSyncModeFromString(const std::string& text);

/* Dotted IPv4 -> its four bytes, or nullopt. */
std::optional<std::array<uint8_t, 4>> parseIpv4(const std::string& text);

}  // namespace multiscan::sopas
