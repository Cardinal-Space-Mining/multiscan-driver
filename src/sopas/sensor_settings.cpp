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

#include "sensor_settings.hpp"

#include <cmath>
#include <sstream>

#include <arpa/inet.h>

#include "cola_client.hpp"


namespace multiscan::sopas
{

namespace
{

// booleans and small enums: one digit, the same in decimal and hex
std::string flag(bool value) { return value ? "1" : "0"; }
std::string digit(int value) { return std::to_string(value); }

int64_t toTenThousandthDeg(double degrees)
{
    return static_cast<int64_t>(std::llround(degrees * 10000.));
}

/* Collects "sWN <name> <arguments>" / "sRN <name>" items. */
class SettingItemList
{
public:
    void add(
        const std::string& name,
        const std::string& arguments,
        std::vector<int64_t> expected_values)
    {
        this->items.push_back(
            {name,
             "sWN " + name + " " + arguments,
             "sRN " + name,
             std::move(expected_values)});
    }

    /* A method: no read-back, its reply status must be 0. */
    void addMethod(const std::string& name, const std::string& arguments)
    {
        this->items.push_back({name, "sMN " + name + " " + arguments, "", {0}});
    }

public:
    std::vector<SettingItem> items;
};

void addLayers(const SensorSettings& settings, SettingItemList& list)
{
    bool all_layers = true;
    for (const bool enabled : settings.layers)
    {
        all_layers &= enabled;
    }
    std::string arguments = flag(!all_layers);
    std::vector<int64_t> expected = {!all_layers};
    for (const bool enabled : settings.layers)
    {
        arguments += " " + flag(enabled);
        expected.push_back(enabled);
    }
    list.add("LFPlayerFilter", arguments, expected);
}

void addOutputRange(const SensorSettings& settings, SettingItemList& list)
{
    const int64_t start = toTenThousandthDeg(settings.output_range_start_deg);
    const int64_t stop = toTenThousandthDeg(settings.output_range_stop_deg);
    // reserved (1), resolution (1250 = 0.125 deg, fixed), start, stop
    list.add(
        "LMPoutputRange",
        "1 +1250 " + signedDecimal(start) + " " + signedDecimal(stop),
        {1, 1250, start, stop});
}

void addPointFilters(const SensorSettings& settings, SettingItemList& list)
{
    const SensorSettings::RangeFilter& range = settings.range_filter;
    list.add(
        "LFPradialDistanceRangeFilter",
        flag(range.enabled) + " " + signedDecimal(range.min_mm) + " " +
            signedDecimal(range.max_mm),
        {range.enabled, range.min_mm, range.max_mm});

    const SensorSettings::CubicFilter& cubic = settings.cubic_filter;
    std::string cubic_arguments = flag(cubic.enabled);
    std::vector<int64_t> cubic_expected = {cubic.enabled};
    for (int axis = 0; axis < 3; axis++)
    {
        cubic_arguments += " " + signedDecimal(cubic.min_mm[axis]) + " " +
                           signedDecimal(cubic.max_mm[axis]);
        cubic_expected.push_back(cubic.min_mm[axis]);
        cubic_expected.push_back(cubic.max_mm[axis]);
    }
    list.add("LFPcubicareafilter", cubic_arguments, cubic_expected);

    // the threshold "must be taken" as 500 mm (operating instructions)
    list.add(
        "LFPparticle",
        flag(settings.particle_filter) + " +500",
        {settings.particle_filter, 500});
    list.add("MCSenseLevel", flag(settings.fog_filter), {settings.fog_filter});
}

void addScanFilters(const SensorSettings& settings, SettingItemList& list)
{
    const SensorSettings::MovingAverage& average = settings.moving_average;
    list.add(
        "LFPmovingAveragingFilter",
        flag(average.enabled) + " " + signedDecimal(average.depth),
        {average.enabled, average.depth});

    const SensorSettings::IntervalFilter& interval = settings.interval_filter;
    list.add(
        "LFPintervalFilter",
        flag(interval.enabled) + " " + signedDecimal(interval.factor),
        {interval.enabled, interval.factor});
}

void addContamination(const SensorSettings& settings, SettingItemList& list)
{
    const SensorSettings::Contamination& contamination = settings.contamination;
    const int64_t cover = contamination.custom_sectors ? 255 : 0;
    std::string arguments = digit(static_cast<int>(contamination.strategy)) +
                            " " + signedDecimal(contamination.response_time_s) +
                            " " +
                            digit(static_cast<int>(contamination.sensitivity)) +
                            " " + signedDecimal(cover);
    std::vector<int64_t> expected = {
        static_cast<int>(contamination.strategy),
        contamination.response_time_s,
        static_cast<int>(contamination.sensitivity),
        cover};
    for (const bool sector : contamination.sectors)
    {
        arguments += " " + flag(sector);
        expected.push_back(sector);
    }
    arguments +=
        " " + flag(contamination.warning) + " " + flag(contamination.error);
    expected.push_back(contamination.warning);
    expected.push_back(contamination.error);
    list.add("ContaminationConfig", arguments, expected);
}

void addTimeSync(const SensorSettings& settings, SettingItemList& list)
{
    const SensorSettings::TimeSync& sync = settings.time_sync;
    list.add(
        "TSCRole",
        digit(static_cast<int>(sync.mode)),
        {static_cast<int>(sync.mode)});
    if (sync.mode != TimeSyncMode::Ntp)
    {
        return;
    }
    // validate() accepts an empty server; the driver fills it in first
    const auto server =
        parseIpv4(sync.ntp_server).value_or(std::array<uint8_t, 4>{0, 0, 0, 0});
    list.add(
        "TSCTCSrvAddr",
        signedDecimal(server[0]) + " " + signedDecimal(server[1]) + " " +
            signedDecimal(server[2]) + " " + signedDecimal(server[3]),
        {server[0], server[1], server[2], server[3]});
    list.add(
        "TSCTCupdatetime",
        signedDecimal(sync.ntp_update_s),
        {sync.ntp_update_s});
    list.add("TSCTCtimezone", signedDecimal(TIMEZONE_UTC), {TIMEZONE_UTC});
}

}  // namespace



// --- Settings ----------------------------------------------------------------

std::string signedDecimal(int64_t value)
{
    return (value < 0 ? "-" : "+") + std::to_string(value < 0 ? -value : value);
}

std::string SensorSettings::validate() const
{
    std::ostringstream problems;
    if (!(this->output_range_start_deg >= -180. &&
          this->output_range_stop_deg <= 180. &&
          this->output_range_start_deg < this->output_range_stop_deg))
    {
        problems
            << "output range must satisfy -180 <= start < stop <= 180 deg; ";
    }
    bool any_layer = false;
    for (const bool enabled : this->layers)
    {
        any_layer |= enabled;
    }
    if (!any_layer)
    {
        problems << "at least one layer must be enabled; ";
    }
    if (this->range_filter.min_mm < 0 || this->range_filter.max_mm > 200000 ||
        this->range_filter.min_mm > this->range_filter.max_mm)
    {
        problems << "range filter must satisfy 0 <= min <= max <= 200000 mm; ";
    }
    for (int axis = 0; axis < 3; axis++)
    {
        if (this->cubic_filter.min_mm[axis] < -200000 ||
            this->cubic_filter.max_mm[axis] > 200000 ||
            this->cubic_filter.min_mm[axis] > this->cubic_filter.max_mm[axis])
        {
            problems << "cubic filter bounds must satisfy -200000 <= min <= "
                        "max <= 200000 mm; ";
            break;
        }
    }
    if (this->moving_average.depth < 2 || this->moving_average.depth > 10)
    {
        problems << "moving average depth must be 2..10; ";
    }
    if (this->interval_filter.factor < 2 || this->interval_filter.factor > 50)
    {
        problems << "interval filter factor must be 2..50; ";
    }
    if (this->contamination.response_time_s < 3 ||
        this->contamination.response_time_s > 60)
    {
        problems << "contamination response time must be 3..60 s; ";
    }
    if (this->time_sync.mode == TimeSyncMode::Ntp)
    {
        if (this->time_sync.ntp_update_s < 1 ||
            this->time_sync.ntp_update_s > 3600)
        {
            problems << "NTP update interval must be 1..3600 s; ";
        }
        if (!this->time_sync.ntp_server.empty() &&
            !parseIpv4(this->time_sync.ntp_server))
        {
            problems << "NTP server must be an IPv4 address; ";
        }
    }
    std::string text = problems.str();
    if (text.size() >= 2)
    {
        text.resize(text.size() - 2);  // the last "; "
    }
    return text;
}

std::vector<SettingItem> settingItems(const SensorSettings& settings)
{
    SettingItemList list;
    list.add(
        "FREchoFilter",
        digit(static_cast<int>(settings.echo_filter)),
        {static_cast<int>(settings.echo_filter)});
    addLayers(settings, list);
    addOutputRange(settings, list);
    addPointFilters(settings, list);
    addScanFilters(settings, list);
    addContamination(settings, list);
    addTimeSync(settings, list);
    list.addMethod("mCLsetscancfglist", flag(settings.interlaced));
    return list.items;
}

std::string compareReadback(
    const std::vector<std::string>& arguments,
    const std::vector<int64_t>& expected)
{
    std::ostringstream difference;
    if (arguments.size() != expected.size())
    {
        difference << "expected " << expected.size()
                   << " values, sensor returned " << arguments.size();
        return difference.str();
    }
    for (size_t i = 0; i < arguments.size(); i++)
    {
        const auto value = parseNumber(arguments[i]);
        if (value &&
            static_cast<uint32_t>(*value) == static_cast<uint32_t>(expected[i]))
        {
            continue;
        }
        difference << "value " << i << ": expected " << expected[i]
                   << ", sensor has '" << arguments[i] << "'";
        if (value)
        {
            difference << " ("
                       << static_cast<int32_t>(static_cast<uint32_t>(*value))
                       << ")";
        }
        return difference.str();
    }
    return "";
}



// --- Parameter values --------------------------------------------------------

std::optional<EchoFilter> echoFilterFromString(const std::string& text)
{
    if (text == "first")
    {
        return EchoFilter::First;
    }
    if (text == "all")
    {
        return EchoFilter::All;
    }
    if (text == "last")
    {
        return EchoFilter::Last;
    }
    return std::nullopt;
}

std::optional<ContaminationStrategy> contaminationStrategyFromString(
    const std::string& text)
{
    if (text == "inactive")
    {
        return ContaminationStrategy::Inactive;
    }
    if (text == "high_availability")
    {
        return ContaminationStrategy::HighAvailability;
    }
    if (text == "sensitive")
    {
        return ContaminationStrategy::Sensitive;
    }
    return std::nullopt;
}

std::optional<ContaminationSensitivity> contaminationSensitivityFromString(
    const std::string& text)
{
    if (text == "high")
    {
        return ContaminationSensitivity::High;
    }
    if (text == "medium")
    {
        return ContaminationSensitivity::Medium;
    }
    if (text == "low")
    {
        return ContaminationSensitivity::Low;
    }
    return std::nullopt;
}

std::optional<TimeSyncMode> timeSyncModeFromString(const std::string& text)
{
    if (text == "off")
    {
        return TimeSyncMode::Off;
    }
    if (text == "ntp")
    {
        return TimeSyncMode::Ntp;
    }
    if (text == "ptp")
    {
        return TimeSyncMode::Ptp;
    }
    return std::nullopt;
}

std::optional<std::array<uint8_t, 4>> parseIpv4(const std::string& text)
{
    in_addr address{};
    if (::inet_pton(AF_INET, text.c_str(), &address) != 1)
    {
        return std::nullopt;
    }
    const uint32_t ip = ntohl(address.s_addr);
    return std::array<uint8_t, 4>{
        static_cast<uint8_t>(ip >> 24),
        static_cast<uint8_t>(ip >> 16),
        static_cast<uint8_t>(ip >> 8),
        static_cast<uint8_t>(ip)};
}

}  // namespace multiscan::sopas
