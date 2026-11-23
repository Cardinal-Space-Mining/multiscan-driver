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

#include "cola_client.hpp"


namespace multiscan::sopas
{

namespace
{
// booleans and small enums: one digit, the same in decimal and hex
std::string flag(bool b) { return b ? "1" : "0"; }
std::string digit(int v) { return std::to_string(v); }

int64_t toTenThousandthDeg(double deg)
{
    return static_cast<int64_t>(std::llround(deg * 10000.));
}
}  // namespace

std::string num(int64_t v)
{
    return (v < 0 ? "-" : "+") + std::to_string(v < 0 ? -v : v);
}

std::string SensorSettings::validate() const
{
    std::ostringstream e;
    if (!(output_range_start_deg >= -180. && output_range_stop_deg <= 180. &&
          output_range_start_deg < output_range_stop_deg))
    {
        e << "output range must satisfy -180 <= start < stop <= 180 deg; ";
    }
    bool any_layer = false;
    for (bool l : layers)
    {
        any_layer |= l;
    }
    if (!any_layer)
    {
        e << "at least one layer must be enabled; ";
    }
    if (range_filter.min_mm < 0 || range_filter.max_mm > 200000 ||
        range_filter.min_mm > range_filter.max_mm)
    {
        e << "range filter must satisfy 0 <= min <= max <= 200000 mm; ";
    }
    for (int i = 0; i < 3; i++)
    {
        if (cubic_filter.min_mm[i] < -200000 ||
            cubic_filter.max_mm[i] > 200000 ||
            cubic_filter.min_mm[i] > cubic_filter.max_mm[i])
        {
            e << "cubic filter bounds must satisfy -200000 <= min <= max <= 200000 mm; ";
            break;
        }
    }
    if (moving_average.depth < 2 || moving_average.depth > 10)
    {
        e << "moving average depth must be 2..10; ";
    }
    if (interval_filter.factor < 2 || interval_filter.factor > 50)
    {
        e << "interval filter factor must be 2..50; ";
    }
    if (contamination.response_time_s < 3 || contamination.response_time_s > 60)
    {
        e << "contamination response time must be 3..60 s; ";
    }
    std::string s = e.str();
    if (s.size() >= 2)
    {
        s.resize(s.size() - 2);
    }
    return s;
}

std::vector<SettingItem> settingItems(const SensorSettings& s)
{
    std::vector<SettingItem> items;
    auto add = [&items](
                   std::string name,
                   std::string args,
                   std::vector<int64_t> expected)
    {
        items.push_back(
            {name,
             "sWN " + name + " " + args,
             "sRN " + name,
             std::move(expected)});
    };

    add("FREchoFilter",
        digit(static_cast<int>(s.echo_filter)),
        {static_cast<int>(s.echo_filter)});

    {
        bool all = true;
        for (bool l : s.layers)
        {
            all &= l;
        }
        std::string args = flag(!all);
        std::vector<int64_t> exp = {!all};
        for (bool l : s.layers)
        {
            args += " " + flag(l);
            exp.push_back(l);
        }
        add("LFPlayerFilter", args, exp);
    }

    {
        const int64_t a = toTenThousandthDeg(s.output_range_start_deg);
        const int64_t b = toTenThousandthDeg(s.output_range_stop_deg);
        // reserved (1), resolution (1250 = 0.125 deg, fixed), start, stop
        add("LMPoutputRange",
            "1 +1250 " + num(a) + " " + num(b),
            {1, 1250, a, b});
    }

    add("LFPradialDistanceRangeFilter",
        flag(s.range_filter.enabled) + " " + num(s.range_filter.min_mm) + " " +
            num(s.range_filter.max_mm),
        {s.range_filter.enabled, s.range_filter.min_mm, s.range_filter.max_mm});

    {
        const auto& c = s.cubic_filter;
        std::string args = flag(c.enabled);
        std::vector<int64_t> exp = {c.enabled};
        for (int i = 0; i < 3; i++)
        {
            args += " " + num(c.min_mm[i]) + " " + num(c.max_mm[i]);
            exp.push_back(c.min_mm[i]);
            exp.push_back(c.max_mm[i]);
        }
        add("LFPcubicareafilter", args, exp);
    }

    // the threshold "must be taken" as 500 mm (operating instructions)
    add("LFPparticle",
        flag(s.particle_filter) + " +500",
        {s.particle_filter, 500});
    add("MCSenseLevel", flag(s.fog_filter), {s.fog_filter});
    add("LFPmovingAveragingFilter",
        flag(s.moving_average.enabled) + " " + num(s.moving_average.depth),
        {s.moving_average.enabled, s.moving_average.depth});
    add("LFPintervalFilter",
        flag(s.interval_filter.enabled) + " " + num(s.interval_filter.factor),
        {s.interval_filter.enabled, s.interval_filter.factor});

    {
        const auto& c = s.contamination;
        const int64_t cover = c.custom_sectors ? 255 : 0;
        std::string args =
            digit(static_cast<int>(c.strategy)) + " " + num(c.response_time_s) +
            " " + digit(static_cast<int>(c.sensitivity)) + " " + num(cover);
        std::vector<int64_t> exp = {
            static_cast<int>(c.strategy),
            c.response_time_s,
            static_cast<int>(c.sensitivity),
            cover};
        for (bool b : c.sectors)
        {
            args += " " + flag(b);
            exp.push_back(b);
        }
        args += " " + flag(c.warning) + " " + flag(c.error);
        exp.push_back(c.warning);
        exp.push_back(c.error);
        add("ContaminationConfig", args, exp);
    }

    // a method without read-back; its reply status must be 0
    items.push_back(
        {"mCLsetscancfglist",
         "sMN mCLsetscancfglist " + flag(s.interlaced),
         "",
         {0}});
    return items;
}

std::string compareReadback(
    const std::vector<std::string>& args,
    const std::vector<int64_t>& expected)
{
    std::ostringstream e;
    if (args.size() != expected.size())
    {
        e << "expected " << expected.size() << " values, sensor returned "
          << args.size();
        return e.str();
    }
    for (size_t i = 0; i < args.size(); i++)
    {
        const auto v = parseNumber(args[i]);
        if (!v ||
            static_cast<uint32_t>(*v) != static_cast<uint32_t>(expected[i]))
        {
            e << "value " << i << ": expected " << expected[i]
              << ", sensor has '" << args[i] << "'";
            if (v)
            {
                e << " (" << static_cast<int32_t>(static_cast<uint32_t>(*v))
                  << ")";
            }
            return e.str();
        }
    }
    return "";
}

std::optional<EchoFilter> echoFilterFromString(const std::string& s)
{
    if (s == "first")
    {
        return EchoFilter::First;
    }
    if (s == "all")
    {
        return EchoFilter::All;
    }
    if (s == "last")
    {
        return EchoFilter::Last;
    }
    return std::nullopt;
}

std::optional<ContaminationStrategy> contaminationStrategyFromString(
    const std::string& s)
{
    if (s == "inactive")
    {
        return ContaminationStrategy::Inactive;
    }
    if (s == "high_availability")
    {
        return ContaminationStrategy::HighAvailability;
    }
    if (s == "sensitive")
    {
        return ContaminationStrategy::Sensitive;
    }
    return std::nullopt;
}

std::optional<ContaminationSensitivity> contaminationSensitivityFromString(
    const std::string& s)
{
    if (s == "high")
    {
        return ContaminationSensitivity::High;
    }
    if (s == "medium")
    {
        return ContaminationSensitivity::Medium;
    }
    if (s == "low")
    {
        return ContaminationSensitivity::Low;
    }
    return std::nullopt;
}

}  // namespace multiscan::sopas
