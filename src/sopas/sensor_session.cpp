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

#include "sensor_session.hpp"

#include <cmath>
#include <cstring>

#include <arpa/inet.h>


namespace multiscan::sopas
{

size_t StartReport::problems() const
{
    size_t n = 0;
    for (const auto& s : settings)
    {
        n += !s.problem.empty();
    }
    return n;
}

SensorSession::SensorSession(
    ColaClient& client,
    std::string password_hash,
    int timeout) :
    cola{client},
    password{std::move(password_hash)},
    timeout_ms{timeout}
{
}

Result SensorSession::call(const std::string& telegram)
{
    return cola.request(telegram, timeout_ms);
}

bool SensorSession::must(const std::string& telegram, std::string& error)
{
    const Result r = call(telegram);
    if (!r.ok())
    {
        error = "'" + telegram + "': " + r.describe();
        return false;
    }
    return true;
}

bool SensorSession::login(std::string& error)
{
    const Result r = call("sMN SetAccessMode 3 " + password);
    if (!r.ok())
    {
        error = "login: " + r.describe();
        return false;
    }
    if (r.reply.args.empty() || r.reply.args[0] != "1")
    {
        error = "login rejected: wrong sopas_password?";
        return false;
    }
    return true;
}

bool SensorSession::run(std::string& error)
{
    const Result r = call("sMN Run");
    if (!r.ok() || r.reply.args.empty() || r.reply.args[0] != "1")
    {
        error =
            "'sMN Run': " + (r.ok() ? std::string("rejected") : r.describe());
        return false;
    }
    return true;
}

bool SensorSession::start(
    const StreamTarget& target,
    const SensorSettings& settings,
    bool apply_settings,
    StartReport& report)
{
    report = StartReport{};
    std::string& err = report.error;

    // --- identity -------------------------------------------------------------
    auto readString = [this](const char* var) -> std::string
    {
        // "sRA DeviceIdent 9 multiScan 8 2.3.0.0R": length-prefixed strings
        const Result r = call(std::string("sRN ") + var);
        return (r.ok() && r.reply.args.size() >= 2) ? r.reply.args[1]
                                                    : std::string();
    };
    {
        const Result r = call("sRN DeviceIdent");
        if (!r.ok())
        {
            err = "'sRN DeviceIdent': " + r.describe();
            return false;
        }
        if (r.reply.args.size() >= 4)
        {
            report.device.type = r.reply.args[1];
            report.device.firmware = r.reply.args[3];
        }
        report.device.serial = readString("SerialNumber");
    }

    // --- stream ---------------------------------------------------------------
    in_addr addr{};
    if (::inet_pton(AF_INET, target.host.c_str(), &addr) != 1)
    {
        err = "stream destination '" + target.host + "' is not an IPv4 address";
        return false;
    }
    const uint32_t ip = ntohl(addr.s_addr);
    std::string ip_args;
    for (int shift = 24; shift >= 0; shift -= 8)
    {
        ip_args += " " + num((ip >> shift) & 0xFF);
    }
    const std::vector<int64_t> ip_exp = {
        (ip >> 24) & 0xFF,
        (ip >> 16) & 0xFF,
        (ip >> 8) & 0xFF,
        ip & 0xFF};

    std::vector<SettingItem> stream_items = {
        {       "ScanDataFormat","sWN ScanDataFormat 2","sRN ScanDataFormat",{2}                                   },
        {"ScanDataPreformatting",
         "sWN ScanDataPreformatting 1", "sRN ScanDataPreformatting",
         {1}                           },
        {  "ScanDataEthSettings",
         "sWN ScanDataEthSettings 1" + ip_args + " " + num(target.scan_port),
         "sRN ScanDataEthSettings",  {}},
        {   "ImuDataEthSettings",
         "sWN ImuDataEthSettings 1" + ip_args + " " + num(target.imu_port),
         "sRN ImuDataEthSettings",  {} },
    };
    for (int i : {2, 3})
    {
        auto& e = stream_items[i].expected;
        e = {1};
        e.insert(e.end(), ip_exp.begin(), ip_exp.end());
        e.push_back(i == 2 ? target.scan_port : target.imu_port);
    }

    if (!login(err))
    {
        return false;
    }
    for (const auto& it : stream_items)
    {
        if (!must(it.write, err))
        {
            return false;
        }
    }

    // --- settings ---------------------------------------------------------------
    const std::vector<SettingItem> items = settingItems(settings);
    for (const auto& it : items)
    {
        SettingStatus st;
        st.name = it.name;
        if (apply_settings)
        {
            const Result r = call(it.write);
            if (!r.ok())
            {
                st.problem = "not applied: " + r.describe();
                if (r.status == Status::IoError || r.status == Status::Timeout)
                {
                    err = "'" + it.write + "': " + r.describe();
                    return false;
                }
            }
            else if (
                it.read.empty() &&
                (r.reply.args.empty() || r.reply.args[0] != "0"))
            {
                st.problem = "rejected (status " +
                             (r.reply.args.empty() ? std::string("?")
                                                   : r.reply.args[0]) +
                             ")";
            }
            else
            {
                st.written = true;
                st.verified =
                    it.read
                        .empty();  // methods can only be checked by their reply
            }
        }
        // with apply_settings off a method (interlace) stays unverified: it has no read-back
        report.settings.push_back(st);
    }

    if (!run(err))
    {
        return false;
    }
    if (!login(err))
    {
        return false;
    }

    // --- read back --------------------------------------------------------------
    auto verify = [this](const SettingItem& it, SettingStatus& st) -> bool
    {
        const Result r = call(it.read);
        if (r.status == Status::IoError || r.status == Status::Timeout)
        {
            return false;
        }
        if (!r.ok())
        {
            st.problem = "read-back failed: " + r.describe();
            return true;
        }
        const std::string diff = compareReadback(r.reply.args, it.expected);
        st.verified = diff.empty();
        if (!diff.empty() && st.problem.empty())
        {
            st.problem = (st.written ? "read-back differs: "
                                     : "sensor differs from configuration: ") +
                         diff;
        }
        return true;
    };
    for (const auto& it : stream_items)
    {
        SettingStatus st;
        st.name = it.name;
        st.written = true;
        if (!verify(it, st))
        {
            err = "read-back of " + it.name + ": connection lost";
            return false;
        }
        if (!st.verified)
        {
            err = it.name + " " +
                  st.problem;  // without these the stream can't work
            return false;
        }
    }
    for (size_t i = 0; i < items.size(); i++)
    {
        if (items[i].read.empty())
        {
            continue;
        }
        if (!verify(items[i], report.settings[i]))
        {
            err = "read-back of " + items[i].name + ": connection lost";
            return false;
        }
    }

    // --- start --------------------------------------------------------------------
    if (!must("sWN ScanDataEnable 1", err))
    {
        return false;
    }
    if (!must(
            std::string("sWN ImuDataEnable ") +
                (target.imu_enabled ? "1" : "0"),
            err))
    {
        return false;
    }
    if (!run(err))
    {
        return false;
    }
    if (!login(err))
    {
        return false;
    }
    const Result r = call("sMN LMCstartmeas");
    if (!r.ok() || r.reply.args.empty() || r.reply.args[0] != "0")
    {
        err = "'sMN LMCstartmeas': " +
              (r.ok() ? std::string("not allowed") : r.describe());
        return false;
    }
    return run(err);  // log out
}

bool SensorSession::readStatus(SensorStatus& status, std::string& error)
{
    status = SensorStatus{};
    auto read = [&](const char* var, Result& r) -> bool
    {
        r = call(std::string("sRN ") + var);
        if (r.status == Status::IoError || r.status == Status::Timeout ||
            r.status == Status::NotConnected)
        {
            error = std::string("'sRN ") + var + "': " + r.describe();
            return false;
        }
        return true;
    };
    auto usable = [](const Result& r, size_t min_args)
    { return r.ok() && r.reply.args.size() >= min_args; };

    Result r;
    if (!read("DevSta", r))
    {
        return false;
    }
    if (usable(r, 1))
    {
        if (const auto v = parseNumber(r.reply.args[0]))
        {
            status.have_state = true;
            status.state = static_cast<int>(*v);
        }
    }

    if (!read("OPcurtmpdev", r))
    {
        return false;
    }
    if (usable(r, 1))
    {
        // IEEE 754 single precision printed as hex
        if (const auto v = parseNumber(r.reply.args[0]))
        {
            const uint32_t bits = static_cast<uint32_t>(*v);
            float t;
            std::memcpy(&t, &bits, sizeof(t));
            status.have_temperature = std::isfinite(t);
            status.temperature_c = t;
        }
    }

    if (!read("ContaminationResult", r))
    {
        return false;
    }
    if (usable(r, 2))
    {
        status.have_contamination = true;
        status.contamination_warning = r.reply.args[0] != "0";
        status.contamination_error = r.reply.args[1] != "0";
    }

    if (!read("ContaminationData", r))
    {
        return false;
    }
    if (usable(r, 1))
    {
        status.have_sectors = true;
        for (const auto& a : r.reply.args)
        {
            status.sectors.push_back(
                static_cast<int>(parseNumber(a).value_or(-1)));
        }
    }
    return true;
}

const char* deviceStateName(int state)
{
    static constexpr const char* NAMES[] = {
        "reserved",
        "startup",
        "service mode",
        "normal operation",
        "suspended operation",
        "service recommended",
        "service required",
        "recoverable error",
        "fatal error"};
    return (state >= 0 && state <= 8) ? NAMES[state] : "unknown";
}

const char* sectorName(int sector)
{
    static constexpr const char* NAMES[] = {
        "deactivated",
        "clean",
        "warning",
        "error"};
    return (sector >= 0 && sector <= 3) ? NAMES[sector] : "unknown";
}

bool SensorSession::stop(std::string& error)
{
    return login(error) && must("sWN ScanDataEnable 0", error) &&
           must("sWN ImuDataEnable 0", error) && run(error);
}

}  // namespace multiscan::sopas
