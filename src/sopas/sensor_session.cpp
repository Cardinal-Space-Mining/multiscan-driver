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

namespace
{

bool replyStartsWith(const Result& result, const char* value)
{
    return result.ok() && !result.reply.arguments.empty() &&
           result.reply.arguments[0] == value;
}

bool connectionFailed(const Result& result)
{
    return result.status == Status::IoError ||
           result.status == Status::Timeout ||
           result.status == Status::NotConnected;
}

/* The stream settings: Compact format, preformatting, and where scan and IMU
 * data go. Fails if the host isn't an IPv4 address. */
bool streamItems(
    const StreamTarget& target,
    std::vector<SettingItem>& items,
    std::string& error)
{
    in_addr address{};
    if (::inet_pton(AF_INET, target.host.c_str(), &address) != 1)
    {
        error =
            "stream destination '" + target.host + "' is not an IPv4 address";
        return false;
    }
    const uint32_t ip = ntohl(address.s_addr);
    std::string ip_arguments;
    std::vector<int64_t> ip_bytes;
    for (int shift = 24; shift >= 0; shift -= 8)
    {
        ip_arguments += " " + signedDecimal((ip >> shift) & 0xFF);
        ip_bytes.push_back((ip >> shift) & 0xFF);
    }
    const auto destination = [&](uint16_t port)
    {
        std::vector<int64_t> expected = {1};
        expected.insert(expected.end(), ip_bytes.begin(), ip_bytes.end());
        expected.push_back(port);
        return expected;
    };

    items = {
        {       "ScanDataFormat","sWN ScanDataFormat 2","sRN ScanDataFormat",{2}                                   },
        {"ScanDataPreformatting",
         "sWN ScanDataPreformatting 1", "sRN ScanDataPreformatting",
         {1}                                                     },
        {  "ScanDataEthSettings",
         "sWN ScanDataEthSettings 1" + ip_arguments + " " +
         signedDecimal(target.scan_port),
         "sRN ScanDataEthSettings", destination(target.scan_port)},
        {   "ImuDataEthSettings",
         "sWN ImuDataEthSettings 1" + ip_arguments + " " +
         signedDecimal(target.imu_port),
         "sRN ImuDataEthSettings",  destination(target.imu_port) },
    };
    return true;
}

}  // namespace



// --- Reports and names -------------------------------------------------------

size_t StartReport::problems() const
{
    size_t count = 0;
    for (const SettingStatus& setting : this->settings)
    {
        count += !setting.problem.empty();
    }
    return count;
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



// --- SensorSession -----------------------------------------------------------

SensorSession::SensorSession(
    ColaClient& client,
    std::string password_hash,
    int timeout_ms) :
    client{client},
    password_hash{std::move(password_hash)},
    timeout_ms{timeout_ms}
{
}

bool SensorSession::start(
    const StreamTarget& target,
    const SensorSettings& settings,
    bool apply_settings,
    StartReport& report)
{
    report = StartReport{};
    std::string& error = report.error;

    // 1. identity
    if (!this->readIdentity(report.device, error))
    {
        return false;
    }

    // 2.-4. log in, set the stream, apply the settings
    std::vector<SettingItem> stream_items;
    if (!streamItems(target, stream_items, error) || !this->login(error))
    {
        return false;
    }
    for (const SettingItem& item : stream_items)
    {
        if (!this->callOrFail(item.write_telegram, error))
        {
            return false;
        }
    }
    const std::vector<SettingItem> items = settingItems(settings);
    if (!this->applySettings(items, apply_settings, report))
    {
        return false;
    }

    // 5.-6. activate, log in again and read everything back
    if (!this->run(error) || !this->login(error) ||
        !this->verifyStream(stream_items, error) ||
        !this->verifySettings(items, report))
    {
        return false;
    }

    // 7.-8. start the output and log out
    return this->startOutput(target, error);
}

bool SensorSession::stop(std::string& error)
{
    return this->login(error) &&
           this->callOrFail("sWN ScanDataEnable 0", error) &&
           this->callOrFail("sWN ImuDataEnable 0", error) && this->run(error);
}

bool SensorSession::readStatus(SensorStatus& status, std::string& error)
{
    status = SensorStatus{};
    const auto usable = [](const Result& result, size_t min_arguments)
    { return result.ok() && result.reply.arguments.size() >= min_arguments; };
    Result result;

    // --- Device state ---
    if (!this->readVariable("DevSta", result, error))
    {
        return false;
    }
    if (usable(result, 1))
    {
        if (const auto value = parseNumber(result.reply.arguments[0]))
        {
            status.has_state = true;
            status.state = static_cast<int>(*value);
        }
    }

    // --- Temperature: IEEE 754 single precision printed as hex ---
    if (!this->readVariable("OPcurtmpdev", result, error))
    {
        return false;
    }
    if (usable(result, 1))
    {
        if (const auto value = parseNumber(result.reply.arguments[0]))
        {
            const uint32_t bits = static_cast<uint32_t>(*value);
            float temperature;
            std::memcpy(&temperature, &bits, sizeof(temperature));
            status.has_temperature = std::isfinite(temperature);
            status.temperature_c = temperature;
        }
    }

    // --- Contamination ---
    if (!this->readVariable("ContaminationResult", result, error))
    {
        return false;
    }
    if (usable(result, 2))
    {
        status.has_contamination = true;
        status.contamination_warning = result.reply.arguments[0] != "0";
        status.contamination_error = result.reply.arguments[1] != "0";
    }

    if (!this->readVariable("ContaminationData", result, error))
    {
        return false;
    }
    if (usable(result, 1))
    {
        status.has_sectors = true;
        for (const std::string& sector : result.reply.arguments)
        {
            status.sectors.push_back(
                static_cast<int>(parseNumber(sector).value_or(-1)));
        }
    }
    return true;
}



// --- Telegrams ---------------------------------------------------------------

Result SensorSession::call(const std::string& telegram)
{
    return this->client.request(telegram, this->timeout_ms);
}

bool SensorSession::callOrFail(const std::string& telegram, std::string& error)
{
    const Result result = this->call(telegram);
    if (!result.ok())
    {
        error = "'" + telegram + "': " + result.describe();
        return false;
    }
    return true;
}

bool SensorSession::login(std::string& error)
{
    const Result result =
        this->call("sMN SetAccessMode 3 " + this->password_hash);
    if (!result.ok())
    {
        error = "login: " + result.describe();
        return false;
    }
    if (!replyStartsWith(result, "1"))
    {
        error = "login rejected: wrong sopas_password?";
        return false;
    }
    return true;
}

bool SensorSession::run(std::string& error)
{
    const Result result = this->call("sMN Run");
    if (!replyStartsWith(result, "1"))
    {
        error = "'sMN Run': " +
                (result.ok() ? std::string("rejected") : result.describe());
        return false;
    }
    return true;
}

bool SensorSession::readVariable(
    const char* variable,
    Result& result,
    std::string& error)
{
    result = this->call(std::string("sRN ") + variable);
    if (connectionFailed(result))
    {
        error = std::string("'sRN ") + variable + "': " + result.describe();
        return false;
    }
    return true;
}



// --- Steps of start() --------------------------------------------------------

/* Device identity; no login needed. */
bool SensorSession::readIdentity(DeviceInfo& device, std::string& error)
{
    const Result identity = this->call("sRN DeviceIdent");
    if (!identity.ok())
    {
        error = "'sRN DeviceIdent': " + identity.describe();
        return false;
    }
    // "sRA DeviceIdent 9 multiScan 8 2.3.0.0R": length-prefixed strings
    if (identity.reply.arguments.size() >= 4)
    {
        device.type = identity.reply.arguments[1];
        device.firmware = identity.reply.arguments[3];
    }
    const Result serial = this->call("sRN SerialNumber");
    device.serial = (serial.ok() && serial.reply.arguments.size() >= 2)
                        ? serial.reply.arguments[1]
                        : std::string();
    return true;
}

/* Writes every setting (unless apply_settings is off) and records how it
 * went. Fails only if the connection is lost. */
bool SensorSession::applySettings(
    const std::vector<SettingItem>& items,
    bool apply_settings,
    StartReport& report)
{
    for (const SettingItem& item : items)
    {
        SettingStatus status;
        status.name = item.name;
        // With apply_settings off a method (interlace) stays unverified: it
        // has no read-back.
        if (apply_settings)
        {
            const bool is_method = item.read_telegram.empty();
            const Result result = this->call(item.write_telegram);
            if (!result.ok())
            {
                status.problem = "not applied: " + result.describe();
                if (result.status == Status::IoError ||
                    result.status == Status::Timeout)
                {
                    report.error =
                        "'" + item.write_telegram + "': " + result.describe();
                    return false;
                }
            }
            else if (is_method && !replyStartsWith(result, "0"))
            {
                status.problem = "rejected (status " +
                                 (result.reply.arguments.empty()
                                      ? std::string("?")
                                      : result.reply.arguments[0]) +
                                 ")";
            }
            else
            {
                status.written = true;
                // methods can only be checked by their reply
                status.verified = is_method;
            }
        }
        report.settings.push_back(status);
    }
    return true;
}

bool SensorSession::verifySetting(
    const SettingItem& item,
    SettingStatus& status)
{
    const Result result = this->call(item.read_telegram);
    if (result.status == Status::IoError || result.status == Status::Timeout)
    {
        return false;
    }
    if (!result.ok())
    {
        status.problem = "read-back failed: " + result.describe();
        return true;
    }
    const std::string difference =
        compareReadback(result.reply.arguments, item.expected_values);
    status.verified = difference.empty();
    if (!difference.empty() && status.problem.empty())
    {
        status.problem =
            (status.written ? "read-back differs: "
                            : "sensor differs from configuration: ") +
            difference;
    }
    return true;
}

/* Without the stream settings the stream can't work: any difference is
 * fatal. */
bool SensorSession::verifyStream(
    const std::vector<SettingItem>& stream_items,
    std::string& error)
{
    for (const SettingItem& item : stream_items)
    {
        SettingStatus status;
        status.name = item.name;
        status.written = true;
        if (!this->verifySetting(item, status))
        {
            error = "read-back of " + item.name + ": connection lost";
            return false;
        }
        if (!status.verified)
        {
            error = item.name + " " + status.problem;
            return false;
        }
    }
    return true;
}

bool SensorSession::verifySettings(
    const std::vector<SettingItem>& items,
    StartReport& report)
{
    for (size_t i = 0; i < items.size(); i++)
    {
        if (items[i].read_telegram.empty())
        {
            continue;
        }
        if (!this->verifySetting(items[i], report.settings[i]))
        {
            report.error =
                "read-back of " + items[i].name + ": connection lost";
            return false;
        }
    }
    return true;
}

/* Enables scan (and IMU) output, activates it, starts the measurement and
 * logs out: the stream keeps running, and DevSta reads "service mode" while
 * a client is logged in. */
bool SensorSession::startOutput(const StreamTarget& target, std::string& error)
{
    if (!this->callOrFail("sWN ScanDataEnable 1", error) ||
        !this->callOrFail(
            std::string("sWN ImuDataEnable ") +
                (target.imu_enabled ? "1" : "0"),
            error) ||
        !this->run(error) || !this->login(error))
    {
        return false;
    }
    const Result result = this->call("sMN LMCstartmeas");
    if (!replyStartsWith(result, "0"))
    {
        error = "'sMN LMCstartmeas': " +
                (result.ok() ? std::string("not allowed") : result.describe());
        return false;
    }
    return this->run(error);  // log out
}

}  // namespace multiscan::sopas
