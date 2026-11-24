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

/*  Start/stop sequence for the multiScan136 over an open CoLa A connection.
 *
 *  start():
 *    1. read device identity (no login needed)
 *    2. log in as Authorized Client
 *    3. set the stream: Compact format, preformatting, scan + IMU destination
 *    4. apply every SensorSettings item (unless apply_settings is false)
 *    5. sMN Run (activates the changes; logs out)
 *    6. log in, read every setting back and compare with the configuration
 *    7. enable scan (and IMU) output, sMN Run, log in, LMCstartmeas
 *    8. sMN Run again to log out: the stream keeps running, and DevSta reads
 *       "service mode" while a client is logged in
 *
 *  Nothing is saved permanently (no mEEwriteall): a power cycle restores the
 *  sensor's stored configuration, which is why this runs on every connect.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cola_client.hpp"
#include "sensor_settings.hpp"


namespace multiscan::sopas
{

struct StreamTarget
{
    std::string host;  // IPv4 the sensor sends to
    uint16_t scan_port{2115};
    uint16_t imu_port{2116};
    bool imu_enabled{true};
};

struct DeviceInfo
{
    std::string type;      // e.g. "multiScan"
    std::string firmware;  // e.g. "2.3.0.0R"
    std::string serial;
};

struct SettingStatus
{
    std::string name;
    /* The write was accepted (always false with apply_settings off). */
    bool written{false};
    /* The read-back equals the configuration (methods: their reply was
     * ok). */
    bool verified{false};
    std::string problem;  // why it isn't written/verified
};

/* Polled while streaming (no login needed). */
struct SensorStatus
{
public:
    enum class DeviceState : int
    {
        Reserved = 0,
        Startup = 1,
        ServiceMode = 2,  // also while a client is logged in
        Normal = 3,
        Suspended = 4,
        ServiceRecommended = 5,
        ServiceRequired = 6,
        RecoverableError = 7,
        FatalError = 8,
    };

    enum class Sector : int
    {
        Deactivated = 0,
        Clean = 1,
        Warning = 2,
        Error = 3,
    };

public:
    bool has_state{false};
    int state{0};  // DeviceState, or the raw value if outside it
    bool has_temperature{false};
    float temperature_c{0.f};  // internal, not ambient
    bool has_contamination{false};
    bool contamination_warning{false};
    bool contamination_error{false};
    bool has_sectors{false};
    std::vector<int> sectors;  // Sector per 30 deg sector
};

const char* deviceStateName(int state);
const char* sectorName(int sector);

struct StartReport
{
    DeviceInfo device;
    std::vector<SettingStatus> settings;
    std::string error;  // fatal: streaming was not started

    bool ok() const { return this->error.empty(); }
    size_t problems() const;
};


class SensorSession
{
public:
    SensorSession(
        ColaClient& client,
        std::string password_hash,
        int timeout_ms);

public:
    bool start(
        const StreamTarget& target,
        const SensorSettings& settings,
        bool apply_settings,
        StartReport& report);
    /* Disable scan and IMU output. */
    bool stop(std::string& error);
    /* Read device state, temperature and contamination. Variables the
     * sensor doesn't know are left unset; false only if the connection
     * failed (error says why). */
    bool readStatus(SensorStatus& status, std::string& error);

protected:
    // --- Telegrams -----------------------------------------------------------
    Result call(const std::string& telegram);
    bool callOrFail(const std::string& telegram, std::string& error);
    bool login(std::string& error);
    bool run(std::string& error);
    /* Reads a variable for readStatus: false only if the connection
     * failed. */
    bool readVariable(const char* variable, Result& result, std::string& error);

    // --- Steps of start() ----------------------------------------------------
    bool readIdentity(DeviceInfo& device, std::string& error);
    bool applySettings(
        const std::vector<SettingItem>& items,
        bool apply_settings,
        StartReport& report);
    /* Compares one item's read-back; false only if the connection failed. */
    bool verifySetting(const SettingItem& item, SettingStatus& status);
    bool verifyStream(
        const std::vector<SettingItem>& stream_items,
        std::string& error);
    bool verifySettings(
        const std::vector<SettingItem>& items,
        StartReport& report);
    bool startOutput(const StreamTarget& target, std::string& error);

protected:
    ColaClient& client;
    std::string password_hash;
    int timeout_ms;
};

}  // namespace multiscan::sopas
