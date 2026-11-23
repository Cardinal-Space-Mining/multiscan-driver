/*  CoLa A client, sensor settings and the start/stop sequence
 *  (src/sopas/), against real device replies (the firmware 2.3 transcript)
 *  and the simulated sensor in support/fake_sensor.hpp.
 */
#include <gtest/gtest.h>

#include <fstream>
#include <map>

#include "sopas/cola_client.hpp"
#include "sopas/sensor_session.hpp"
#include "sopas/sensor_settings.hpp"
#include "support/fake_sensor.hpp"
#include "support/mscap.hpp"

using namespace multiscan::sopas;
using ms_test::FakeSensor;

namespace
{

const std::string TRANSCRIPT = ms_test::fixture("sopas_transcript_fw2.3.txt");

/* Replies the device gave to "sRN <var>", in order. */
std::map<std::string, std::vector<std::vector<std::string>>> readbacks()
{
    std::map<std::string, std::vector<std::vector<std::string>>> out;
    std::ifstream f(TRANSCRIPT);
    std::string line;
    while (std::getline(f, line))
    {
        if (line.rfind("< sRA ", 0) == 0)
        {
            const Telegram t = parseTelegram(line.substr(2));
            out[t.name].push_back(t.args);
        }
    }
    return out;
}

/* The test values the transcript wrote (second read-back of each variable). */
SensorSettings transcriptTestValues()
{
    SensorSettings s;
    s.echo_filter = EchoFilter::First;
    s.layers = {
        true,
        true,
        false,
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
        true,
        false};
    s.output_range_start_deg = -90.;
    s.output_range_stop_deg = 45.;
    s.range_filter = {true, 500, 30000};
    s.cubic_filter = {
        true,
        {-50000, -70000, -10000},
        { 60000,  80000,  20000}
    };
    s.particle_filter = true;
    s.fog_filter = true;
    s.moving_average = {true, 3};
    s.interval_filter = {true, 2};
    s.contamination.strategy = ContaminationStrategy::HighAvailability;
    s.contamination.response_time_s = 10;
    s.contamination.sensitivity = ContaminationSensitivity::Medium;
    s.contamination.custom_sectors = false;
    s.contamination.sectors = {
        true,
        true,
        true,
        true,
        false,
        false,
        true,
        true,
        true,
        true,
        true,
        true};
    s.contamination.warning = true;
    s.contamination.error = false;
    s.time_sync.mode = TimeSyncMode::Ntp;
    s.time_sync.ntp_server = "10.11.10.1";
    s.time_sync.ntp_update_s = 10;
    return s;
}

struct Connected
{
    explicit Connected(FakeSensor& fake)
    {
        std::string err;
        EXPECT_TRUE(client.connect("127.0.0.1", fake.port(), 1000, err)) << err;
    }
    ColaClient client;
};

}  // namespace


// --- Formats -------------------------------------------------------------------------

TEST(Sopas, ParsesNumbersLikeTheDevicePrintsThem)
{
    EXPECT_EQ(parseNumber("4E2"), 1250);
    EXPECT_EQ(parseNumber("+1250"), 1250);
    EXPECT_EQ(parseNumber("-900000"), -900000);
    EXPECT_EQ(
        static_cast<int32_t>(static_cast<uint32_t>(*parseNumber("FFF24460"))),
        -900000);
    EXPECT_FALSE(parseNumber(""));
    EXPECT_FALSE(parseNumber("12G"));
    EXPECT_EQ(num(5), "+5");
    EXPECT_EQ(num(-5), "-5");
    EXPECT_EQ(num(0), "+0");
}

TEST(Sopas, SplitsTelegrams)
{
    const Telegram t = parseTelegram("sRA DeviceIdent 9 multiScan 8 2.3.0.0R");
    EXPECT_EQ(t.type, "sRA");
    EXPECT_EQ(t.name, "DeviceIdent");
    EXPECT_EQ(
        t.args,
        (std::vector<std::string>{"9", "multiScan", "8", "2.3.0.0R"}));
    const Telegram e = parseTelegram("sFA A");
    EXPECT_EQ(e.type, "sFA");
    EXPECT_EQ(e.args, (std::vector<std::string>{"A"}));
}

TEST(SensorSettings, ExpectedReadbacksMatchTheRealDevice)
{
    const auto rb = readbacks();
    for (const auto& it : settingItems(transcriptTestValues()))
    {
        if (it.read.empty())
        {
            continue;  // interlace: no read-back
        }
        // the device's read-back after the test write is one of the later
        // replies (the first is the original value)
        const auto& replies = rb.at(it.name);
        ASSERT_GE(replies.size(), 2u) << it.name;
        bool matched = false;
        std::string diff;
        for (size_t i = 1; i < replies.size() && !matched; i++)
        {
            diff = compareReadback(replies[i], it.expected);
            matched = diff.empty();
        }
        EXPECT_TRUE(matched) << it.name << ": " << it.write << ": " << diff;
    }
}

TEST(SensorSettings, DefaultsMatchTheTeamsCurrentSensorConfiguration)
{
    // switching to the new driver must not change these
    const auto rb = readbacks();
    for (const auto& it : settingItems(SensorSettings{}))
    {
        if (it.name == "LFPradialDistanceRangeFilter" ||
            it.name == "LFPintervalFilter")
        {
            continue;  // disabled filters with different (unused) stored bounds
        }
        if (it.read.empty())
        {
            continue;
        }
        EXPECT_EQ(compareReadback(rb.at(it.name).at(0), it.expected), "")
            << it.name;
    }
}

TEST(SensorSettings, FakeFormatsWritesLikeTheDevice)
{
    // the simulator must turn every recorded write into the recorded read-back
    std::ifstream f(TRANSCRIPT);
    std::string line, pending_var, pending_args;
    std::map<std::string, std::string> accepted;  // var -> last accepted args
    size_t checked = 0;
    while (std::getline(f, line))
    {
        if (line.rfind("> sWN ", 0) == 0)
        {
            const Telegram t = parseTelegram(line.substr(2));
            pending_var = t.name;
            pending_args = line.substr(2 + 4 + t.name.size() + 1);
        }
        else if (line.rfind("< sWA ", 0) == 0 && !pending_var.empty())
        {
            accepted[pending_var] =
                pending_args;  // rejected writes (sFA) change nothing
            pending_var.clear();
        }
        else if (line.rfind("< sRA ", 0) == 0)
        {
            const Telegram t = parseTelegram(line.substr(2));
            const auto it = accepted.find(t.name);
            if (it != accepted.end())
            {
                EXPECT_EQ(
                    FakeSensor::toDeviceFormat(it->second),
                    line.substr(6 + t.name.size() + 1))
                    << t.name;
                accepted.erase(it);
                checked++;
            }
        }
    }
    EXPECT_GE(checked, 18u);
}

TEST(SensorSettings, Validation)
{
    EXPECT_EQ(SensorSettings{}.validate(), "");
    SensorSettings s;
    s.output_range_start_deg = 10;
    s.output_range_stop_deg = 5;
    EXPECT_NE(s.validate().find("output range"), std::string::npos);
    s = {};
    s.layers.fill(false);
    EXPECT_NE(s.validate().find("layer"), std::string::npos);
    s = {};
    s.contamination.response_time_s = 61;
    EXPECT_NE(s.validate().find("response time"), std::string::npos);
    s = {};
    s.cubic_filter.min_mm[1] = 5;
    s.cubic_filter.max_mm[1] = 4;
    EXPECT_NE(s.validate().find("cubic"), std::string::npos);
    s = {};
    s.interval_filter.factor = 1;  // rejected by firmware 2.3
    EXPECT_NE(s.validate().find("interval"), std::string::npos);
}


// --- Client ----------------------------------------------------------------------------

TEST(ColaClient, RequestReplyAndErrors)
{
    FakeSensor fake(TRANSCRIPT);
    Connected c(fake);
    EXPECT_EQ(c.client.localAddress(), "127.0.0.1");

    Result r = c.client.request("sRN FREchoFilter", 1000);
    ASSERT_TRUE(r.ok()) << r.describe();
    EXPECT_EQ(r.reply.args, (std::vector<std::string>{"2"}));

    r = c.client.request("sWN FREchoFilter 0", 1000);  // not logged in
    EXPECT_EQ(r.status, Status::SopasError);
    EXPECT_EQ(r.sopas_error, 10);
    EXPECT_NE(r.describe().find("WRITE_ACCESSDENIED"), std::string::npos);
    EXPECT_EQ(r.describe().rfind("sFA A:", 0), 0u)
        << "codes print in hex, like the device";

    r = c.client.request("sRN NoSuchVariable", 1000);
    EXPECT_EQ(r.sopas_error, 3);
}

TEST(ColaClient, DeliversEventsAndSurvivesFragmentation)
{
    FakeSensor fake(TRANSCRIPT);
    fake.events_before_reply = {
        "sSN ContaminationResult 0 0",
        "sSN ContaminationData 1 1 1 1 1 1 1 1 1 1 1 2"};
    fake.byte_by_byte = true;
    Connected c(fake);
    std::vector<std::string> events;
    c.client.onEvent([&](const Telegram& t) { events.push_back(t.name); });
    const Result r = c.client.request("sRN DevSta", 2000);
    ASSERT_TRUE(r.ok()) << r.describe();
    EXPECT_EQ(r.reply.args, (std::vector<std::string>{"3"}));
    EXPECT_EQ(
        events,
        (std::vector<std::string>{"ContaminationResult", "ContaminationData"}));
}

TEST(ColaClient, SkipsStaleReplies)
{
    FakeSensor fake(TRANSCRIPT);
    fake.stale_reply_first = true;
    Connected c(fake);
    const Result r = c.client.request("sRN TSCRole", 1000);
    ASSERT_TRUE(r.ok()) << r.describe();
    EXPECT_EQ(r.reply.name, "TSCRole");
}

TEST(ColaClient, TimeoutAndLostConnection)
{
    FakeSensor fake(TRANSCRIPT);
    fake.no_reply = {"sRN DevSta"};
    fake.close_after = 2;
    Connected c(fake);
    EXPECT_EQ(c.client.request("sRN DevSta", 100).status, Status::Timeout);
    EXPECT_TRUE(c.client.connected());
    EXPECT_TRUE(c.client.request("sRN TSCRole", 1000).ok());
    EXPECT_EQ(c.client.request("sRN TSCRole", 1000).status, Status::IoError);
    EXPECT_FALSE(c.client.connected());
    EXPECT_EQ(
        c.client.request("sRN TSCRole", 1000).status,
        Status::NotConnected);
}

TEST(ColaClient, ConnectFailureIsReported)
{
    uint16_t closed_port;
    {
        FakeSensor tmp(TRANSCRIPT);
        closed_port = tmp.port();
    }
    ColaClient c;
    std::string err;
    EXPECT_FALSE(c.connect("127.0.0.1", closed_port, 500, err));
    EXPECT_NE(err.find("refused"), std::string::npos) << err;
    EXPECT_FALSE(c.connect("no-such-host.invalid", 2111, 500, err));
}


// --- Start / stop sequence ------------------------------------------------------------------

TEST(SensorSession, StartAppliesVerifiesAndStartsStreaming)
{
    FakeSensor fake(TRANSCRIPT);
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    StartReport rep;
    const SensorSettings cfg = transcriptTestValues();
    ASSERT_TRUE(session.start({"10.11.10.1", 2115, 2116, true}, cfg, true, rep))
        << rep.error;
    EXPECT_EQ(rep.device.type, "multiScan");
    EXPECT_EQ(rep.device.firmware, "2.3.0.0R");
    EXPECT_EQ(rep.device.serial, "23170008");
    EXPECT_EQ(rep.problems(), 0u);
    for (const auto& s : rep.settings)
    {
        EXPECT_TRUE(s.written && s.verified) << s.name << ": " << s.problem;
    }
    // the sensor now holds what the device itself reported after the same writes
    EXPECT_EQ(fake.value("LMPoutputRange"), "1 4E2 FFF24460 6DDD0");
    EXPECT_EQ(fake.value("ScanDataEthSettings"), "1 A B A 1 843");
    EXPECT_EQ(fake.value("ImuDataEthSettings"), "1 A B A 1 844");
    EXPECT_EQ(fake.value("ScanDataEnable"), "1");
    EXPECT_EQ(fake.value("ImuDataEnable"), "1");
    EXPECT_FALSE(fake.interlaced);

    // order: every write while logged in; Run before read-back; startmeas last
    const auto log = fake.requests();
    size_t first_run = SIZE_MAX, first_readback_of_settings = SIZE_MAX;
    for (size_t i = 0; i < log.size(); i++)
    {
        if (log[i].telegram.rfind("sWN", 0) == 0 ||
            log[i].telegram.rfind("sMN mCL", 0) == 0 ||
            log[i].telegram == "sMN LMCstartmeas")
        {
            EXPECT_TRUE(log[i].logged_in) << log[i].telegram;
        }
        if (log[i].telegram == "sMN Run" && first_run == SIZE_MAX)
        {
            first_run = i;
        }
        if (log[i].telegram == "sRN FREchoFilter" &&
            first_readback_of_settings == SIZE_MAX)
        {
            first_readback_of_settings = i;
        }
        EXPECT_EQ(log[i].telegram.find("mEEwriteall"), std::string::npos)
            << "must never persist";
    }
    EXPECT_LT(first_run, first_readback_of_settings);
    ASSERT_GE(log.size(), 2u);
    EXPECT_EQ(log[log.size() - 2].telegram, "sMN LMCstartmeas");
    EXPECT_EQ(log.back().telegram, "sMN Run");  // logs out

    // logged out, so DevSta shows the real state rather than service mode
    SensorStatus st;
    std::string err;
    ASSERT_TRUE(session.readStatus(st, err)) << err;
    EXPECT_TRUE(st.have_state);
    EXPECT_EQ(st.state, 3);
}

TEST(SensorSession, InterlaceIsAppliedAsAMethod)
{
    FakeSensor fake(TRANSCRIPT);
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    SensorSettings cfg;
    cfg.interlaced = true;
    StartReport rep;
    ASSERT_TRUE(session.start({"10.11.10.1", 2115, 2116, true}, cfg, true, rep))
        << rep.error;
    EXPECT_TRUE(fake.interlaced);
}

TEST(SensorSession, ReportOnlyModeNeverWritesSettings)
{
    FakeSensor fake(TRANSCRIPT);
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    SensorSettings cfg;
    cfg.echo_filter = EchoFilter::First;  // the sensor has "last"
    StartReport rep;
    ASSERT_TRUE(
        session.start({"10.11.10.1", 2115, 2116, true}, cfg, false, rep))
        << rep.error;
    for (const auto& r : fake.requests())
    {
        const bool stream = r.telegram.find("ScanData") != std::string::npos ||
                            r.telegram.find("ImuData") != std::string::npos;
        EXPECT_FALSE(r.telegram.rfind("sWN", 0) == 0 && !stream) << r.telegram;
    }
    EXPECT_EQ(fake.value("FREchoFilter"), "2");
    bool flagged = false;
    for (const auto& s : rep.settings)
    {
        if (s.name == "FREchoFilter")
        {
            flagged = !s.verified &&
                      s.problem.find("sensor differs") != std::string::npos;
        }
    }
    EXPECT_TRUE(flagged);
}

TEST(SensorSession, RejectedAndIgnoredWritesAreReportedNotFatal)
{
    FakeSensor fake(TRANSCRIPT);
    fake.fail_writes["LFPparticle"] = 4;
    fake.ignore_writes = {"MCSenseLevel"};
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    SensorSettings cfg;
    cfg.particle_filter = true;
    cfg.fog_filter = true;
    StartReport rep;
    ASSERT_TRUE(session.start({"10.11.10.1", 2115, 2116, true}, cfg, true, rep))
        << rep.error;
    EXPECT_EQ(rep.problems(), 2u);
    for (const auto& s : rep.settings)
    {
        if (s.name == "LFPparticle")
        {
            EXPECT_NE(s.problem.find("LOCALCONDITIONFAILED"), std::string::npos)
                << s.problem;
        }
        if (s.name == "MCSenseLevel")
        {
            EXPECT_NE(s.problem.find("read-back differs"), std::string::npos)
                << s.problem;
        }
    }
    EXPECT_EQ(fake.value("ScanDataEnable"), "1") << "streaming still starts";
}

TEST(SensorSession, FatalProblemsStopTheStart)
{
    {
        FakeSensor fake(TRANSCRIPT);
        fake.password = "12345678";
        Connected c(fake);
        SensorSession session(c.client, "F4724744", 1000);
        StartReport rep;
        EXPECT_FALSE(
            session.start({"10.11.10.1", 2115, 2116, true}, {}, true, rep));
        EXPECT_NE(rep.error.find("wrong sopas_password"), std::string::npos)
            << rep.error;
    }
    {
        FakeSensor fake(TRANSCRIPT);
        fake.ignore_writes = {
            "ScanDataEthSettings"};  // stream would go elsewhere
        Connected c(fake);
        SensorSession session(c.client, "F4724744", 1000);
        StartReport rep;
        // (the recorded destination is 10.11.10.1:2115, so ask for another one)
        EXPECT_FALSE(
            session.start({"10.11.10.7", 2115, 2116, true}, {}, true, rep));
        EXPECT_NE(rep.error.find("ScanDataEthSettings"), std::string::npos)
            << rep.error;
        EXPECT_EQ(fake.value("ScanDataEnable"), "0");
    }
    {
        FakeSensor fake(TRANSCRIPT);
        fake.close_after = 12;
        Connected c(fake);
        SensorSession session(c.client, "F4724744", 1000);
        StartReport rep;
        EXPECT_FALSE(
            session.start({"10.11.10.1", 2115, 2116, true}, {}, true, rep));
        EXPECT_NE(rep.error.find("connection lost"), std::string::npos)
            << rep.error;
    }
    {
        FakeSensor fake(TRANSCRIPT);
        Connected c(fake);
        SensorSession session(c.client, "F4724744", 1000);
        StartReport rep;
        EXPECT_FALSE(
            session.start({"not-an-ip", 2115, 2116, true}, {}, true, rep));
    }
}

TEST(SensorSession, EventsDuringStartAreHarmless)
{
    FakeSensor fake(TRANSCRIPT);
    fake.events_before_reply = {"sSN ContaminationResult 0 0"};
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    StartReport rep;
    EXPECT_TRUE(session.start({"10.11.10.1", 2115, 2116, true}, {}, true, rep))
        << rep.error;
    EXPECT_EQ(rep.problems(), 0u);
}

TEST(SensorSettings, TimeSyncItems)
{
    SensorSettings s;
    auto names = [](const SensorSettings& x)
    {
        std::vector<std::string> n;
        for (const auto& it : settingItems(x))
        {
            if (it.name.rfind("TSC", 0) == 0)
            {
                n.push_back(it.write);
            }
        }
        return n;
    };
    EXPECT_EQ(names(s), std::vector<std::string>{"sWN TSCRole 0"});
    s.time_sync.mode = TimeSyncMode::Ptp;
    EXPECT_EQ(names(s), std::vector<std::string>{"sWN TSCRole 3"});
    s.time_sync.mode = TimeSyncMode::Ntp;
    s.time_sync.ntp_server = "10.11.10.1";
    s.time_sync.ntp_update_s = 10;
    EXPECT_EQ(
        names(s),
        (std::vector<std::string>{
            "sWN TSCRole 1",
            "sWN TSCTCSrvAddr +10 +11 +10 +1",
            "sWN TSCTCupdatetime +10",
            "sWN TSCTCtimezone +34"}));
    EXPECT_EQ(s.validate(), "");

    // the device rejects 0 and 3601 (sFA 30)
    s.time_sync.ntp_update_s = 0;
    EXPECT_NE(s.validate().find("NTP update"), std::string::npos);
    s.time_sync.ntp_update_s = 3601;
    EXPECT_NE(s.validate().find("NTP update"), std::string::npos);
    s.time_sync.ntp_update_s = 10;
    s.time_sync.ntp_server = "ntp.local";
    EXPECT_NE(s.validate().find("IPv4"), std::string::npos);
    s.time_sync.ntp_server = "";  // the driver fills in its own address
    EXPECT_EQ(s.validate(), "");

    EXPECT_EQ(timeSyncModeFromString("ptp"), TimeSyncMode::Ptp);
    EXPECT_FALSE(timeSyncModeFromString("gps"));
}

TEST(SensorSession, ReadsStatusLikeTheDevicePrintsIt)
{
    FakeSensor fake(TRANSCRIPT);
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    SensorStatus st;
    std::string err;

    // recorded values: 48.32 deg C (first recording), all sectors clean, no
    // contamination
    ASSERT_TRUE(session.readStatus(st, err)) << err;
    EXPECT_TRUE(st.have_state && st.have_temperature && st.have_contamination);
    EXPECT_EQ(st.state, 3);
    EXPECT_NEAR(st.temperature_c, 48.32f, 0.01f);
    EXPECT_FALSE(st.contamination_warning || st.contamination_error);
    ASSERT_TRUE(st.have_sectors);
    EXPECT_EQ(st.sectors, std::vector<int>(12, 1));

    // the second recording (29.05 deg C) and the manual's examples
    fake.setValue("OPcurtmpdev", "41E86666");
    ASSERT_TRUE(session.readStatus(st, err)) << err;
    EXPECT_NEAR(st.temperature_c, 29.05f, 0.01f);
    fake.setValue("OPcurtmpdev", "420C0000");
    fake.setValue("ContaminationResult", "1 0");
    fake.setValue("ContaminationData", "0 3 2 2 2 2 2 2 2 2 2 2");
    fake.setValue("DevSta", "7");
    ASSERT_TRUE(session.readStatus(st, err)) << err;
    EXPECT_FLOAT_EQ(st.temperature_c, 35.f);
    EXPECT_TRUE(st.contamination_warning);
    EXPECT_FALSE(st.contamination_error);
    EXPECT_EQ(st.sectors[0], 0);
    EXPECT_EQ(st.sectors[1], 3);
    EXPECT_STREQ(deviceStateName(st.state), "recoverable error");
    EXPECT_STREQ(sectorName(st.sectors[1]), "error");

    // a variable without values is left unset rather than failing
    fake.setValue("ContaminationData", "");
    ASSERT_TRUE(session.readStatus(st, err)) << err;
    EXPECT_FALSE(st.have_sectors);
    EXPECT_TRUE(st.have_state);
}

TEST(SensorSession, StatusReportsALostConnection)
{
    FakeSensor fake(TRANSCRIPT);
    fake.close_after = 2;  // DevSta and OPcurtmpdev answered, then dropped
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    SensorStatus st;
    std::string err;
    EXPECT_FALSE(session.readStatus(st, err));
    EXPECT_NE(err.find("ContaminationResult"), std::string::npos) << err;
}

TEST(SensorSession, StopDisablesOutput)
{
    FakeSensor fake(TRANSCRIPT);
    Connected c(fake);
    SensorSession session(c.client, "F4724744", 1000);
    StartReport rep;
    ASSERT_TRUE(session.start({"10.11.10.1", 2115, 2116, false}, {}, true, rep))
        << rep.error;
    EXPECT_EQ(fake.value("ImuDataEnable"), "0");
    std::string err;
    ASSERT_TRUE(session.stop(err)) << err;
    EXPECT_EQ(fake.value("ScanDataEnable"), "0");
    EXPECT_EQ(fake.requests().back().telegram, "sMN Run");
}
