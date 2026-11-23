/*  Checks that the independent Compact reader/writer (support/compact_packet.hpp)
 *  reproduces real sensor traffic byte for byte, and records the sensor
 *  behaviour the rewrite relies on, as observed on 2026-11-22 with a
 *  multiScan136 on firmware 2.3.0 (serial 23170008).
 */
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <set>

#include "support/compact_packet.hpp"
#include "support/mscap.hpp"

using namespace ms_test;

namespace
{
constexpr uint16_t SCAN_PORT = 2115;
constexpr uint16_t IMU_PORT = 2116;

const char* const FIXTURES[] = {
    "live_1echo.mscap.gz",  // layers 6+14 filtered, last echo (team default)
    "all_layers_3echo.mscap.gz",  // all 16 layers, all echoes
    "range_pm10.mscap.gz",        // LMPoutputRange -10..+10 deg
    "interlaced.mscap.gz",        // mCLsetscancfglist 1
};

std::vector<ScanTelegram> scans(const std::string& name)
{
    std::vector<ScanTelegram> out;
    for (const auto& d : readCapture(fixture(name)))
    {
        if (d.port == SCAN_PORT)
        {
            auto t = parseScan(d.data);
            EXPECT_TRUE(t.has_value()) << name;
            if (t)
            {
                out.push_back(std::move(*t));
            }
        }
    }
    return out;
}
}  // namespace


TEST(CompactPacket, RealCapturesRoundTripExactly)
{
    for (const char* name : FIXTURES)
    {
        size_t n_scan = 0, n_imu = 0;
        for (const auto& d : readCapture(fixture(name)))
        {
            ASSERT_TRUE(crcValid(d.data)) << name;
            if (d.port == SCAN_PORT)
            {
                auto t = parseScan(d.data);
                ASSERT_TRUE(t.has_value()) << name;
                EXPECT_EQ(serialize(*t), d.data) << name;
                n_scan++;
            }
            else
            {
                ASSERT_EQ(d.port, IMU_PORT);
                auto t = parseImu(d.data);
                ASSERT_TRUE(t.has_value()) << name;
                EXPECT_EQ(serialize(*t), d.data) << name;
                n_imu++;
            }
        }
        EXPECT_GT(n_scan, 40u) << name;
        // range_pm10 was captured on the scan port only
        if (std::string(name) != "range_pm10.mscap.gz")
        {
            EXPECT_GT(n_imu, 15u) << name;
        }
    }
}

TEST(CompactPacket, SyntheticSegmentsMatchRealSizes)
{
    // sizes of real datagrams for the same configurations
    EXPECT_EQ(
        serialize(makeSegment(SyntheticConfig::live(), 3, 7)).size(),
        3456u);
    SyntheticConfig all;
    all.echoes = 3;
    EXPECT_EQ(serialize(makeSegment(all, 3, 7)).size(), 14160u);
}

TEST(CompactPacket, SyntheticSegmentsRoundTrip)
{
    std::vector<SyntheticConfig> cfgs(6);
    cfgs[0] = SyntheticConfig::live();
    cfgs[2].echoes = 3;
    cfgs[3].content_echoes = ECHO_HAS_DISTANCE;
    cfgs[4].content_beams = BEAM_HAS_PROPERTIES;
    cfgs[5].distance_scale = 2.5f;
    cfgs[5].layers = {
        true,
        false,
        false,
        true,
        true,
        true,
        false,
        true,
        false,
        false,
        false,
        false,
        false,
        true,
        false,
        false};
    for (const auto& cfg : cfgs)
    {
        for (uint32_t seg = 0; seg < 12; seg++)
        {
            const auto bytes = serialize(makeSegment(cfg, seg, 42));
            auto t = parseScan(bytes);
            ASSERT_TRUE(t.has_value());
            EXPECT_EQ(serialize(*t), bytes);
        }
    }
}

TEST(CompactPacket, RejectsCorruption)
{
    auto bytes = serialize(makeSegment(SyntheticConfig::live(), 0, 1));
    auto bad = bytes;
    bad[1000] ^= 0x40;
    EXPECT_FALSE(parseScan(bad).has_value());
    EXPECT_FALSE(
        parseScan(std::span(bytes).first(bytes.size() - 1)).has_value());
}


// --- Sensor behaviour the rewrite depends on ---------------------------------

TEST(SensorBehaviour, FramesAreTwelveSegmentsKeyedByFrameNumber)
{
    const auto t = scans("live_1echo.mscap.gz");
    std::map<uint64_t, std::vector<uint64_t>> frames;
    for (size_t i = 0; i < t.size(); i++)
    {
        ASSERT_EQ(t[i].modules.size(), 2u);
        for (const auto& m : t[i].modules)
        {
            // every module of a telegram carries the same segment and frame numbers
            EXPECT_EQ(m.segment_counter, t[i].modules[0].segment_counter);
            EXPECT_EQ(m.frame_number, t[i].modules[0].frame_number);
            EXPECT_EQ(m.lines, 7u);
            EXPECT_EQ(m.beams, 30u);
            EXPECT_EQ(m.echoes, 1u);
        }
        // the telegram counter counts scan telegrams only (IMU telegrams are interleaved)
        if (i)
        {
            EXPECT_EQ(t[i].telegram_counter, t[i - 1].telegram_counter + 1);
        }
        frames[t[i].modules[0].frame_number].push_back(
            t[i].modules[0].segment_counter);
    }
    ASSERT_EQ(frames.size(), 10u);
    const std::vector<uint64_t> all = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    for (const auto& [f, segs] : frames)
    {
        EXPECT_EQ(segs, all) << "frame " << f;
    }
}

TEST(SensorBehaviour, AllLayersModuleOrderIsLowResThenHighRes)
{
    const auto t = scans("all_layers_3echo.mscap.gz");
    ASSERT_FALSE(t.empty());
    const auto& m = t[0].modules;
    ASSERT_EQ(m.size(), 4u);
    EXPECT_EQ(m[0].lines, 7u);
    EXPECT_EQ(m[1].lines, 7u);
    EXPECT_EQ(m[2].lines, 1u);
    EXPECT_EQ(m[2].beams, 240u);
    EXPECT_EQ(m[3].lines, 1u);
    EXPECT_EQ(m[3].beams, 240u);
    for (const auto& mm : m)
    {
        EXPECT_EQ(mm.echoes, 3u);
    }
}

TEST(SensorBehaviour, NarrowOutputRangeOmitsSegments)
{
    // with -10..+10 deg only segments where either measuring unit sees the range are sent
    std::set<uint64_t> segs;
    for (const auto& t : scans("range_pm10.mscap.gz"))
    {
        segs.insert(t.modules[0].segment_counter);
    }
    EXPECT_EQ(segs, (std::set<uint64_t>{0, 1, 5, 6, 7, 11}));
}

TEST(SensorBehaviour, InterlacedShiftsAzimuthByEighthDegreePerFrame)
{
    std::vector<float> theta;
    for (const auto& t : scans("interlaced.mscap.gz"))
    {
        if (t.modules[0].segment_counter == 1)
        {
            theta.push_back(t.modules[0].theta_start[0]);
        }
        EXPECT_EQ(t.modules[0].beams, 30u);
    }
    ASSERT_GE(theta.size(), 9u);
    constexpr double EIGHTH_DEG = 0.125 * M_PI / 180.0;
    for (size_t i = 1; i < theta.size(); i++)
    {
        double d = theta[i] - theta[i - 1];
        if (d < -3 * EIGHTH_DEG)
        {
            d += 8 * EIGHTH_DEG;  // wraps after 8 frames
        }
        EXPECT_NEAR(d, EIGHTH_DEG, 1e-4) << i;
    }
}
