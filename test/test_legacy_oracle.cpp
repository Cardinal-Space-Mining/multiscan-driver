/*  Locks in what the current driver publishes: the oracle (a copy of the
 *  driver's data path on the vendored parser) must agree with the independent
 *  specification in support/expected_points.hpp, on real captures and on
 *  synthetic segments covering configurations the sensor can produce.
 *
 *  Tests named LegacyBug_* assert known defects of the current driver. The
 *  rewrite must invert them (see the frame assembler tests in phase 3).
 */
#include <gtest/gtest.h>

#include <map>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

#include "oracle/legacy_pipeline.hpp"
#include "support/compact_packet.hpp"
#include "support/expected_points.hpp"
#include "support/mscap.hpp"

#include "point_fields.hpp"
#define MS_DRIVER_POINT_TYPE_FIELDS MS_POINT_FIELD_ENABLE_ALL
#include "point_type.hpp"

using namespace ms_test;

namespace
{

void expectPointsEqual(
    const ExpectedPoint& got,
    const ExpectedPoint& exp,
    const std::string& where)
{
    EXPECT_FLOAT_EQ(got.range, exp.range) << where;
    EXPECT_FLOAT_EQ(got.azimuth, exp.azimuth) << where;
    EXPECT_FLOAT_EQ(got.elevation, exp.elevation) << where;
    EXPECT_NEAR(got.x, exp.x, 1e-6f) << where;
    EXPECT_NEAR(got.y, exp.y, 1e-6f) << where;
    EXPECT_NEAR(got.z, exp.z, 1e-6f) << where;
    EXPECT_EQ(got.intensity, exp.intensity) << where;
    EXPECT_EQ(got.layer, exp.layer) << where;
    EXPECT_EQ(got.echo, exp.echo) << where;
    EXPECT_EQ(got.index, exp.index) << where;
    EXPECT_EQ(got.time_us, exp.time_us) << where;
    EXPECT_EQ(got.reflective, exp.reflective) << where;
}

/* Compare one published cloud against the concatenated expectation of the
 * telegrams that make up the frame. Stops at the first mismatching point to
 * keep failure output readable. */
void expectCloudMatches(
    const legacy::Cloud& cloud,
    const std::vector<ScanTelegram>& frame,
    int echo_selection,
    const std::string& name)
{
    std::vector<ExpectedPoint> exp;
    for (const auto& t : frame)
    {
        auto p = expectedPoints(t, echo_selection);
        exp.insert(exp.end(), p.begin(), p.end());
    }
    ASSERT_EQ(cloud.point_step, AllFieldsLayout::point_step) << name;
    ASSERT_EQ(cloud.pointCount(), exp.size()) << name;
    for (size_t i = 0; i < exp.size(); i++)
    {
        const auto got =
            readAllFieldsPoint(cloud.data.data() + i * cloud.point_step);
        const auto before = ::testing::Test::HasFailure();
        expectPointsEqual(got, exp[i], name + " point " + std::to_string(i));
        if (!before && ::testing::Test::HasFailure())
        {
            return;
        }
    }
}

struct FeedResult
{
    legacy::Pipeline pipe;
    std::vector<std::vector<ScanTelegram>>
        frames;  // telegrams per frame, arrival order
    std::vector<ImuTelegram> imus;
    explicit FeedResult(int echo_selection) : pipe(echo_selection) {}
};

std::unique_ptr<FeedResult> feedCapture(
    const std::string& name,
    int echo_selection = 1)
{
    auto r = std::make_unique<FeedResult>(echo_selection);
    std::map<uint64_t, size_t> frame_slot;
    for (const auto& d : readCapture(fixture(name)))
    {
        r->pipe.feed(d.data, d.recv_time_ns);
        if (auto t = parseScan(d.data))
        {
            const uint64_t f = t->modules.at(0).frame_number;
            if (!frame_slot.count(f))
            {
                frame_slot[f] = r->frames.size();
                r->frames.emplace_back();
            }
            r->frames[frame_slot[f]].push_back(std::move(*t));
        }
        else if (auto imu = parseImu(d.data))
        {
            r->imus.push_back(*imu);
        }
    }
    return r;
}

}  // namespace


TEST(LegacyOracle, PointFieldListMatchesAllFieldsLayout)
{
    using L = AllFieldsLayout;
    const std::vector<sensor_msgs::msg::PointField> fields =
        MS_DRIVER_POINT_FIELD_LIST;
    const std::vector<std::pair<std::string, uint32_t>> expected = {
        {         "x",          L::x},
        {         "y",          L::y},
        {         "z",          L::z},
        { "intensity",  L::intensity},
        {     "range",      L::range},
        {   "azimuth",    L::azimuth},
        { "elevation",  L::elevation},
        {     "layer",      L::layer},
        {      "echo",       L::echo},
        {     "index",      L::index},
        {        "tl",         L::tl},
        {        "th",         L::th},
        {"reflective", L::reflective}
    };
    ASSERT_EQ(fields.size(), expected.size());
    for (size_t i = 0; i < fields.size(); i++)
    {
        EXPECT_EQ(fields[i].name, expected[i].first);
        EXPECT_EQ(fields[i].offset, expected[i].second) << fields[i].name;
    }
}

TEST(LegacyOracle, MatchesSpecOnLiveCapture)
{
    auto r = feedCapture("live_1echo.mscap.gz");
    EXPECT_EQ(r->pipe.receive_rejects, 0u);
    EXPECT_EQ(r->pipe.parse_failures, 0u);
    ASSERT_EQ(r->frames.size(), 10u);
    ASSERT_EQ(r->pipe.clouds.size(), 10u);
    for (size_t f = 0; f < r->frames.size(); f++)
    {
        EXPECT_EQ(r->pipe.clouds[f].pointCount(), 5040u);
        expectCloudMatches(
            r->pipe.clouds[f],
            r->frames[f],
            1,
            "live frame " + std::to_string(f));
    }
}

TEST(LegacyOracle, MatchesSpecOnAllLayersCaptureForEachEcho)
{
    for (int echo = 1; echo <= 3; echo++)
    {
        auto r = feedCapture("all_layers_3echo.mscap.gz", echo);
        ASSERT_EQ(r->pipe.clouds.size(), r->frames.size());
        for (size_t f = 0; f < r->frames.size(); f++)
        {
            EXPECT_EQ(r->pipe.clouds[f].pointCount(), 10800u);
            expectCloudMatches(
                r->pipe.clouds[f],
                r->frames[f],
                echo,
                "all-layers echo " + std::to_string(echo) + " frame " +
                    std::to_string(f));
        }
    }
}

TEST(LegacyOracle, MatchesSpecOnInterlacedCapture)
{
    auto r = feedCapture("interlaced.mscap.gz");
    ASSERT_EQ(r->pipe.clouds.size(), r->frames.size());
    for (size_t f = 0; f < r->frames.size(); f++)
    {
        expectCloudMatches(
            r->pipe.clouds[f],
            r->frames[f],
            1,
            "interlaced frame " + std::to_string(f));
    }
}

TEST(LegacyOracle, ImuMatchesCapture)
{
    auto r = feedCapture("live_1echo.mscap.gz");
    ASSERT_EQ(r->pipe.imus.size(), r->imus.size());
    for (size_t i = 0; i < r->imus.size(); i++)
    {
        const auto& got = r->pipe.imus[i];
        const auto& exp = r->imus[i];
        for (int k = 0; k < 3; k++)
        {
            EXPECT_EQ(got.linear_acceleration[k], exp.acceleration[k]);
            EXPECT_EQ(got.angular_velocity[k], exp.angular_velocity[k]);
        }
        for (int k = 0; k < 4; k++)
        {
            EXPECT_EQ(got.orientation_wxyz[k], exp.orientation[k]);
        }
    }
}

TEST(LegacyOracle, MatchesSpecOnSyntheticConfigurations)
{
    struct Case
    {
        std::string name;
        SyntheticConfig cfg;
        int echo_selection;
    };
    std::vector<Case> cases;
    cases.push_back({"live", SyntheticConfig::live(), 1});
    cases.push_back({"all layers", SyntheticConfig{}, 1});
    {
        SyntheticConfig c;
        c.echoes = 3;
        cases.push_back({"3 echoes, select 2", c, 2});
        cases.push_back({"3 echoes, select 4 (falls back to 1)", c, 4});
    }
    {
        SyntheticConfig c = SyntheticConfig::live();
        c.content_echoes = ECHO_HAS_DISTANCE;
        cases.push_back({"no rssi", c, 1});
    }
    {
        SyntheticConfig c = SyntheticConfig::live();
        c.content_beams = BEAM_HAS_PROPERTIES;
        cases.push_back({"no per-beam azimuth", c, 1});
    }
    {
        SyntheticConfig c;
        c.distance_scale = 2.5f;
        cases.push_back({"distance scale 2.5", c, 1});
    }
    {
        SyntheticConfig c;
        c.layers = {
            true,
            false,
            true,
            false,
            true,
            true,
            false,
            true,
            true,
            true,
            false,
            false,
            true,
            false,
            true,
            true};
        cases.push_back({"sparse layer filter", c, 1});
    }
    for (const auto& c : cases)
    {
        legacy::Pipeline pipe(c.echo_selection);
        for (uint64_t frame = 5; frame < 7; frame++)
        {
            std::vector<ScanTelegram> telegrams;
            for (uint32_t seg = 0; seg < 12; seg++)
            {
                telegrams.push_back(makeSegment(c.cfg, seg, frame));
                pipe.feed(serialize(telegrams.back()), 0);
                if (seg == 5)
                {
                    pipe.feed(serialize(makeImu(frame * 50'000 + 20'000)), 0);
                }
            }
            ASSERT_EQ(pipe.clouds.size(), frame - 4) << c.name;
            expectCloudMatches(
                pipe.clouds.back(),
                telegrams,
                c.echo_selection,
                c.name);
        }
        EXPECT_EQ(pipe.imus.size(), 2u) << c.name;
    }
}

TEST(LegacyOracle, RejectsCorruptDatagrams)
{
    legacy::Pipeline pipe;
    auto bytes = serialize(makeSegment(SyntheticConfig::live(), 0, 1));
    bytes[500] ^= 1;
    pipe.feed(bytes, 0);
    EXPECT_EQ(pipe.receive_rejects, 1u);
    EXPECT_TRUE(pipe.clouds.empty());
}


// --- Known defects of the current driver --------------------------------------

TEST(LegacyOracle, LegacyBug_NarrowOutputRangeNeverPublishes)
{
    // the sensor sends only 6 of 12 segments at -10..+10 deg; the driver waits for all 12
    auto r = feedCapture("range_pm10.mscap.gz");
    EXPECT_EQ(r->frames.size(), 10u);
    EXPECT_EQ(r->pipe.clouds.size(), 0u);
}

TEST(LegacyOracle, LegacyBug_DroppedSegmentShiftsFramePhase)
{
    legacy::Pipeline pipe;
    for (uint64_t frame = 0; frame < 4; frame++)
    {
        for (uint32_t seg = 0; seg < 12; seg++)
        {
            if (frame == 1 && seg == 7)
            {
                continue;  // one lost datagram
            }
            pipe.feed(
                serialize(makeSegment(SyntheticConfig::live(), seg, frame)),
                0);
        }
    }
    // Frame 1 is never published: its segments 0-6 are overwritten by frame 2.
    // From then on every cloud is published at segment 7 and holds segments
    // 8-11 of the previous revolution plus 0-7 of the current one. Each cloud
    // is still one contiguous ~50 ms sweep, but frames no longer start at
    // segment 0 and the drop is invisible to consumers.
    ASSERT_EQ(pipe.clouds.size(), 3u);
    for (size_t c = 1; c < pipe.clouds.size(); c++)
    {
        uint64_t t_min = UINT64_MAX, t_max = 0;
        const auto& cloud = pipe.clouds[c];
        for (size_t i = 0; i < cloud.pointCount(); i++)
        {
            const auto p =
                readAllFieldsPoint(cloud.data.data() + i * cloud.point_step);
            t_min = std::min(t_min, p.time_us);
            t_max = std::max(t_max, p.time_us);
        }
        const uint64_t prev_frame_seg8 =
            1'000'000'000ULL + c * 50'000ULL + 8 * 4'167ULL;
        EXPECT_EQ(t_min, prev_frame_seg8)
            << "cloud " << c << " starts at segment 8 of frame " << c;
        EXPECT_LT(t_max - t_min, 55'000u) << "cloud " << c;
    }
}
