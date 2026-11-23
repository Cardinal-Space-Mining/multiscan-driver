/*  Frame assembly by frame number (src/frame_assembler.cpp).
 *
 *  The first tests replace the oracle's LegacyBug_* behaviour: a narrowed
 *  output range publishes again, and a dropped datagram yields one partial
 *  frame instead of shifting every later frame.
 */
#include <gtest/gtest.h>

#include <cstring>
#include <map>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "support/compact_packet.hpp"
#include "support/expected_points.hpp"
#include "support/mscap.hpp"

using namespace ms_test;
using multiscan::FrameAssembler;
using multiscan::PointLayout;
namespace compact = multiscan::compact;

namespace
{

uint32_t maskOf(std::initializer_list<uint32_t> segs)
{
    uint32_t m = 0;
    for (uint32_t s : segs)
    {
        m |= 1u << s;
    }
    return m;
}
constexpr uint32_t ALL = 0xFFF;

struct Harness
{
    explicit Harness(FrameAssembler::Config cfg = {}) :
        decoder(PointLayout::all()),
        assembler(decoder, cfg, [](uint64_t us, uint64_t) { return us * 1000; })
    {
    }

    void feed(const ScanTelegram& t, uint64_t recv_ns = 0)
    {
        sent[{t.modules[0].frame_number, t.modules[0].segment_counter}] = t;
        ASSERT_EQ(
            assembler.add(serialize(t), recv_ns, frames),
            compact::Error::None);
    }
    void feed(
        uint64_t frame,
        uint32_t seg,
        const SyntheticConfig& cfg = SyntheticConfig::live())
    {
        feed(
            makeSegment(cfg, seg, frame),
            frame * 50'000'000 + seg * 4'167'000);
    }

    /* A frame's points must be exactly its segments' points, in arrival
     * order, and nothing else. */
    void expectFrameContent(
        const FrameAssembler::Frame& f,
        const std::vector<uint32_t>& arrival_order)
    {
        std::vector<ExpectedPoint> exp;
        for (uint32_t s : arrival_order)
        {
            auto p = expectedPoints(sent.at({f.frame_number, s}), 1);
            exp.insert(exp.end(), p.begin(), p.end());
        }
        ASSERT_EQ(f.points, exp.size()) << "frame " << f.frame_number;
        ASSERT_EQ(f.data.size(), exp.size() * 52);
        for (size_t i = 0; i < exp.size(); i++)
        {
            const auto got = readAllFieldsPoint(f.data.data() + i * 52);
            ASSERT_EQ(got.index, exp[i].index)
                << "frame " << f.frame_number << " point " << i;
            ASSERT_EQ(got.layer, exp[i].layer);
            ASSERT_EQ(got.time_us, exp[i].time_us);
            ASSERT_EQ(got.range, exp[i].range);
        }
    }

    compact::ScanDecoder decoder;
    FrameAssembler assembler;
    std::vector<FrameAssembler::Frame> frames;
    std::map<std::pair<uint64_t, uint64_t>, ScanTelegram> sent;
};

std::vector<uint32_t> range(uint32_t a, uint32_t b)
{
    std::vector<uint32_t> v;
    for (uint32_t i = a; i < b; i++)
    {
        v.push_back(i);
    }
    return v;
}

}  // namespace


// --- Fixes for the legacy defects ---------------------------------------------------

TEST(FrameAssembler, NarrowOutputRangePublishesEveryFrame)
{
    // real capture: the sensor sends segments 0, 1, 5, 6, 7, 11 only
    compact::ScanDecoder dec(PointLayout::all());
    FrameAssembler a(dec, {}, [](uint64_t us, uint64_t) { return us * 1000; });
    std::vector<FrameAssembler::Frame> frames;
    std::vector<std::vector<ScanTelegram>> per_frame;
    for (const auto& d : readCapture(fixture("range_pm10.mscap.gz")))
    {
        auto t = parseScan(d.data);
        if (!t)
        {
            continue;
        }
        if (per_frame.empty() || per_frame.back()[0].modules[0].frame_number !=
                                     t->modules[0].frame_number)
        {
            per_frame.emplace_back();
        }
        per_frame.back().push_back(*t);
        ASSERT_EQ(a.add(d.data, d.recv_time_ns, frames), compact::Error::None);
    }
    const uint32_t sent = maskOf({0, 1, 5, 6, 7, 11});
    ASSERT_EQ(per_frame.size(), 10u);
    // the first 3 frames end partial (published when the next one starts) while
    // the assembler learns the pattern; from then on frames complete at segment 11
    ASSERT_EQ(frames.size(), 10u);
    for (size_t i = 0; i < frames.size(); i++)
    {
        EXPECT_EQ(frames[i].complete, i >= 3) << i;
        EXPECT_EQ(frames[i].segment_mask, sent) << i;
        std::vector<ExpectedPoint> exp;
        for (const auto& t : per_frame[i])
        {
            auto p = expectedPoints(t, 1);
            exp.insert(exp.end(), p.begin(), p.end());
        }
        ASSERT_EQ(frames[i].points, exp.size());
        for (size_t k = 0; k < exp.size(); k++)
        {
            ASSERT_EQ(
                readAllFieldsPoint(frames[i].data.data() + k * 52).index,
                exp[k].index);
        }
    }
    EXPECT_EQ(a.expectedMask(), sent);
    EXPECT_EQ(a.stats().frames_partial, 3u);
    EXPECT_EQ(a.stats().frames_complete, 7u);
    EXPECT_EQ(a.stats().telegrams_lost, 0u);
}

TEST(FrameAssembler, DroppedSegmentYieldsOnePartialFrame)
{
    Harness h;
    for (uint64_t f = 0; f < 4; f++)
    {
        for (uint32_t s = 0; s < 12; s++)
        {
            if (!(f == 1 && s == 7))
            {
                h.feed(f, s);
            }
        }
    }

    ASSERT_EQ(h.frames.size(), 4u);
    for (uint64_t f = 0; f < 4; f++)
    {
        const auto& fr = h.frames[f];
        EXPECT_EQ(fr.frame_number, f);
        EXPECT_EQ(fr.complete, f != 1);
        EXPECT_EQ(fr.segment_mask, f == 1 ? (ALL & ~maskOf({7})) : ALL);
        auto order = range(0, 12);
        if (f == 1)
        {
            order.erase(order.begin() + 7);
        }
        h.expectFrameContent(fr, order);
        // every cloud covers one revolution and starts at segment 0
        EXPECT_EQ(fr.time_min_us, 1'000'000'000ULL + f * 50'000ULL);
    }
    EXPECT_EQ(h.assembler.stats().telegrams_lost, 1u);
    EXPECT_EQ(h.assembler.stats().frames_partial, 1u);
    EXPECT_EQ(h.assembler.expectedMask(), ALL)
        << "a single loss must not change the expected set";
}


// --- Normal operation and edge cases -------------------------------------------------

TEST(FrameAssembler, CompleteFramesAreEmittedOnTheirLastSegment)
{
    Harness h;
    for (uint32_t s = 0; s < 11; s++)
    {
        h.feed(5, s);
    }
    EXPECT_TRUE(h.frames.empty());
    h.feed(5, 11);
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_TRUE(h.frames[0].complete);
    EXPECT_EQ(h.frames[0].stamp_ns, (1'000'000'000ULL + 5 * 50'000ULL) * 1000);
    h.expectFrameContent(h.frames[0], range(0, 12));
}

TEST(FrameAssembler, PartialFramesCanBeDiscarded)
{
    FrameAssembler::Config cfg;
    cfg.publish_partial_frames = false;
    Harness h(cfg);
    for (uint64_t f = 0; f < 3; f++)
    {
        for (uint32_t s = 0; s < 12; s++)
        {
            if (!(f == 1 && s == 3))
            {
                h.feed(f, s);
            }
        }
    }
    ASSERT_EQ(h.frames.size(), 2u);
    EXPECT_EQ(h.frames[0].frame_number, 0u);
    EXPECT_EQ(h.frames[1].frame_number, 2u);
    EXPECT_EQ(h.assembler.stats().frames_discarded, 1u);
}

TEST(FrameAssembler, DuplicateAndLateSegmentsAreDropped)
{
    Harness h;
    for (uint32_t s = 0; s < 6; s++)
    {
        h.feed(0, s);
    }
    h.feed(0, 3);  // duplicate
    for (uint32_t s = 6; s < 12; s++)
    {
        h.feed(0, s);
    }
    h.feed(0, 4);  // late: frame 0 was already emitted
    for (uint32_t s = 0; s < 12; s++)
    {
        h.feed(1, s);
    }
    h.feed(0, 9);  // late, older frame
    ASSERT_EQ(h.frames.size(), 2u);
    h.expectFrameContent(h.frames[0], range(0, 12));
    h.expectFrameContent(h.frames[1], range(0, 12));
    EXPECT_EQ(h.assembler.stats().segments_duplicate, 1u);
    EXPECT_EQ(h.assembler.stats().segments_late, 2u);
}

TEST(FrameAssembler, ReorderedSegmentsStayInTheirFrame)
{
    Harness h;
    const std::vector<uint32_t> order = {0, 1, 2, 4, 3, 5, 6, 7, 8, 10, 9, 11};
    for (uint32_t s : order)
    {
        h.feed(2, s);
    }
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_TRUE(h.frames[0].complete);
    h.expectFrameContent(h.frames[0], order);
    EXPECT_EQ(h.assembler.stats().telegrams_lost, 0u)
        << "reordering is not loss";
}

TEST(FrameAssembler, JoiningMidRevolutionDiscardsTheFirstFrameSilently)
{
    Harness h;
    for (uint32_t s = 5; s < 12; s++)
    {
        h.feed(7, s);
    }
    for (uint32_t s = 0; s < 12; s++)
    {
        h.feed(8, s);
    }
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0].frame_number, 8u);
    EXPECT_EQ(h.assembler.stats().frames_partial, 0u);
    EXPECT_EQ(h.assembler.stats().frames_discarded, 0u);
    EXPECT_EQ(h.assembler.expectedMask(), ALL);
}

TEST(FrameAssembler, MissedFramesAndLostTelegramsAreCounted)
{
    Harness h;
    for (uint32_t s = 0; s < 12; s++)
    {
        h.feed(0, s);
    }
    for (uint32_t s = 0; s < 12; s++)
    {
        h.feed(3, s);  // frames 1 and 2 never arrive
    }
    ASSERT_EQ(h.frames.size(), 2u);
    EXPECT_EQ(h.assembler.stats().frames_missed, 2u);
    EXPECT_EQ(h.assembler.stats().telegrams_lost, 24u);
}

TEST(FrameAssembler, SensorRestartResetsState)
{
    Harness h;
    for (uint32_t s = 0; s < 12; s++)
    {
        h.feed(5000, s);
    }
    for (uint32_t s = 0; s < 4; s++)
    {
        h.feed(5001, s);
    }
    // sensor reboots: frame numbers (and telegram counters) start over
    for (uint64_t f = 1; f < 3; f++)
    {
        for (uint32_t s = 0; s < 12; s++)
        {
            h.feed(f, s);
        }
    }
    ASSERT_EQ(h.frames.size(), 3u);
    EXPECT_EQ(h.frames[0].frame_number, 5000u);
    EXPECT_EQ(h.frames[1].frame_number, 1u);
    EXPECT_EQ(h.frames[2].frame_number, 2u);
    h.expectFrameContent(h.frames[1], range(0, 12));
    EXPECT_EQ(h.assembler.stats().sensor_restarts, 1u);
}

TEST(FrameAssembler, ExpectedSegmentsGrowBackWhenTheRangeWidens)
{
    Harness h;
    const std::vector<uint32_t> narrow = {0, 1, 2, 9, 10, 11};
    uint64_t f = 0;
    for (; f < 5; f++)
    {
        for (uint32_t s : narrow)
        {
            h.feed(f, s);
        }
    }
    EXPECT_EQ(h.assembler.expectedMask(), maskOf({0, 1, 2, 9, 10, 11}));
    h.frames.clear();
    // output range widened again: all segments arrive
    for (; f < 8; f++)
    {
        for (uint32_t s = 0; s < 12; s++)
        {
            h.feed(f, s);
        }
    }
    EXPECT_EQ(h.assembler.expectedMask(), ALL);
    // segment 3 arrives before the narrow set {.., 9, 10, 11} is satisfied,
    // so even the first wide frame is whole
    ASSERT_EQ(h.frames.size(), 3u);
    for (const auto& fr : h.frames)
    {
        EXPECT_TRUE(fr.complete);
        EXPECT_EQ(fr.segment_mask, ALL);
    }
}

TEST(FrameAssembler, StaleFramesAreFlushed)
{
    Harness h;
    for (uint32_t s = 0; s < 12; s++)
    {
        h.feed(0, s);
    }
    for (uint32_t s = 0; s < 8; s++)
    {
        h.feed(1, s);
    }
    ASSERT_EQ(h.frames.size(), 1u);
    const uint64_t last = 1 * 50'000'000ULL + 7 * 4'167'000ULL;
    h.assembler.flushStale(last + 50'000'000, h.frames);
    EXPECT_EQ(h.frames.size(), 1u);
    h.assembler.flushStale(last + 150'000'000, h.frames);
    ASSERT_EQ(h.frames.size(), 2u);
    EXPECT_FALSE(h.frames[1].complete);
    h.expectFrameContent(h.frames[1], range(0, 8));
    EXPECT_EQ(h.assembler.expectedMask(), ALL)
        << "stream stops don't teach the expected set";
}

TEST(FrameAssembler, GrowsBeyondTheInitialSegmentCapacity)
{
    FrameAssembler::Config cfg;
    cfg.initial_segment_points = 100;  // all-layer segments hold 900 points
    Harness h(cfg);
    SyntheticConfig all;
    all.echoes = 3;
    for (uint64_t f = 0; f < 2; f++)
    {
        for (uint32_t s = 0; s < 12; s++)
        {
            h.feed(f, s, all);
        }
    }
    ASSERT_EQ(h.frames.size(), 2u);
    h.expectFrameContent(h.frames[0], range(0, 12));
    h.expectFrameContent(h.frames[1], range(0, 12));
}

TEST(FrameAssembler, DecodeErrorsDoNotDisturbTheFrame)
{
    Harness h;
    for (uint32_t s = 0; s < 6; s++)
    {
        h.feed(0, s);
    }
    auto bad = serialize(makeSegment(SyntheticConfig::live(), 6, 0));
    bad[900] ^= 0xFF;
    EXPECT_EQ(h.assembler.add(bad, 0, h.frames), compact::Error::BadCrc);
    for (uint32_t s = 6; s < 12; s++)
    {
        h.feed(0, s);
    }
    ASSERT_EQ(h.frames.size(), 1u);
    h.expectFrameContent(h.frames[0], range(0, 12));
    EXPECT_EQ(h.assembler.stats().decode_errors, 1u);
}
