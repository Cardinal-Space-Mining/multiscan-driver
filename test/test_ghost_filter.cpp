// Ghost filter: frames filtered segment by segment must be exactly what the
// classifier gives for the whole frame at once.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "ghost/ghost_filter.hpp"
#include "support/mscap.hpp"

using namespace ms_test;
using multiscan::Field;
using multiscan::FrameAssembler;
using multiscan::GhostFilter;
using multiscan::GhostFilterWorker;
using multiscan::PointLayout;
namespace compact = multiscan::compact;

namespace
{

const std::string FULL_MODEL = fixture("ghost_full_30trees.model.txt");
const std::string XYZ_MODEL = fixture("ghost_xyz_30trees.model.txt");

PointLayout layoutWithArtifact()
{
    std::string err;
    return *PointLayout::fromNames(
        {"x",
         "y",
         "z",
         "intensity",
         "range",
         "azimuth",
         "elevation",
         "layer",
         "echo",
         "index",
         "time",
         "reflective",
         "artifact"},
        err);
}

template<typename T> T fieldOf(const uint8_t* point, int32_t offset)
{
    T v;
    std::memcpy(&v, point + offset, sizeof(T));
    return v;
}

bool isHd(uint32_t layer) { return layer == 5 || layer == 13; }

/* Runs a capture through the assembler. Every emitted frame is kept as the
 * assembler produced it (`raw`) and as the hook-fed filter left it. */
struct CaptureRun
{
    std::vector<FrameAssembler::Frame> raw, filtered;
    std::vector<GhostFilter::Result> results;
};

CaptureRun runCapture(
    const std::string& capture,
    const PointLayout& layout,
    GhostFilter* filter,
    uint32_t echo_selection = 1)
{
    compact::ScanDecoder decoder(
        layout,
        compact::DecodeOptions{.echo_selection = echo_selection});
    FrameAssembler assembler(
        decoder,
        {},
        [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; });
    if (filter)
    {
        assembler.setSegmentHook(
            [filter](
                const compact::ScanInfo& info,
                std::span<const uint8_t> dgram,
                uint64_t frame,
                uint32_t first)
            { filter->addSegment(dgram, frame, first, info.points); });
    }
    CaptureRun run;
    std::vector<FrameAssembler::Frame> out;
    for (const auto& d : readCapture(capture))
    {
        if (compact::telegramType(d.data) != compact::TelegramType::Scan)
        {
            continue;
        }
        assembler.add(d.data, d.recv_time_ns, out);
        for (auto& f : out)
        {
            run.raw.push_back(f);
            if (filter)
            {
                run.results.push_back(filter->finishFrame(f, layout));
            }
            run.filtered.push_back(std::move(f));
        }
        out.clear();
    }
    return run;
}

/* The whole-frame answer: Filter::process on the frame's low-resolution
 * layer points, -1 for the others. */
std::vector<std::vector<float>> reference(
    const std::vector<FrameAssembler::Frame>& frames,
    const PointLayout& layout,
    const std::string& model,
    double threshold)
{
    artifact_filter::Filter f(artifact_filter::Model::load(model));
    f.early_exit(true);
    const uint32_t step = layout.pointStep();
    std::vector<std::vector<float>> out;
    for (const auto& fr : frames)
    {
        std::vector<float> xyz, inten, prob;
        std::vector<uint32_t> idx;
        for (uint32_t i = 0; i < fr.points; i++)
        {
            const uint8_t* p = fr.data.data() + static_cast<size_t>(i) * step;
            if (isHd(fieldOf<uint32_t>(p, layout.offset(Field::Layer))))
            {
                continue;
            }
            for (Field c : {Field::X, Field::Y, Field::Z})
            {
                xyz.push_back(fieldOf<float>(p, layout.offset(c)));
            }
            inten.push_back(fieldOf<float>(p, layout.offset(Field::Intensity)));
            idx.push_back(i);
        }
        f.process(
            xyz.data(),
            idx.size(),
            inten.data(),
            static_cast<int64_t>(fr.stamp_ns),
            prob,
            threshold);
        std::vector<float> all(fr.points, -1.f);
        for (size_t k = 0; k < idx.size(); k++)
        {
            all[idx[k]] = prob[k];
        }
        out.push_back(std::move(all));
    }
    return out;
}

void expectAnnotatedLikeWholeFrame(
    const std::string& capture,
    const std::string& model,
    uint32_t echo_selection = 1)
{
    const PointLayout layout = layoutWithArtifact();
    GhostFilter filter(
        artifact_filter::Model::load(model),
        {.mode = GhostFilter::Mode::Annotate,
         .echo_selection = echo_selection});
    const CaptureRun run = runCapture(capture, layout, &filter, echo_selection);
    ASSERT_GT(run.raw.size(), 2u);
    const auto want = reference(run.raw, layout, model, filter.threshold());

    const uint32_t step = layout.pointStep();
    const int32_t o_art = layout.offset(Field::Artifact);
    size_t classified = 0;
    for (size_t k = 0; k < run.filtered.size(); k++)
    {
        const auto& fr = run.filtered[k];
        ASSERT_TRUE(run.results[k].filtered) << "frame " << k;
        ASSERT_EQ(fr.points, run.raw[k].points);
        size_t differ = 0, n_class = 0;
        for (uint32_t i = 0; i < fr.points; i++)
        {
            const float got = fieldOf<float>(fr.data.data() + i * step, o_art);
            differ += std::memcmp(&got, &want[k][i], sizeof(float)) != 0;
            n_class += got >= 0.f;
        }
        EXPECT_EQ(differ, 0u) << capture << " frame " << k;
        EXPECT_EQ(n_class, run.results[k].classified);
        classified += n_class;
        // nothing else changed
        auto a = run.raw[k].data, b = fr.data;
        for (uint32_t i = 0; i < fr.points; i++)
        {
            std::memset(a.data() + i * step + o_art, 0, 4);
            std::memset(b.data() + i * step + o_art, 0, 4);
        }
        EXPECT_EQ(a, b);
    }
    EXPECT_GT(classified, 0u);
}

}  // namespace


TEST(GhostFilter, StreamedEqualsWholeFrameFullModel)
{
    expectAnnotatedLikeWholeFrame(fixture("live_1echo.mscap.gz"), FULL_MODEL);
}

TEST(GhostFilter, StreamedEqualsWholeFrameXyzModel)
{
    expectAnnotatedLikeWholeFrame(fixture("live_1echo.mscap.gz"), XYZ_MODEL);
}

TEST(GhostFilter, HighResolutionLayersAreLeftOut)
{
    // all 16 layers, 3 echoes; the HD layers' points stay -1
    expectAnnotatedLikeWholeFrame(
        fixture("all_layers_3echo.mscap.gz"),
        FULL_MODEL,
        2);
}

TEST(GhostFilter, PartialFramesFromNarrowedOutputRange)
{
    // 6 of 12 segments: frames end when the next one starts until the
    // assembler learns the mask, which exercises finishing a frame before
    // the assembler emits it
    expectAnnotatedLikeWholeFrame(fixture("range_pm10.mscap.gz"), FULL_MODEL);
}

TEST(GhostFilter, DropRemovesExactlyTheFlaggedPoints)
{
    const PointLayout layout = layoutWithArtifact();
    // a threshold that flags a good share of points with this small model
    GhostFilter annotate(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Annotate});
    const CaptureRun ann =
        runCapture(fixture("live_1echo.mscap.gz"), layout, &annotate);
    std::vector<float> probs;
    const uint32_t step = layout.pointStep();
    const int32_t o_art = layout.offset(Field::Artifact);
    for (const auto& fr : ann.filtered)
    {
        for (uint32_t i = 0; i < fr.points; i++)
        {
            const float p = fieldOf<float>(fr.data.data() + i * step, o_art);
            if (p >= 0.f)
            {
                probs.push_back(p);
            }
        }
    }
    std::sort(probs.begin(), probs.end());
    const double thr = probs[probs.size() * 9 / 10];

    GhostFilter drop(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Drop, .threshold = thr});
    const CaptureRun run =
        runCapture(fixture("live_1echo.mscap.gz"), layout, &drop);
    ASSERT_EQ(run.filtered.size(), ann.filtered.size());
    size_t dropped = 0;
    for (size_t k = 0; k < run.filtered.size(); k++)
    {
        // expected: the annotated frame's points under the threshold
        std::vector<uint8_t> want;
        const auto& a = ann.filtered[k];
        const auto& raw = run.raw[k];
        uint32_t flagged = 0;
        for (uint32_t i = 0; i < a.points; i++)
        {
            const float p = fieldOf<float>(a.data.data() + i * step, o_art);
            if (p >= thr)
            {
                flagged++;
                continue;
            }
            want.insert(
                want.end(),
                raw.data.begin() + i * step,
                raw.data.begin() + (i + 1) * step);
        }
        EXPECT_EQ(run.results[k].flagged, flagged);
        EXPECT_EQ(run.filtered[k].points, raw.points - flagged);
        EXPECT_EQ(run.filtered[k].data, want) << "frame " << k;
        dropped += flagged;
    }
    EXPECT_GT(dropped, 100u);
}

TEST(GhostFilter, WorkerPublishesWhatTheFilterGives)
{
    const PointLayout layout = layoutWithArtifact();
    GhostFilter sync(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Annotate});
    const CaptureRun want =
        runCapture(fixture("live_1echo.mscap.gz"), layout, &sync);

    std::vector<FrameAssembler::Frame> published;
    std::vector<GhostFilter::Result> results;
    {
        GhostFilterWorker worker(
            std::make_unique<GhostFilter>(
                artifact_filter::Model::load(FULL_MODEL),
                GhostFilter::Config{.mode = GhostFilter::Mode::Annotate}),
            layout,
            [&](FrameAssembler::Frame& f, const GhostFilter::Result& r)
            {
                published.push_back(std::move(f));
                results.push_back(r);
            },
            1000);  // never overloaded here

        compact::ScanDecoder decoder(layout);
        FrameAssembler assembler(
            decoder,
            {},
            [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; });
        assembler.setSegmentHook(
            [&worker](
                const compact::ScanInfo& info,
                std::span<const uint8_t> dgram,
                uint64_t frame,
                uint32_t first) { worker.segment(info, dgram, frame, first); });
        std::vector<FrameAssembler::Frame> out;
        for (const auto& d : readCapture(fixture("live_1echo.mscap.gz")))
        {
            if (compact::telegramType(d.data) != compact::TelegramType::Scan)
            {
                continue;
            }
            assembler.add(d.data, d.recv_time_ns, out);
            for (auto& f : out)
            {
                worker.frame(std::move(f));
            }
            out.clear();
        }
        worker.drain();
        const auto stats = worker.takeStats();
        EXPECT_EQ(stats.frames_filtered, want.filtered.size());
        EXPECT_EQ(stats.frames_unfiltered, 0u);
        EXPECT_EQ(stats.window_frames, want.filtered.size());
        EXPECT_EQ(worker.takeStats().window_frames, 0u);
    }
    ASSERT_EQ(published.size(), want.filtered.size());
    for (size_t k = 0; k < published.size(); k++)
    {
        EXPECT_TRUE(results[k].filtered);
        EXPECT_EQ(published[k].frame_number, want.filtered[k].frame_number);
        EXPECT_EQ(published[k].data, want.filtered[k].data) << "frame " << k;
    }
}

TEST(GhostFilter, WorkerThatFallsBehindPublishesUnfiltered)
{
    const PointLayout layout = layoutWithArtifact();
    std::vector<FrameAssembler::Frame> published;
    std::vector<GhostFilter::Result> results;
    std::vector<FrameAssembler::Frame> raw;
    GhostFilterWorker::Stats stats;
    {
        GhostFilterWorker worker(
            std::make_unique<GhostFilter>(
                artifact_filter::Model::load(FULL_MODEL),
                GhostFilter::Config{
                    .mode = GhostFilter::Mode::Drop,
                    .threshold = 0.}),  // drops every return
            layout,
            [&](FrameAssembler::Frame& f, const GhostFilter::Result& r)
            {
                // a slow consumer: the queue fills up behind it
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                published.push_back(std::move(f));
                results.push_back(r);
            },
            1);

        compact::ScanDecoder decoder(layout);
        FrameAssembler assembler(
            decoder,
            {},
            [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; });
        assembler.setSegmentHook(
            [&worker](
                const compact::ScanInfo& info,
                std::span<const uint8_t> dgram,
                uint64_t frame,
                uint32_t first) { worker.segment(info, dgram, frame, first); });
        std::vector<FrameAssembler::Frame> out;
        for (const auto& d : readCapture(fixture("live_1echo.mscap.gz")))
        {
            if (compact::telegramType(d.data) != compact::TelegramType::Scan)
            {
                continue;
            }
            assembler.add(d.data, d.recv_time_ns, out);
            for (auto& f : out)
            {
                raw.push_back(f);
                worker.frame(std::move(f));
            }
            out.clear();
        }
        worker.drain();
        stats = worker.takeStats();
    }
    ASSERT_EQ(published.size(), raw.size());
    EXPECT_GT(stats.frames_overloaded, 0u);
    EXPECT_EQ(
        stats.frames_filtered + stats.frames_unfiltered,
        published.size());
    for (size_t k = 0; k < published.size(); k++)
    {
        if (results[k].filtered)
        {
            EXPECT_LT(published[k].points, raw[k].points);
        }
        else
        {
            // untouched
            EXPECT_EQ(published[k].data, raw[k].data) << "frame " << k;
        }
    }
}

TEST(GhostFilter, ResetForgetsTheFrameInProgress)
{
    const PointLayout layout = layoutWithArtifact();
    GhostFilter filter(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Annotate});
    compact::ScanDecoder decoder(layout);
    FrameAssembler assembler(
        decoder,
        {},
        [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; });
    bool reset_done = false;
    uint32_t segments = 0;
    assembler.setSegmentHook(
        [&](const compact::ScanInfo& info,
            std::span<const uint8_t> dgram,
            uint64_t frame,
            uint32_t first)
        {
            filter.addSegment(dgram, frame, first, info.points);
            if (++segments == 3 && !reset_done)
            {
                filter.reset();  // e.g. a reconnect in the middle of a frame
                reset_done = true;
            }
        });
    std::vector<FrameAssembler::Frame> out;
    std::vector<GhostFilter::Result> results;
    for (const auto& d : readCapture(fixture("live_1echo.mscap.gz")))
    {
        if (compact::telegramType(d.data) != compact::TelegramType::Scan)
        {
            continue;
        }
        assembler.add(d.data, d.recv_time_ns, out);
        for (auto& f : out)
        {
            results.push_back(filter.finishFrame(f, layout));
        }
        out.clear();
    }
    ASSERT_GT(results.size(), 2u);
    // the frame it was reset in has only some segments in the filter: left
    // as it is; the following ones are filtered
    EXPECT_FALSE(results[0].filtered);
    for (size_t k = 1; k < results.size(); k++)
    {
        EXPECT_TRUE(results[k].filtered) << "frame " << k;
    }
}

TEST(GhostFilter, RefusesModelsForOtherSensors)
{
    artifact_filter::Model m = artifact_filter::Model::load(FULL_MODEL);
    m.cols = 180;
    EXPECT_THROW(GhostFilter(std::move(m), {}), std::runtime_error);
    m = artifact_filter::Model::load(FULL_MODEL);
    m.scan_rate_hz = 10.;
    EXPECT_THROW(GhostFilter(std::move(m), {}), std::runtime_error);
}
