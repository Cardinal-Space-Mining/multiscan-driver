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
#include "ghost/ghost_filter_worker.hpp"
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

// --- Fixtures ----------------------------------------------------------------

const std::string FULL_MODEL = fixture("ghost_full_30trees.model.txt");
const std::string XYZ_MODEL = fixture("ghost_xyz_30trees.model.txt");
const std::string LIVE_CAPTURE = fixture("live_1echo.mscap.gz");

PointLayout layoutWithArtifact()
{
    std::string error;
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
        error);
}

template<typename T> T loadField(const uint8_t* point, int32_t offset)
{
    T value;
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}

const uint8_t* pointAt(
    const FrameAssembler::Frame& frame,
    uint32_t point,
    const PointLayout& layout)
{
    return frame.data.data() + static_cast<size_t>(point) * layout.pointStep();
}

float artifactOf(
    const FrameAssembler::Frame& frame,
    uint32_t point,
    const PointLayout& layout)
{
    return loadField<float>(
        pointAt(frame, point, layout),
        layout.offset(Field::Artifact));
}

bool isHighResolutionLayer(uint32_t layer) { return layer == 5 || layer == 13; }

FrameAssembler::ClockOffset noClockOffset()
{
    return [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; };
}

/* Feeds every scan telegram of a capture to the assembler, and every frame
 * it emits to `onFrame`. */
template<typename OnFrame>
void feedCapture(
    const std::string& capture,
    FrameAssembler& assembler,
    OnFrame onFrame)
{
    std::vector<FrameAssembler::Frame> emitted;
    for (const CapturedDatagram& datagram : readCapture(capture))
    {
        if (compact::telegramType(datagram.data) != compact::TelegramType::Scan)
        {
            continue;
        }
        assembler.add(datagram.data, datagram.receive_time_ns, emitted);
        for (FrameAssembler::Frame& frame : emitted)
        {
            onFrame(frame);
        }
        emitted.clear();
    }
}


// --- Synchronous runs --------------------------------------------------------

/* Every frame of a capture, as the assembler produced it (`raw`) and as the
 * hook-fed filter left it. */
struct CaptureRun
{
    std::vector<FrameAssembler::Frame> raw;
    std::vector<FrameAssembler::Frame> filtered;
    std::vector<GhostFilter::Result> results;
};

CaptureRun runCapture(
    const std::string& capture,
    const PointLayout& layout,
    GhostFilter& filter,
    uint32_t echo_selection = 1)
{
    compact::ScanDecoder decoder(
        layout,
        compact::DecodeOptions{.echo_selection = echo_selection});
    FrameAssembler assembler(decoder, {}, noClockOffset());
    assembler.setSegmentHook(
        [&filter](
            const compact::ScanInfo& info,
            std::span<const uint8_t> datagram,
            uint64_t frame_number,
            uint32_t first_point)
        {
            filter.addSegment(
                datagram,
                frame_number,
                first_point,
                info.num_points);
        });

    CaptureRun run;
    feedCapture(
        capture,
        assembler,
        [&](FrameAssembler::Frame& frame)
        {
            run.raw.push_back(frame);
            run.results.push_back(filter.finishFrame(frame, layout));
            run.filtered.push_back(std::move(frame));
        });
    return run;
}

/* The whole-frame answer: Filter::process on the frame's low-resolution
 * layer points, -1 for the others. */
std::vector<std::vector<float>> wholeFrameProbabilities(
    const std::vector<FrameAssembler::Frame>& frames,
    const PointLayout& layout,
    const std::string& model,
    double threshold)
{
    artifact_filter::Filter filter{artifact_filter::Model::load(model)};
    filter.setEarlyExit(true);
    std::vector<std::vector<float>> probabilities;
    for (const FrameAssembler::Frame& frame : frames)
    {
        std::vector<float> xyz;
        std::vector<float> intensity;
        std::vector<uint32_t> classified_points;
        for (uint32_t point = 0; point < frame.num_points; point++)
        {
            const uint8_t* values = pointAt(frame, point, layout);
            if (isHighResolutionLayer(
                    loadField<uint32_t>(values, layout.offset(Field::Layer))))
            {
                continue;
            }
            for (const Field axis : {Field::X, Field::Y, Field::Z})
            {
                xyz.push_back(loadField<float>(values, layout.offset(axis)));
            }
            intensity.push_back(
                loadField<float>(values, layout.offset(Field::Intensity)));
            classified_points.push_back(point);
        }

        std::vector<float> classified_probability;
        filter.process(
            xyz.data(),
            classified_points.size(),
            intensity.data(),
            static_cast<int64_t>(frame.stamp_ns),
            classified_probability,
            threshold);
        std::vector<float> frame_probability(frame.num_points, -1.f);
        for (size_t i = 0; i < classified_points.size(); i++)
        {
            frame_probability[classified_points[i]] = classified_probability[i];
        }
        probabilities.push_back(std::move(frame_probability));
    }
    return probabilities;
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
    const CaptureRun run = runCapture(capture, layout, filter, echo_selection);
    ASSERT_GT(run.raw.size(), 2u);
    const std::vector<std::vector<float>> expected =
        wholeFrameProbabilities(run.raw, layout, model, filter.threshold());

    const uint32_t point_step = layout.pointStep();
    const int32_t artifact_offset = layout.offset(Field::Artifact);
    size_t total_classified = 0;
    for (size_t k = 0; k < run.filtered.size(); k++)
    {
        const FrameAssembler::Frame& frame = run.filtered[k];
        ASSERT_TRUE(run.results[k].filtered) << "frame " << k;
        ASSERT_EQ(frame.num_points, run.raw[k].num_points);

        size_t differing = 0;
        size_t classified = 0;
        for (uint32_t point = 0; point < frame.num_points; point++)
        {
            const float probability = artifactOf(frame, point, layout);
            differing +=
                std::memcmp(&probability, &expected[k][point], sizeof(float)) !=
                0;
            classified += probability >= 0.f;
        }
        EXPECT_EQ(differing, 0u) << capture << " frame " << k;
        EXPECT_EQ(classified, run.results[k].points_classified);
        total_classified += classified;

        // nothing else changed
        std::vector<uint8_t> raw_without_artifact = run.raw[k].data;
        std::vector<uint8_t> filtered_without_artifact = frame.data;
        for (uint32_t point = 0; point < frame.num_points; point++)
        {
            const size_t offset =
                static_cast<size_t>(point) * point_step + artifact_offset;
            std::memset(raw_without_artifact.data() + offset, 0, 4);
            std::memset(filtered_without_artifact.data() + offset, 0, 4);
        }
        EXPECT_EQ(raw_without_artifact, filtered_without_artifact);
    }
    EXPECT_GT(total_classified, 0u);
}


// --- Worker runs -------------------------------------------------------------

/* What a GhostFilterWorker published for a capture. */
struct WorkerRun
{
    std::vector<FrameAssembler::Frame> raw;
    std::vector<FrameAssembler::Frame> published;
    std::vector<GhostFilter::Result> results;
    GhostFilterWorker::Stats stats;
    uint64_t window_frames_after_take{0};
};

/* Runs a capture through a worker; `consumer_delay` makes publishing slow. */
WorkerRun runWorker(
    const std::string& capture,
    const PointLayout& layout,
    GhostFilter::Config config,
    uint32_t max_backlog,
    std::chrono::milliseconds consumer_delay = {})
{
    WorkerRun run;
    GhostFilterWorker worker(
        std::make_unique<GhostFilter>(
            artifact_filter::Model::load(FULL_MODEL),
            config),
        layout,
        [&](FrameAssembler::Frame& frame, const GhostFilter::Result& result)
        {
            std::this_thread::sleep_for(consumer_delay);
            run.published.push_back(std::move(frame));
            run.results.push_back(result);
        },
        max_backlog);

    compact::ScanDecoder decoder(layout);
    FrameAssembler assembler(decoder, {}, noClockOffset());
    assembler.setSegmentHook(
        [&worker](
            const compact::ScanInfo& info,
            std::span<const uint8_t> datagram,
            uint64_t frame_number,
            uint32_t first_point)
        { worker.segment(info, datagram, frame_number, first_point); });
    feedCapture(
        capture,
        assembler,
        [&](FrameAssembler::Frame& frame)
        {
            run.raw.push_back(frame);
            worker.frame(std::move(frame));
        });

    worker.drain();
    run.stats = worker.takeStats();
    run.window_frames_after_take = worker.takeStats().window_frames;
    return run;
}

}  // namespace



// --- Streamed = whole frame --------------------------------------------------

TEST(GhostFilter, StreamedEqualsWholeFrameFullModel)
{
    expectAnnotatedLikeWholeFrame(LIVE_CAPTURE, FULL_MODEL);
}

TEST(GhostFilter, StreamedEqualsWholeFrameXyzModel)
{
    expectAnnotatedLikeWholeFrame(LIVE_CAPTURE, XYZ_MODEL);
}

TEST(GhostFilter, HighResolutionLayersAreLeftOut)
{
    // all 16 layers, 3 echoes; the high-resolution layers' points stay -1
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



// --- Modes -------------------------------------------------------------------

TEST(GhostFilter, DropRemovesExactlyTheFlaggedPoints)
{
    const PointLayout layout = layoutWithArtifact();
    const uint32_t point_step = layout.pointStep();

    // a threshold that flags a good share of points with this small model
    GhostFilter annotating(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Annotate});
    const CaptureRun annotated = runCapture(LIVE_CAPTURE, layout, annotating);
    std::vector<float> probabilities;
    for (const FrameAssembler::Frame& frame : annotated.filtered)
    {
        for (uint32_t point = 0; point < frame.num_points; point++)
        {
            const float probability = artifactOf(frame, point, layout);
            if (probability >= 0.f)
            {
                probabilities.push_back(probability);
            }
        }
    }
    std::sort(probabilities.begin(), probabilities.end());
    const double threshold = probabilities[probabilities.size() * 9 / 10];

    GhostFilter dropping(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Drop, .threshold = threshold});
    const CaptureRun run = runCapture(LIVE_CAPTURE, layout, dropping);
    ASSERT_EQ(run.filtered.size(), annotated.filtered.size());
    size_t total_dropped = 0;
    for (size_t k = 0; k < run.filtered.size(); k++)
    {
        // expected: the raw frame's points the annotated run put under the
        // threshold
        const FrameAssembler::Frame& annotated_frame = annotated.filtered[k];
        const FrameAssembler::Frame& raw = run.raw[k];
        std::vector<uint8_t> expected;
        uint32_t flagged = 0;
        for (uint32_t point = 0; point < annotated_frame.num_points; point++)
        {
            if (artifactOf(annotated_frame, point, layout) >= threshold)
            {
                flagged++;
                continue;
            }
            const auto first_byte = raw.data.begin() + point * point_step;
            expected.insert(
                expected.end(),
                first_byte,
                first_byte + point_step);
        }
        EXPECT_EQ(run.results[k].points_flagged, flagged);
        EXPECT_EQ(run.filtered[k].num_points, raw.num_points - flagged);
        EXPECT_EQ(run.filtered[k].data, expected) << "frame " << k;
        total_dropped += flagged;
    }
    EXPECT_GT(total_dropped, 100u);
}



// --- Worker ------------------------------------------------------------------

TEST(GhostFilter, WorkerPublishesWhatTheFilterGives)
{
    const PointLayout layout = layoutWithArtifact();
    GhostFilter synchronous(
        artifact_filter::Model::load(FULL_MODEL),
        {.mode = GhostFilter::Mode::Annotate});
    const CaptureRun expected = runCapture(LIVE_CAPTURE, layout, synchronous);

    const WorkerRun run = runWorker(
        LIVE_CAPTURE,
        layout,
        {.mode = GhostFilter::Mode::Annotate},
        1000);  // never overloaded here
    EXPECT_EQ(run.stats.frames_filtered, expected.filtered.size());
    EXPECT_EQ(run.stats.frames_unfiltered, 0u);
    EXPECT_EQ(run.stats.window_frames, expected.filtered.size());
    EXPECT_EQ(run.window_frames_after_take, 0u);

    ASSERT_EQ(run.published.size(), expected.filtered.size());
    for (size_t k = 0; k < run.published.size(); k++)
    {
        EXPECT_TRUE(run.results[k].filtered);
        EXPECT_EQ(
            run.published[k].frame_number,
            expected.filtered[k].frame_number);
        EXPECT_EQ(run.published[k].data, expected.filtered[k].data)
            << "frame " << k;
    }
}

TEST(GhostFilter, WorkerThatFallsBehindPublishesUnfiltered)
{
    const PointLayout layout = layoutWithArtifact();
    // a slow consumer: the queue fills up behind it
    const WorkerRun run = runWorker(
        LIVE_CAPTURE,
        layout,
        {.mode = GhostFilter::Mode::Drop, .threshold = 0.},  // drops all
        1,
        std::chrono::milliseconds(20));

    ASSERT_EQ(run.published.size(), run.raw.size());
    EXPECT_GT(run.stats.frames_overloaded, 0u);
    EXPECT_EQ(
        run.stats.frames_filtered + run.stats.frames_unfiltered,
        run.published.size());
    for (size_t k = 0; k < run.published.size(); k++)
    {
        if (run.results[k].filtered)
        {
            EXPECT_LT(run.published[k].num_points, run.raw[k].num_points);
        }
        else
        {
            // untouched
            EXPECT_EQ(run.published[k].data, run.raw[k].data) << "frame " << k;
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
    FrameAssembler assembler(decoder, {}, noClockOffset());
    bool reset_done = false;
    uint32_t segments = 0;
    assembler.setSegmentHook(
        [&](const compact::ScanInfo& info,
            std::span<const uint8_t> datagram,
            uint64_t frame_number,
            uint32_t first_point)
        {
            filter.addSegment(
                datagram,
                frame_number,
                first_point,
                info.num_points);
            if (++segments == 3 && !reset_done)
            {
                filter.reset();  // e.g. a reconnect in the middle of a frame
                reset_done = true;
            }
        });
    std::vector<GhostFilter::Result> results;
    feedCapture(
        LIVE_CAPTURE,
        assembler,
        [&](FrameAssembler::Frame& frame)
        { results.push_back(filter.finishFrame(frame, layout)); });

    ASSERT_GT(results.size(), 2u);
    // the frame it was reset in has only some segments in the filter: left
    // as it is; the following ones are filtered
    EXPECT_FALSE(results[0].filtered);
    for (size_t k = 1; k < results.size(); k++)
    {
        EXPECT_TRUE(results[k].filtered) << "frame " << k;
    }
}



// --- Models ------------------------------------------------------------------

TEST(GhostFilter, RefusesModelsForOtherSensors)
{
    artifact_filter::Model model = artifact_filter::Model::load(FULL_MODEL);
    model.num_columns = 180;
    EXPECT_THROW(GhostFilter(std::move(model), {}), std::runtime_error);
    model = artifact_filter::Model::load(FULL_MODEL);
    model.scan_rate_hz = 10.;
    EXPECT_THROW(GhostFilter(std::move(model), {}), std::runtime_error);
}

TEST(GhostFilter, DefaultModelsFitTheDriver)
{
    // the models the driver installs (models/), as ghost_filter.model_file
    // and fallback_model_file name them by default
    const struct
    {
        const char* file;
        const char* variant;
        bool use_intensity;
    } defaults[] = {
        {"artifact_full.model.txt", "full",  true},
        { "artifact_xyz.model.txt",  "xyz", false},
    };
    for (const auto& expected : defaults)
    {
        const std::string path =
            std::string(MS_MODELS_DIR) + "/" + expected.file;
        artifact_filter::Model model = artifact_filter::Model::load(path);
        EXPECT_EQ(model.variant, expected.variant) << path;
        EXPECT_EQ(model.use_intensity, expected.use_intensity) << path;
        // without early exit a frame takes too long on one core
        EXPECT_TRUE(model.hasEarlyExit()) << path;
        const GhostFilter filter(std::move(model), {});
        EXPECT_TRUE(filter.earlyExitActive()) << path;
    }
}

TEST(GhostFilter, BrokenModelFilesAreRefused)
{
    EXPECT_THROW(
        artifact_filter::Model::load(fixture("does_not_exist.model.txt")),
        std::runtime_error);
    // a file that isn't a model
    EXPECT_THROW(
        artifact_filter::Model::load(fixture("golden_clouds.txt")),
        std::runtime_error);
}
