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

// Ghost filter timing on a capture (not a test):
//
//   bench_ghost_filter [MODEL] [CAPTURE] [--paced] [--no-early-exit]
//
// MODEL (*.model.txt) defaults to the driver's models/artifact_full.model.txt,
// CAPTURE (*.mscap or *.mscap.gz) to test/data/live_1echo.mscap.gz.
//
// Feeds the capture's segments through the assembler and the filter as the
// driver does (one thread), and reports the filter's work per segment, the
// time from the last segment of a frame to the finished frame (the latency
// the filter adds), and for comparison the whole frame classified at once.
// --paced feeds every segment at its recorded arrival time, as a live sensor
// would (idle gaps, cold caches). Run it on the target machine with the real
// model and a capture of a real scene (tools/mscap_capture.py).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "ghost/ghost_filter.hpp"
#include "support/mscap.hpp"

using namespace multiscan;
using Clock = std::chrono::steady_clock;

namespace
{

struct Options
{
    std::string model{std::string(MS_MODELS_DIR) + "/artifact_full.model.txt"};
    std::string capture{std::string(MS_TEST_DATA_DIR) + "/live_1echo.mscap.gz"};
    bool paced{false};
    bool early_exit{true};
};

bool endsWith(const std::string& text, const std::string& suffix)
{
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) ==
               0;
}

bool parseOptions(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; i++)
    {
        const std::string argument = argv[i];
        if (argument == "--paced")
        {
            options.paced = true;
        }
        else if (argument == "--no-early-exit")
        {
            options.early_exit = false;
        }
        else if (endsWith(argument, ".model.txt"))
        {
            options.model = argument;
        }
        else if (
            endsWith(argument, ".mscap") || endsWith(argument, ".mscap.gz"))
        {
            options.capture = argument;
        }
        else
        {
            return false;
        }
    }
    return true;
}

double toMicroseconds(Clock::duration duration)
{
    return std::chrono::duration<double, std::micro>(duration).count();
}

/* Timings in microseconds, reported in milliseconds. */
struct Timings
{
    std::vector<double> samples_us;

    void print(const char* name)
    {
        if (this->samples_us.empty())
        {
            return;
        }
        std::vector<double>& sorted = this->samples_us;
        std::sort(sorted.begin(), sorted.end());
        double sum = 0;
        for (const double sample : sorted)
        {
            sum += sample;
        }
        const size_t count = sorted.size();
        std::printf(
            "  %-34s n=%5zu  mean %7.2f  p50 %7.2f  p99 %7.2f  max %7.2f ms\n",
            name,
            count,
            sum / count * 1e-3,
            sorted[count / 2] * 1e-3,
            sorted[std::min(count - 1, count * 99 / 100)] * 1e-3,
            sorted.back() * 1e-3);
    }
};

/* The whole-frame computation the streamed filter must match. */
class WholeFrameReference
{
public:
    WholeFrameReference(const std::string& model, bool early_exit) :
        filter{artifact_filter::Model::load(model)}
    {
        this->filter.setEarlyExit(early_exit);
    }

    /* Classifies the frame at once; returns how many probabilities differ
     * from the artifact field of `filtered`. */
    size_t compare(
        const FrameAssembler::Frame& raw,
        const FrameAssembler::Frame& filtered,
        const PointLayout& layout,
        Timings& timings)
    {
        const uint32_t point_step = layout.pointStep();
        this->xyz.clear();
        this->intensity.clear();
        this->classified_points.clear();
        for (uint32_t point = 0; point < raw.num_points; point++)
        {
            const uint8_t* values = raw.data.data() + point * point_step;
            uint32_t layer;
            std::memcpy(&layer, values + layout.offset(Field::Layer), 4);
            if (layer == 5 || layer == 13)
            {
                continue;  // high-resolution layers aren't classified
            }
            float position_and_intensity[4];  // x y z intensity come first
            std::memcpy(position_and_intensity, values, 16);
            this->xyz.insert(
                this->xyz.end(),
                position_and_intensity,
                position_and_intensity + 3);
            this->intensity.push_back(position_and_intensity[3]);
            this->classified_points.push_back(point);
        }

        const Clock::time_point start = Clock::now();
        this->filter.process(
            this->xyz.data(),
            this->classified_points.size(),
            this->intensity.data(),
            static_cast<int64_t>(raw.stamp_ns),
            this->probability);
        timings.samples_us.push_back(toMicroseconds(Clock::now() - start));

        size_t differing = 0;
        for (size_t i = 0; i < this->classified_points.size(); i++)
        {
            float streamed;
            std::memcpy(
                &streamed,
                filtered.data.data() + this->classified_points[i] * point_step +
                    layout.offset(Field::Artifact),
                4);
            differing += std::memcmp(&streamed, &this->probability[i], 4) != 0;
        }
        return differing;
    }

protected:
    artifact_filter::Filter filter;
    std::vector<float> xyz;
    std::vector<float> intensity;
    std::vector<float> probability;
    std::vector<uint32_t> classified_points;
};

}  // namespace


int main(int argc, char** argv)
{
    Options options;
    if (!parseOptions(argc, argv, options))
    {
        std::fprintf(
            stderr,
            "usage: bench_ghost_filter [MODEL.model.txt] [CAPTURE.mscap[.gz]] "
            "[--paced] [--no-early-exit]\n");
        return 2;
    }

    std::string error;
    const PointLayout layout = *PointLayout::fromNames(
        {"x", "y", "z", "intensity", "layer", "index", "artifact"},
        error);
    GhostFilter filter(
        artifact_filter::Model::load(options.model),
        {.mode = GhostFilter::Mode::Annotate,
         .early_exit = options.early_exit});
    WholeFrameReference reference(options.model, options.early_exit);
    const artifact_filter::Model& model = filter.model();
    std::printf(
        "model %s (%s, %zu trees), early exit %s, %s\n",
        model.variant.c_str(),
        model.use_intensity ? "range + intensity" : "range only",
        model.tree_roots.size(),
        filter.earlyExitActive() ? "on" : "off",
        options.paced ? "paced at the recorded arrival times"
                      : "as fast as possible");

    // --- The driver's pipeline, timed ---
    compact::ScanDecoder decoder(layout);
    FrameAssembler assembler(
        decoder,
        {},
        [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; });
    Timings segment_work;
    Timings frame_work;
    Timings added_latency;
    Timings whole_frame;
    double last_segment_us = 0;
    double open_frame_work_us = 0;
    assembler.setSegmentHook(
        [&](const compact::ScanInfo& info,
            std::span<const uint8_t> datagram,
            uint64_t frame_number,
            uint32_t first_point)
        {
            const Clock::time_point start = Clock::now();
            filter.addSegment(
                datagram,
                frame_number,
                first_point,
                info.num_points);
            last_segment_us = toMicroseconds(Clock::now() - start);
            segment_work.samples_us.push_back(last_segment_us);
            open_frame_work_us += last_segment_us;
        });

    const std::vector<ms_test::CapturedDatagram> datagrams =
        ms_test::readCapture(options.capture);
    const uint64_t first_receive_ns =
        datagrams.empty() ? 0 : datagrams.front().receive_time_ns;
    const Clock::time_point replay_start = Clock::now();
    std::vector<FrameAssembler::Frame> emitted;
    size_t frames = 0;
    size_t unfiltered = 0;
    size_t differing = 0;
    for (const ms_test::CapturedDatagram& datagram : datagrams)
    {
        if (compact::telegramType(datagram.data) != compact::TelegramType::Scan)
        {
            continue;
        }
        if (options.paced)
        {
            std::this_thread::sleep_until(
                replay_start +
                std::chrono::nanoseconds(
                    datagram.receive_time_ns - first_receive_ns));
        }
        last_segment_us = 0;
        assembler.add(datagram.data, datagram.receive_time_ns, emitted);
        for (FrameAssembler::Frame& frame : emitted)
        {
            const FrameAssembler::Frame raw = frame;
            const Clock::time_point start = Clock::now();
            const GhostFilter::Result result =
                filter.finishFrame(frame, layout);
            const double finish_us = toMicroseconds(Clock::now() - start);
            frames++;
            if (!result.filtered)
            {
                unfiltered++;
                continue;
            }
            added_latency.samples_us.push_back(last_segment_us + finish_us);
            frame_work.samples_us.push_back(open_frame_work_us + finish_us);
            open_frame_work_us = 0;

            const size_t frame_differing =
                reference.compare(raw, frame, layout, whole_frame);
            if (frame_differing)
            {
                std::printf(
                    "frame %zu: %zu differ\n",
                    frames - 1,
                    frame_differing);
            }
            differing += frame_differing;
        }
        emitted.clear();
    }

    std::printf(
        "%s: %zu frames (%zu unfiltered), %zu probabilities differ from the "
        "whole-frame result\n",
        options.capture.c_str(),
        frames,
        unfiltered,
        differing);
    segment_work.print("work per segment");
    frame_work.print("work per frame (all segments)");
    added_latency.print("last segment -> frame done (added)");
    whole_frame.print("whole frame at once");
    return differing ? 1 : 0;
}
