// Ghost filter timing on a capture (not a test):
//
//   bench_ghost_filter MODEL [CAPTURE] [--paced] [--no-early-exit]
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
double us(Clock::duration d)
{
    return std::chrono::duration<double, std::micro>(d).count();
}

struct Stat
{
    std::vector<double> v;
    void print(const char* name)
    {
        if (v.empty())
        {
            return;
        }
        std::sort(v.begin(), v.end());
        double s = 0;
        for (double x : v)
        {
            s += x;
        }
        std::printf(
            "  %-34s n=%5zu  mean %7.2f  p50 %7.2f  p99 %7.2f  max %7.2f ms\n",
            name,
            v.size(),
            s / v.size() * 1e-3,
            v[v.size() / 2] * 1e-3,
            v[std::min(v.size() - 1, v.size() * 99 / 100)] * 1e-3,
            v.back() * 1e-3);
    }
};
}  // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> pos;
    bool paced = false, early = true;
    for (int i = 1; i < argc; i++)
    {
        const std::string a = argv[i];
        if (a == "--paced")
        {
            paced = true;
        }
        else if (a == "--no-early-exit")
        {
            early = false;
        }
        else
        {
            pos.push_back(a);
        }
    }
    if (pos.empty())
    {
        std::fprintf(
            stderr,
            "usage: bench_ghost_filter MODEL [CAPTURE] [--paced] [--no-early-exit]\n");
        return 2;
    }
    const std::string capture =
        pos.size() > 1 ? pos[1]
                       : std::string(MS_TEST_DATA_DIR) + "/live_1echo.mscap.gz";

    std::string err;
    const PointLayout layout = *PointLayout::fromNames(
        {"x", "y", "z", "intensity", "layer", "index", "artifact"},
        err);
    GhostFilter filter(
        artifact_filter::Model::load(pos[0]),
        {.mode = GhostFilter::Mode::Annotate, .early_exit = early});
    artifact_filter::Filter whole(artifact_filter::Model::load(pos[0]));
    whole.early_exit(early);
    const auto& m = filter.model();
    std::printf(
        "model %s (%s, %zu trees), early exit %s, %s\n",
        m.variant.c_str(),
        m.use_intensity ? "range + intensity" : "range only",
        m.tree_root.size(),
        filter.early_exit_active() ? "on" : "off",
        paced ? "paced at the recorded arrival times" : "as fast as possible");

    compact::ScanDecoder decoder(layout);
    FrameAssembler assembler(
        decoder,
        {},
        [](const compact::ScanInfo&, uint64_t) -> int64_t { return 0; });
    Stat seg, last, whole_t, work;
    double hook_us = 0, frame_work_us = 0;
    assembler.setSegmentHook(
        [&](const compact::ScanInfo& info,
            std::span<const uint8_t> dgram,
            uint64_t frame,
            uint32_t first)
        {
            const auto t0 = Clock::now();
            filter.addSegment(dgram, frame, first, info.points);
            hook_us = us(Clock::now() - t0);
            seg.v.push_back(hook_us);
            frame_work_us += hook_us;
        });

    const auto caps = ms_test::readCapture(capture);
    const uint64_t recv0 = caps.empty() ? 0 : caps.front().recv_time_ns;
    const auto start = Clock::now();
    const uint32_t step = layout.pointStep();
    std::vector<FrameAssembler::Frame> out;
    std::vector<float> xyz, inten, prob;
    size_t frames = 0, unfiltered = 0, mismatched = 0;
    for (const auto& d : caps)
    {
        if (compact::telegramType(d.data) != compact::TelegramType::Scan)
        {
            continue;
        }
        if (paced)
        {
            std::this_thread::sleep_until(
                start + std::chrono::nanoseconds(d.recv_time_ns - recv0));
        }
        hook_us = 0;
        assembler.add(d.data, d.recv_time_ns, out);
        for (auto& f : out)
        {
            const FrameAssembler::Frame raw = f;
            const auto t0 = Clock::now();
            const GhostFilter::Result r = filter.finishFrame(f, layout);
            const double fin = us(Clock::now() - t0);
            frames++;
            if (!r.filtered)
            {
                unfiltered++;
                continue;
            }
            last.v.push_back(hook_us + fin);
            work.v.push_back(frame_work_us + fin);
            frame_work_us = 0;

            // the same frame at once, for comparison (and as a check)
            xyz.clear();
            inten.clear();
            std::vector<uint32_t> idx;
            for (uint32_t i = 0; i < raw.points; i++)
            {
                const uint8_t* p = raw.data.data() + i * step;
                uint32_t layer;
                std::memcpy(&layer, p + layout.offset(Field::Layer), 4);
                if (layer == 5 || layer == 13)
                {
                    continue;
                }
                float v[4];
                std::memcpy(v, p, 16);
                xyz.insert(xyz.end(), v, v + 3);
                inten.push_back(v[3]);
                idx.push_back(i);
            }
            const auto t1 = Clock::now();
            whole.process(
                xyz.data(),
                idx.size(),
                inten.data(),
                static_cast<int64_t>(raw.stamp_ns),
                prob);
            whole_t.v.push_back(us(Clock::now() - t1));
            size_t differ = 0;
            for (size_t k = 0; k < idx.size(); k++)
            {
                float got;
                std::memcpy(
                    &got,
                    f.data.data() + idx[k] * step +
                        layout.offset(Field::Artifact),
                    4);
                differ += std::memcmp(&got, &prob[k], 4) != 0;
            }
            if (differ)
            {
                std::printf("frame %zu: %zu differ\n", frames - 1, differ);
            }
            mismatched += differ;
        }
        out.clear();
    }
    std::printf(
        "%s: %zu frames (%zu unfiltered), %zu probabilities differ from "
        "the whole-frame result\n",
        capture.c_str(),
        frames,
        unfiltered,
        mismatched);
    seg.print("work per segment");
    work.print("work per frame (all segments)");
    last.print("last segment -> frame done (added)");
    whole_t.print("whole frame at once");
    return mismatched ? 1 : 0;
}
