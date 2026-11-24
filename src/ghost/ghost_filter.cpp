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

#include "ghost/ghost_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>


namespace multiscan
{

namespace
{
constexpr double RAD2DEG = 180. / 3.14159265358979323846;
constexpr uint32_t HD_LAYER_A = 5, HD_LAYER_B = 13;  // 0.125 deg layers

PointLayout filterLayout()
{
    std::string error;
    return *PointLayout::fromNames(
        {"x", "y", "z", "intensity", "azimuth", "elevation", "layer"},
        error);
}

template<typename T> T loadAt(const uint8_t* p, int32_t offset)
{
    T v;
    std::memcpy(&v, p + offset, sizeof(T));
    return v;
}
}  // namespace


// --- GhostFilter -------------------------------------------------------------

GhostFilter::GhostFilter(artifact_filter::Model model, Config config) :
    filter{std::move(model)},
    cfg{config},
    thr{config.threshold >= 0. ? config.threshold : filter.model().threshold},
    decoder{
        filterLayout(),
        compact::DecodeOptions{.echo_selection = config.echo_selection}}
{
    const artifact_filter::Model& m = filter.model();
    // the multiScan's 14 low-resolution layers, 1 degree columns, 20 Hz
    if (m.rings != 14 || m.cols != 360 ||
        (m.scan_rate_hz > 0. && std::abs(m.scan_rate_hz - 20.) > 0.5))
    {
        throw std::runtime_error(
            "the model was made for another sensor (" +
            std::to_string(m.rings) + " rings, " + std::to_string(m.cols) +
            " columns, " + std::to_string(m.scan_rate_hz) +
            " Hz; the multiScan has 14, 360, 20)");
    }
    filter.early_exit(cfg.early_exit);
    const PointLayout& L = decoder.layout();
    o_int = L.offset(Field::Intensity);
    o_az = L.offset(Field::Azimuth);
    o_el = L.offset(Field::Elevation);
    o_layer = L.offset(Field::Layer);
    step = L.pointStep();
}

void GhostFilter::reset()
{
    // the rest of a frame in progress is left unfiltered
    const bool was_open = open;
    const uint64_t number = current.number;
    current = {};
    current.valid = current.broken = was_open;
    current.number = number;
    finished = {};
    filter.reset();
}

void GhostFilter::addSegment(
    std::span<const uint8_t> datagram,
    uint64_t frame_number,
    uint32_t first_point,
    uint32_t points,
    bool skip)
{
    if (!open || frame_number != current.number)
    {
        // a new frame: the one before is complete (its frame, if the
        // assembler emits it, follows)
        if (open)
        {
            finishOpen();
        }
        current = {};
        current.valid = true;
        current.number = frame_number;
        open = true;
    }
    if (current.broken)
    {
        return;
    }
    if (skip)
    {
        current.broken = current.skipped = true;
        return;
    }

    if (buffer.size() < static_cast<size_t>(points) * step)
    {
        buffer.resize(static_cast<size_t>(points) * step);
    }
    compact::ScanInfo info;
    const compact::Error err =
        decoder.decode(datagram, buffer.data(), buffer.size() / step, info);
    if (err != compact::Error::None || info.points != points)
    {
        current.broken = true;  // not the points the frame holds
        return;
    }
    if (!current.begun)
    {
        filter.begin(static_cast<int64_t>(info.start_time_us) * 1000);
        current.begun = true;
    }

    // Points in runs of consecutive low-resolution layer points (the decoder
    // writes every line's beams together)
    const uint8_t* base = buffer.data();
    const size_t first_run = current.runs.size();
    uint32_t i = 0;
    while (i < points)
    {
        while (i < points)
        {
            const uint32_t layer = loadAt<uint32_t>(base + i * step, o_layer);
            if (layer != HD_LAYER_A && layer != HD_LAYER_B)
            {
                break;
            }
            i++;
        }
        const uint32_t start = i;
        while (i < points)
        {
            const uint32_t layer = loadAt<uint32_t>(base + i * step, o_layer);
            if (layer == HD_LAYER_A || layer == HD_LAYER_B)
            {
                break;
            }
            i++;
        }
        if (i > start)
        {
            const uint8_t* p = base + static_cast<size_t>(start) * step;
            filter.add(
                i - start,
                reinterpret_cast<const float*>(p),
                step,
                reinterpret_cast<const float*>(p + o_int),
                step);
            current.runs.push_back({first_point + start, i - start});
        }
    }
    // Every beam of the segment, with or without a return, is complete now.
    for (size_t ri = first_run; ri < current.runs.size(); ri++)
    {
        const Run& r = current.runs[ri];
        const uint8_t* p =
            base + static_cast<size_t>(r.frame_point - first_point) * step;
        for (uint32_t k = 0; k < r.count; k++, p += step)
        {
            filter.cover(filter.cell_at(
                loadAt<float>(p, o_az) * RAD2DEG,
                loadAt<float>(p, o_el) * RAD2DEG));
        }
    }
    filter.classify_ready();
}

void GhostFilter::finishOpen()
{
    if (current.broken || !current.begun)
    {
        // what was added is incomplete: don't compare the next frame with it
        filter.reset();
        current.prob.clear();
        current.broken = true;
    }
    else
    {
        filter.finish(current.prob, thr);
    }
    finished = std::move(current);
    current = {};
    open = false;
}

GhostFilter::Result GhostFilter::finishFrame(
    FrameAssembler::Frame& frame,
    const PointLayout& layout)
{
    if (open && current.number == frame.frame_number)
    {
        finishOpen();
    }
    Result r;
    if (!finished.valid || finished.number != frame.frame_number)
    {
        return r;  // the filter never saw this frame
    }
    FrameState done = std::move(finished);
    finished = {};
    r.skipped = done.skipped;
    if (done.broken)
    {
        return r;
    }

    const uint32_t fstep = layout.pointStep();
    const int32_t o_art = layout.offset(Field::Artifact);
    if (frame.data.size() < static_cast<size_t>(frame.points) * fstep)
    {
        return r;
    }
    for (const Run& run : done.runs)
    {
        if (static_cast<uint64_t>(run.frame_point) + run.count > frame.points)
        {
            return r;  // doesn't match the frame
        }
    }

    std::vector<uint8_t> drop;
    if (cfg.mode == Mode::Drop)
    {
        drop.assign(frame.points, 0);
    }
    size_t k = 0;
    for (const Run& run : done.runs)
    {
        for (uint32_t j = 0; j < run.count; j++, k++)
        {
            const float p = done.prob[k];
            const uint32_t idx = run.frame_point + j;
            r.classified += (p >= 0.f);
            const bool flagged = p >= thr;
            r.flagged += flagged;
            if (cfg.mode == Mode::Drop)
            {
                drop[idx] = flagged;
            }
            else if (o_art >= 0)
            {
                std::memcpy(
                    frame.data.data() + static_cast<size_t>(idx) * fstep +
                        o_art,
                    &p,
                    sizeof(p));
            }
        }
    }
    if (cfg.mode == Mode::Drop && r.flagged)
    {
        uint8_t* d = frame.data.data();
        uint32_t w = 0;
        for (uint32_t i = 0; i < frame.points; i++)
        {
            if (drop[i])
            {
                continue;
            }
            if (w != i)
            {
                std::memcpy(
                    d + static_cast<size_t>(w) * fstep,
                    d + static_cast<size_t>(i) * fstep,
                    fstep);
            }
            w++;
        }
        frame.points = w;
        frame.data.resize(static_cast<size_t>(w) * fstep);
    }
    r.filtered = true;
    return r;
}


// --- GhostFilterWorker -------------------------------------------------------

GhostFilterWorker::GhostFilterWorker(
    std::unique_ptr<GhostFilter> filter,
    PointLayout frame_layout,
    Publish publish_fn,
    uint32_t backlog) :
    ghost{std::move(filter)},
    layout{std::move(frame_layout)},
    publish{std::move(publish_fn)},
    max_backlog{std::max<uint32_t>(backlog, 1)},
    thread{&GhostFilterWorker::run, this}
{
}

GhostFilterWorker::~GhostFilterWorker()
{
    {
        std::lock_guard lock{mtx};
        stopping = true;
    }
    cv.notify_all();
    thread.join();
}

void GhostFilterWorker::push(Job&& job)
{
    {
        std::lock_guard lock{mtx};
        if (job.kind == Job::Kind::Frame)
        {
            queued_frames++;
        }
        jobs.push_back(std::move(job));
    }
    cv.notify_one();
}

void GhostFilterWorker::segment(
    const compact::ScanInfo& info,
    std::span<const uint8_t> datagram,
    uint64_t frame_number,
    uint32_t first_point)
{
    Job j;
    j.kind = Job::Kind::Segment;
    j.datagram.assign(datagram.begin(), datagram.end());
    j.frame_number = frame_number;
    j.first_point = first_point;
    j.points = info.points;
    push(std::move(j));
}

void GhostFilterWorker::frame(FrameAssembler::Frame&& f)
{
    Job j;
    j.kind = Job::Kind::Frame;
    j.frame = std::move(f);
    j.queued = std::chrono::steady_clock::now();
    push(std::move(j));
}

void GhostFilterWorker::reset()
{
    Job j;
    j.kind = Job::Kind::Reset;
    push(std::move(j));
}

void GhostFilterWorker::drain()
{
    std::unique_lock lock{mtx};
    idle_cv.wait(lock, [this] { return (jobs.empty() && !busy) || stopping; });
}

GhostFilterWorker::Stats GhostFilterWorker::takeStats()
{
    std::lock_guard lock{mtx};
    Stats s = stats;
    stats.window_frames = 0;
    stats.window_delay_sum_ns = stats.window_delay_max_ns = 0;
    stats.window_work_sum_ns = stats.window_work_max_ns = 0;
    return s;
}

void GhostFilterWorker::run()
{
    using Clock = std::chrono::steady_clock;
    const auto ns = [](Clock::duration d)
    {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(d).count());
    };
    for (;;)
    {
        Job job;
        bool skip = false;
        {
            std::unique_lock lock{mtx};
            cv.wait(lock, [this] { return !jobs.empty() || stopping; });
            if (stopping)
            {
                break;
            }
            job = std::move(jobs.front());
            jobs.pop_front();
            if (job.kind == Job::Kind::Frame)
            {
                queued_frames--;
            }
            skip = queued_frames >= max_backlog;
            busy = true;
        }

        const Clock::time_point t0 = Clock::now();
        switch (job.kind)
        {
            case Job::Kind::Segment:
                try
                {
                    ghost->addSegment(
                        job.datagram,
                        job.frame_number,
                        job.first_point,
                        job.points,
                        skip);
                }
                catch (const std::exception&)
                {
                    ghost->reset();
                }
                work_ns += ns(Clock::now() - t0);
                break;
            case Job::Kind::Frame:
            {
                GhostFilter::Result res;
                try
                {
                    res = ghost->finishFrame(job.frame, layout);
                }
                catch (const std::exception&)
                {
                    ghost->reset();
                    res = {};
                }
                const Clock::time_point t1 = Clock::now();
                const uint64_t work = work_ns + ns(t1 - t0);
                const uint64_t delay = ns(t1 - job.queued);
                work_ns = 0;
                publish(job.frame, res);
                std::lock_guard lock{mtx};
                if (res.filtered)
                {
                    stats.frames_filtered++;
                    stats.points_classified += res.classified;
                    stats.points_flagged += res.flagged;
                    stats.window_frames++;
                    stats.window_delay_sum_ns += delay;
                    stats.window_delay_max_ns =
                        std::max(stats.window_delay_max_ns, delay);
                    stats.window_work_sum_ns += work;
                    stats.window_work_max_ns =
                        std::max(stats.window_work_max_ns, work);
                }
                else
                {
                    stats.frames_unfiltered++;
                    stats.frames_overloaded += res.skipped;
                }
                break;
            }
            case Job::Kind::Reset:
                ghost->reset();
                work_ns = 0;
                break;
        }

        std::lock_guard lock{mtx};
        busy = false;
        if (jobs.empty())
        {
            idle_cv.notify_all();
        }
    }
    std::lock_guard lock{mtx};
    busy = false;
    idle_cv.notify_all();
}

}  // namespace multiscan
