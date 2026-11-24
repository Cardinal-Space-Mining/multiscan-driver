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

#include "ghost/ghost_filter_worker.hpp"

#include <algorithm>
#include <exception>


namespace multiscan
{

namespace
{
uint64_t toNanoseconds(std::chrono::steady_clock::duration duration)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
}
}  // namespace



// --- Setup -------------------------------------------------------------------

GhostFilterWorker::GhostFilterWorker(
    std::unique_ptr<GhostFilter> filter,
    PointLayout frame_layout,
    Publish publish,
    uint32_t max_backlog) :
    ghost_filter{std::move(filter)},
    frame_layout{std::move(frame_layout)},
    publish{std::move(publish)},
    max_backlog{std::max<uint32_t>(max_backlog, 1)},
    worker_thread{&GhostFilterWorker::run, this}
{
}

GhostFilterWorker::~GhostFilterWorker()
{
    {
        std::lock_guard lock{this->queue_mutex};
        this->stopping = true;
    }
    this->job_available.notify_all();
    this->worker_thread.join();
}



// --- Receive thread ----------------------------------------------------------

void GhostFilterWorker::segment(
    const compact::ScanInfo& info,
    std::span<const uint8_t> datagram,
    uint64_t frame_number,
    uint32_t first_point)
{
    Job job;
    job.kind = Job::Kind::Segment;
    job.datagram.assign(datagram.begin(), datagram.end());
    job.frame_number = frame_number;
    job.first_point = first_point;
    job.num_points = info.num_points;
    this->push(std::move(job));
}

void GhostFilterWorker::frame(FrameAssembler::Frame&& frame)
{
    Job job;
    job.kind = Job::Kind::Frame;
    job.frame = std::move(frame);
    job.queued_time = Clock::now();
    this->push(std::move(job));
}

void GhostFilterWorker::reset()
{
    Job job;
    job.kind = Job::Kind::Reset;
    this->push(std::move(job));
}

void GhostFilterWorker::drain()
{
    std::unique_lock lock{this->queue_mutex};
    this->became_idle.wait(
        lock,
        [this]
        { return (this->jobs.empty() && !this->busy) || this->stopping; });
}

GhostFilterWorker::Stats GhostFilterWorker::takeStats()
{
    std::lock_guard lock{this->queue_mutex};
    const Stats taken = this->stats;
    this->stats.window_frames = 0;
    this->stats.window_delay_sum_ns = 0;
    this->stats.window_delay_max_ns = 0;
    this->stats.window_work_sum_ns = 0;
    this->stats.window_work_max_ns = 0;
    return taken;
}

void GhostFilterWorker::push(Job&& job)
{
    {
        std::lock_guard lock{this->queue_mutex};
        if (job.kind == Job::Kind::Frame)
        {
            this->queued_frames++;
        }
        this->jobs.push_back(std::move(job));
    }
    this->job_available.notify_one();
}



// --- Worker thread -----------------------------------------------------------

void GhostFilterWorker::run()
{
    Job job;
    bool skip_segments = false;
    while (this->waitForJob(job, skip_segments))
    {
        switch (job.kind)
        {
            case Job::Kind::Segment:
                this->runSegment(job, skip_segments);
                break;
            case Job::Kind::Frame:
                this->runFrame(job);
                break;
            case Job::Kind::Reset:
                this->ghost_filter->reset();
                this->open_frame_work_ns = 0;
                break;
        }
        this->markIdle();
    }
    this->markIdle();
}

/* Takes the next job; false when stopping. skip_segments tells whether the
 * worker is too far behind to filter. */
bool GhostFilterWorker::waitForJob(Job& job, bool& skip_segments)
{
    std::unique_lock lock{this->queue_mutex};
    this->job_available.wait(
        lock,
        [this] { return !this->jobs.empty() || this->stopping; });
    if (this->stopping)
    {
        return false;
    }
    job = std::move(this->jobs.front());
    this->jobs.pop_front();
    if (job.kind == Job::Kind::Frame)
    {
        this->queued_frames--;
    }
    skip_segments = this->queued_frames >= this->max_backlog;
    this->busy = true;
    return true;
}

void GhostFilterWorker::runSegment(const Job& job, bool skip)
{
    const Clock::time_point start = Clock::now();
    try
    {
        this->ghost_filter->addSegment(
            job.datagram,
            job.frame_number,
            job.first_point,
            job.num_points,
            skip);
    }
    catch (const std::exception&)
    {
        this->ghost_filter->reset();
    }
    this->open_frame_work_ns += toNanoseconds(Clock::now() - start);
}

void GhostFilterWorker::runFrame(Job& job)
{
    const Clock::time_point start = Clock::now();
    GhostFilter::Result result;
    try
    {
        result = this->ghost_filter->finishFrame(job.frame, this->frame_layout);
    }
    catch (const std::exception&)
    {
        this->ghost_filter->reset();
        result = {};
    }
    const Clock::time_point done = Clock::now();
    const uint64_t work_ns =
        this->open_frame_work_ns + toNanoseconds(done - start);
    const uint64_t delay_ns = toNanoseconds(done - job.queued_time);
    this->open_frame_work_ns = 0;

    this->publish(job.frame, result);
    this->recordFrame(result, delay_ns, work_ns);
}

void GhostFilterWorker::recordFrame(
    const GhostFilter::Result& result,
    uint64_t delay_ns,
    uint64_t work_ns)
{
    std::lock_guard lock{this->queue_mutex};
    Stats& totals = this->stats;
    if (!result.filtered)
    {
        totals.frames_unfiltered++;
        totals.frames_overloaded += result.skipped;
        return;
    }
    totals.frames_filtered++;
    totals.points_classified += result.points_classified;
    totals.points_flagged += result.points_flagged;
    totals.window_frames++;
    totals.window_delay_sum_ns += delay_ns;
    totals.window_delay_max_ns = std::max(totals.window_delay_max_ns, delay_ns);
    totals.window_work_sum_ns += work_ns;
    totals.window_work_max_ns = std::max(totals.window_work_max_ns, work_ns);
}

void GhostFilterWorker::markIdle()
{
    std::lock_guard lock{this->queue_mutex};
    this->busy = false;
    if (this->jobs.empty() || this->stopping)
    {
        this->became_idle.notify_all();
    }
}

}  // namespace multiscan
