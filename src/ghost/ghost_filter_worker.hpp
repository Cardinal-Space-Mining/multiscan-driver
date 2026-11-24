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

/*  Runs a GhostFilter on its own thread: the receive thread queues segments
 *  and frames, the worker filters them and hands every frame, filtered or
 *  not, to a callback (which publishes it).
 *
 *  When the worker falls behind, new segments are skipped and their frames
 *  published unfiltered, so a slow machine can't fall further and further
 *  behind.
 */
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "ghost/ghost_filter.hpp"


namespace multiscan
{

class GhostFilterWorker
{
public:
    struct Stats
    {
        // --- Totals ---
        uint64_t frames_filtered{0};
        uint64_t frames_unfiltered{0};  // published without filtering
        uint64_t frames_overloaded{0};  // of those: the worker fell behind
        uint64_t points_classified{0};
        uint64_t points_flagged{0};

        // --- Since the last takeStats() ---
        uint64_t window_frames{0};
        /* From the assembler emitting a frame to its publication: what the
         * filter adds to the latency. */
        uint64_t window_delay_sum_ns{0};
        uint64_t window_delay_max_ns{0};
        /* The filter's work on a frame, all its segments included. */
        uint64_t window_work_sum_ns{0};
        uint64_t window_work_max_ns{0};
    };

    using Publish = std::function<
        void(FrameAssembler::Frame& frame, const GhostFilter::Result& result)>;

public:
    /* While max_backlog or more frames wait, new segments are skipped. */
    GhostFilterWorker(
        std::unique_ptr<GhostFilter> filter,
        PointLayout frame_layout,
        Publish publish,
        uint32_t max_backlog = 2);
    ~GhostFilterWorker();

    GhostFilterWorker(const GhostFilterWorker&) = delete;
    GhostFilterWorker& operator=(const GhostFilterWorker&) = delete;

public:
    // --- From the receive thread, in the order the assembler produces them --
    void segment(
        const compact::ScanInfo& info,
        std::span<const uint8_t> datagram,
        uint64_t frame_number,
        uint32_t first_point);
    void frame(FrameAssembler::Frame&& frame);
    void reset();

    /* Blocks until everything queued so far has been published. */
    void drain();

    /* Totals, and the window since the last call (which this resets). */
    Stats takeStats();

    const GhostFilter& filter() const { return *this->ghost_filter; }

protected:
    using Clock = std::chrono::steady_clock;

    struct Job
    {
        enum class Kind
        {
            Segment,
            Frame,
            Reset
        };

        Kind kind{Kind::Segment};
        // Kind::Segment
        std::vector<uint8_t> datagram;
        uint64_t frame_number{0};
        uint32_t first_point{0};
        uint32_t num_points{0};
        // Kind::Frame
        FrameAssembler::Frame frame;
        Clock::time_point queued_time;
    };

protected:
    void push(Job&& job);
    void run();
    bool waitForJob(Job& job, bool& skip_segments);
    void runSegment(const Job& job, bool skip);
    void runFrame(Job& job);
    void recordFrame(
        const GhostFilter::Result& result,
        uint64_t delay_ns,
        uint64_t work_ns);
    void markIdle();

protected:
    std::unique_ptr<GhostFilter> ghost_filter;
    PointLayout frame_layout;
    Publish publish;
    uint32_t max_backlog;

    // --- Guarded by queue_mutex ----------------------------------------------
    std::mutex queue_mutex;
    std::condition_variable job_available;
    std::condition_variable became_idle;
    std::deque<Job> jobs;
    uint32_t queued_frames{0};
    bool busy{false};
    bool stopping{false};
    Stats stats;

    // --- Worker thread only --------------------------------------------------
    uint64_t open_frame_work_ns{0};  // work on the frame being fed
    std::thread worker_thread;
};

}  // namespace multiscan
