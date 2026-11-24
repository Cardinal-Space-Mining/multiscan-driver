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

/*  Ghost point (artifact) filter: the classifier trained in ghost-ml-ws
 *  (annotator_app), run on every frame as its segments arrive.
 *
 *  - artifact_filter/artifact_filter.hpp is a copy of ghost-ml-ws
 *    src/ghost_tester/include/artifact_filter/artifact_filter.hpp; keep it
 *    identical (it mirrors the Python feature code, see its header).
 *  - Each accepted segment is decoded again into the fields the filter needs
 *    (same decoder, so the points are the same and in the same order as in
 *    the published frame) and handed to the filter. A grid cell is classified
 *    as soon as every cell its inputs look at has arrived, so when the last
 *    segment of a frame arrives only the cells next to it are left (about a
 *    fifth on the multiScan: its upper and lower seven layers are sent half a
 *    revolution apart). The result is identical to classifying the whole
 *    frame at once.
 *  - The high-resolution layers (ids 5 and 13) are not part of the model's
 *    grid: their points are not classified and never dropped.
 *  - GhostFilter is synchronous; GhostFilterWorker runs it on its own thread
 *    and hands finished frames to a callback.
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
#include <string>
#include <thread>
#include <vector>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "ghost/artifact_filter/artifact_filter.hpp"


namespace multiscan
{

class GhostFilter
{
public:
    enum class Mode
    {
        Drop,      // remove flagged points from the frame
        Annotate,  // write every point's probability to the artifact field
    };

    struct Config
    {
        Mode mode = Mode::Drop;
        /* probability from which a point is an artifact; < 0: the model's */
        double threshold = -1.;
        /* stop evaluating trees for clearly clean returns (see the header) */
        bool early_exit = true;
        /* as the published decoder's (DecodeOptions::echo_selection) */
        uint32_t echo_selection = 1;
    };

    struct Result
    {
        bool filtered = false;  // false: the frame is published as it is
        uint32_t classified = 0;
        uint32_t flagged = 0;  // dropped, in Mode::Drop
        bool skipped = false;  // not filtered because segments were skipped
    };

    /* Throws std::runtime_error if the model doesn't fit this sensor. */
    GhostFilter(artifact_filter::Model model, Config config);

    /* A segment the assembler accepted (its SegmentHook). With skip, the
     * segment is not looked at and its frame is left unfiltered. */
    void addSegment(
        std::span<const uint8_t> datagram,
        uint64_t frame_number,
        uint32_t first_point,
        uint32_t points,
        bool skip = false);

    /* Applies the result to a frame the assembler emitted. `layout` is the
     * frame's; Mode::Annotate needs its artifact field. */
    Result finishFrame(FrameAssembler::Frame& frame, const PointLayout& layout);

    /* Forget the earlier scans; the frame in progress is left unfiltered. */
    void reset();

    const artifact_filter::Model& model() const { return filter.model(); }
    bool early_exit_active() const { return filter.early_exit(); }
    double threshold() const { return thr; }
    const Config& config() const { return cfg; }

private:
    struct Run  // consecutive points handed to the filter
    {
        uint32_t frame_point;  // index in the frame
        uint32_t count;
    };
    struct FrameState
    {
        bool valid = false;
        bool broken = false;   // skipped or inconsistent: not filtered
        bool skipped = false;  // broken because segments were skipped
        bool begun = false;    // the filter has begun this frame
        uint64_t number = 0;
        std::vector<Run> runs;
        std::vector<float> prob;  // per point handed to the filter
    };

    void finishOpen();

    artifact_filter::Filter filter;
    Config cfg;
    double thr;
    compact::ScanDecoder decoder;
    int32_t o_int, o_az, o_el, o_layer;
    uint32_t step;
    std::vector<uint8_t> buffer;

    bool open = false;
    FrameState current;   // being fed
    FrameState finished;  // finished, waiting for the assembler's frame
};


class GhostFilterWorker
{
public:
    struct Stats
    {
        uint64_t frames_filtered = 0;
        uint64_t frames_unfiltered = 0;  // published without filtering
        uint64_t frames_overloaded = 0;  // of those: the worker fell behind
        uint64_t points_classified = 0;
        uint64_t points_flagged = 0;
        /* Since the last takeStats(): frames, and the time from the
         * assembler emitting a frame to its publication (what the filter
         * adds to the latency) and the filter's work per frame. */
        uint64_t window_frames = 0;
        uint64_t window_delay_sum_ns = 0, window_delay_max_ns = 0;
        uint64_t window_work_sum_ns = 0, window_work_max_ns = 0;
    };

    using Publish = std::function<
        void(FrameAssembler::Frame& frame, const GhostFilter::Result& result)>;

    /* While more than max_backlog frames wait, new segments are skipped and
     * their frames published unfiltered, so a slow machine can't fall
     * further and further behind. */
    GhostFilterWorker(
        std::unique_ptr<GhostFilter> filter,
        PointLayout layout,
        Publish publish,
        uint32_t max_backlog = 2);
    ~GhostFilterWorker();

    GhostFilterWorker(const GhostFilterWorker&) = delete;
    GhostFilterWorker& operator=(const GhostFilterWorker&) = delete;

    /* From the receive thread, in the order the assembler produces them. */
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

    const GhostFilter& filter() const { return *ghost; }

private:
    struct Job
    {
        enum class Kind
        {
            Segment,
            Frame,
            Reset
        } kind = Kind::Segment;
        std::vector<uint8_t> datagram;
        uint64_t frame_number = 0;
        uint32_t first_point = 0, points = 0;
        FrameAssembler::Frame frame;
        std::chrono::steady_clock::time_point queued;
    };

    void run();
    void push(Job&& job);

    std::unique_ptr<GhostFilter> ghost;
    PointLayout layout;
    Publish publish;
    uint32_t max_backlog;

    std::mutex mtx;
    std::condition_variable cv, idle_cv;
    std::deque<Job> jobs;        // guarded by mtx
    uint32_t queued_frames = 0;  // guarded by mtx
    bool busy = false;           // guarded by mtx
    bool stopping = false;       // guarded by mtx
    Stats stats;                 // guarded by mtx
    uint64_t work_ns = 0;        // worker thread: work on the open frame
    std::thread thread;
};

}  // namespace multiscan
