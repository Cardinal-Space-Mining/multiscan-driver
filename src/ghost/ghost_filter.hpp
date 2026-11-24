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

/*  Ghost point (artifact) filter: the classifier trained in ghost-ml-ws, run
 *  on every frame as its segments arrive.
 *
 *  - The classifier itself is include/multiscan_driver/artifact_filter/
 *    (shared with ghost-ml-ws, which builds its tools against it).
 *  - Each accepted segment is decoded again into the fields the classifier
 *    needs (same decoder, so the points are the same and in the same order
 *    as in the published frame) and handed to it. A grid cell is classified
 *    as soon as every cell its inputs look at has arrived, so when the last
 *    segment of a frame arrives only the cells next to it are left (about a
 *    fifth on the multiScan: its upper and lower seven layers are sent half
 *    a revolution apart). The result is identical to classifying the whole
 *    frame at once.
 *  - The high-resolution layers (ids 5 and 13) are not part of the model's
 *    grid: their points are not classified and never dropped.
 *  - GhostFilter is synchronous; GhostFilterWorker (ghost_filter_worker.hpp)
 *    runs it on its own thread.
 */
#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include <multiscan_driver/artifact_filter/filter.hpp>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"


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
        Mode mode{Mode::Drop};
        /* Probability from which a point is an artifact; < 0: the model's. */
        double threshold{-1.};
        /* Stop evaluating trees for clearly clean returns (see
         * artifact_filter::Model::predict). */
        bool early_exit{true};
        /* As the published decoder's (DecodeOptions::echo_selection). */
        uint32_t echo_selection{1};
    };

    struct Result
    {
        bool filtered{false};  // false: the frame is published as it is
        /* Not filtered because the worker skipped segments (overloaded). */
        bool skipped{false};
        uint32_t points_classified{0};
        uint32_t points_flagged{0};  // dropped, in Mode::Drop
    };

public:
    /* Throws std::runtime_error if the model doesn't fit this sensor. */
    GhostFilter(artifact_filter::Model model, Config config);

public:
    /* A segment the assembler accepted (its SegmentHook). With skip, the
     * segment is not looked at and its frame is left unfiltered. */
    void addSegment(
        std::span<const uint8_t> datagram,
        uint64_t frame_number,
        uint32_t first_point,
        uint32_t num_points,
        bool skip = false);

    /* Applies the result to a frame the assembler emitted. `layout` is the
     * frame's; Mode::Annotate needs its artifact field. */
    Result finishFrame(FrameAssembler::Frame& frame, const PointLayout& layout);

    /* Forget the earlier scans; the frame in progress is left unfiltered. */
    void reset();

public:
    const artifact_filter::Model& model() const { return this->filter.model(); }
    bool earlyExitActive() const { return this->filter.earlyExitActive(); }
    double threshold() const { return this->flag_threshold; }
    const Config& config() const { return this->filter_config; }

protected:
    /* Consecutive points of a frame handed to the classifier. */
    struct PointRun
    {
        uint32_t first_frame_point;
        uint32_t num_points;
    };

    struct FrameState
    {
        bool valid{false};
        bool broken{false};   // skipped or inconsistent: not filtered
        bool skipped{false};  // broken because segments were skipped
        bool begun{false};    // the classifier has begun this frame
        uint64_t frame_number{0};
        std::vector<PointRun> runs;
        std::vector<float> probability;  // per point handed to the classifier
    };

    /* Byte offsets in the classifier's own point layout. */
    struct FieldOffsets
    {
        int32_t intensity;
        int32_t azimuth;
        int32_t elevation;
        int32_t layer;
    };

protected:
    // --- Segments ------------------------------------------------------------
    void startFrame(uint64_t frame_number);
    bool decodeSegment(std::span<const uint8_t> datagram, uint32_t num_points);
    void addLowResolutionPoints(uint32_t first_point, uint32_t num_points);
    void coverSegmentBeams(size_t first_run, uint32_t first_point);
    void finishOpenFrame();

    // --- Frames --------------------------------------------------------------
    bool runsFitFrame(
        const FrameState& state,
        const FrameAssembler::Frame& frame) const;
    void annotate(
        const FrameState& state,
        FrameAssembler::Frame& frame,
        const PointLayout& layout,
        Result& result) const;
    void drop(
        const FrameState& state,
        FrameAssembler::Frame& frame,
        const PointLayout& layout,
        Result& result) const;

protected:
    artifact_filter::Filter filter;
    Config filter_config;
    double flag_threshold;

    compact::ScanDecoder segment_decoder;
    FieldOffsets field_offsets;
    uint32_t point_step;
    std::vector<uint8_t> segment_points;

    bool frame_open{false};
    FrameState current_frame;   // being fed
    FrameState finished_frame;  // finished, waiting for the assembler's frame
};

}  // namespace multiscan
