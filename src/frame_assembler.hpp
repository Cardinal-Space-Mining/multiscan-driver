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

/*  Groups decoded scan segments into frames (one sensor revolution) by the
 *  frame number every Compact telegram carries.
 *
 *  - Segments are decoded straight into the frame's point buffer in arrival
 *    order. A finished frame hands that buffer over (move, no copy).
 *  - A frame is emitted as soon as every *expected* segment has arrived. The
 *    expected set starts as all segments and follows what the sensor actually
 *    sends: if frames keep ending with the same segments missing (a narrowed
 *    output range omits whole segments), those segments stop being expected;
 *    a segment outside the set arriving adds it back.
 *  - When a frame ends incomplete (the next frame starts, or the stream goes
 *    quiet) it is emitted as partial, or discarded if configured. The frame
 *    the driver joins mid-revolution at startup is discarded silently.
 *  - Duplicate and late segments are dropped; gaps in the telegram counter
 *    are counted as lost telegrams; frame numbers jumping backwards (sensor
 *    restart) reset the state.
 *  - Times are moved to the host clock: a callback gives the host - sensor
 *    offset for each telegram, which is added to the frame stamp and to
 *    every point's time field.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "compact/decoder.hpp"


namespace multiscan
{

class FrameAssembler
{
public:
    struct Config
    {
        uint32_t segments_per_frame = 12;
        /* Emit frames that end with expected segments missing. */
        bool publish_partial_frames = true;
        /* Consecutive frames missing the same segments before those
         * segments stop being expected. */
        uint32_t learn_frames = 3;
        /* A frame with no new segment for this long is closed. */
        uint64_t stale_frame_ns = 100'000'000;
        /* Room per segment until the first one is decoded; afterwards the
         * last segment's and last complete frame's sizes are used. */
        uint32_t initial_segment_points = 1024;
    };

    struct Frame
    {
        std::vector<uint8_t> data;  // points * point_step bytes
        uint32_t points = 0;
        uint64_t frame_number = 0;
        uint32_t segment_mask = 0;   // segments contained
        uint32_t expected_mask = 0;  // segments expected when emitted
        bool complete = false;
        uint64_t stamp_ns = 0;     // earliest segment start (host clock)
        uint64_t time_min_us = 0;  // earliest/latest point time (host clock)
        uint64_t time_max_us = 0;
    };

    struct Stats
    {
        uint64_t frames_complete = 0;
        uint64_t frames_partial = 0;  // emitted incomplete
        uint64_t frames_discarded =
            0;  // incomplete and not emitted (excluding startup)
        uint64_t frames_missed = 0;  // frame numbers never seen
        uint64_t segments = 0;       // accepted
        uint64_t segments_duplicate = 0;
        uint64_t segments_late = 0;
        uint64_t segments_out_of_range = 0;
        uint64_t telegrams_lost =
            0;  // telegram counter gaps not filled by late arrivals
        uint64_t decode_errors = 0;
        uint64_t sensor_restarts = 0;
        uint64_t expected_mask_changes = 0;
        compact::Error last_decode_error = compact::Error::None;
    };

    /* host - sensor clock offset (ns) for a decoded telegram that arrived at
     * recv_ns. Called for every telegram that decodes, so it can also feed a
     * clock estimator. */
    using ClockOffset =
        std::function<int64_t(const compact::ScanInfo& info, uint64_t recv_ns)>;

    FrameAssembler(
        compact::ScanDecoder& decoder,
        Config config,
        ClockOffset clock_offset);

    /* Decode and assemble one scan telegram. Frames that complete or end
     * because of it are appended to `out`. */
    compact::Error add(
        std::span<const uint8_t> datagram,
        uint64_t recv_ns,
        std::vector<Frame>& out);

    /* Close the current frame if it received nothing for stale_frame_ns. */
    void flushStale(uint64_t now_ns, std::vector<Frame>& out);

    /* Forget all state, e.g. after reconnecting to the sensor. */
    void reset();

    const Stats& stats() const { return counters; }
    uint32_t expectedMask() const { return expected_mask; }

private:
    void startFrame(uint64_t frame_number);
    void closeFrame(bool learn, std::vector<Frame>& out);
    void emit(bool complete, std::vector<Frame>& out);
    uint8_t* tail(size_t& capacity_points);
    void ensureTail(size_t points);

    compact::ScanDecoder& decoder;
    Config cfg;
    ClockOffset clock_offset;
    uint32_t step;
    int32_t time_field;  // byte offset of the point time, or -1
    uint32_t all_mask;

    // current frame
    bool active = false;
    bool joined_mid_frame = false;
    uint64_t frame_number = 0;
    uint32_t seen_mask = 0;
    uint32_t used_points = 0;
    uint64_t stamp_ns = 0, time_min_us = 0, time_max_us = 0;
    uint64_t last_segment_ns = 0;
    std::vector<uint8_t> buffer;

    // history
    bool have_last_emitted = false;
    uint64_t last_emitted_frame = 0;
    bool have_counter = false;
    uint64_t last_counter = 0;
    uint32_t expected_mask;
    uint32_t partial_mask = 0, partial_streak = 0;
    uint32_t frame_points_hint;
    uint32_t segment_points_hint;

    Stats counters;
};

}  // namespace multiscan
