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
        uint32_t segments_per_frame{12};
        /* Emit frames that end with expected segments missing. */
        bool publish_partial_frames{true};
        /* Consecutive frames missing the same segments before those
         * segments stop being expected. */
        uint32_t learn_frames{3};
        /* A frame with no new segment for this long is closed. */
        uint64_t stale_frame_ns{100'000'000};
        /* Room per segment until the first one is decoded; afterwards the
         * last segment's and last complete frame's sizes are used. */
        uint32_t initial_segment_points{1024};
    };

    struct Frame
    {
        std::vector<uint8_t> data;  // num_points * point_step bytes
        uint32_t num_points{0};
        uint64_t frame_number{0};
        uint32_t segment_mask{0};   // segments contained
        uint32_t expected_mask{0};  // segments expected when emitted
        bool complete{false};
        uint64_t stamp_ns{0};  // earliest segment start (host clock)
        /* Earliest and latest point time (host clock). */
        uint64_t time_min_us{0};
        uint64_t time_max_us{0};
    };

    struct Stats
    {
        uint64_t frames_complete{0};
        uint64_t frames_partial{0};  // emitted incomplete
        /* Incomplete and not emitted (excluding startup). */
        uint64_t frames_discarded{0};
        uint64_t frames_missed{0};  // frame numbers never seen
        uint64_t segments{0};       // accepted
        uint64_t segments_duplicate{0};
        uint64_t segments_late{0};
        uint64_t segments_out_of_range{0};
        /* Telegram counter gaps not filled by late arrivals. */
        uint64_t telegrams_lost{0};
        uint64_t decode_errors{0};
        uint64_t sensor_restarts{0};
        uint64_t expected_mask_changes{0};
        compact::Error last_decode_error{compact::Error::None};
    };

    /* host - sensor clock offset (ns) for a decoded telegram that arrived at
     * receive_time_ns. Called for every telegram that decodes, so it can
     * also feed a clock estimator. */
    using ClockOffset = std::function<
        int64_t(const compact::ScanInfo& info, uint64_t receive_time_ns)>;

    /* Called for every segment that is accepted into a frame, once it has
     * been decoded there and before the frame can be emitted: the telegram,
     * the frame number and where the segment's points start in the frame.
     * Not called for the frame joined mid-revolution, which is never
     * emitted. */
    using SegmentHook = std::function<void(
        const compact::ScanInfo& info,
        std::span<const uint8_t> datagram,
        uint64_t frame_number,
        uint32_t first_point)>;

public:
    FrameAssembler(
        compact::ScanDecoder& decoder,
        Config config,
        ClockOffset clock_offset);

public:
    /* Decode and assemble one scan telegram. Frames that complete or end
     * because of it are appended to `out`. */
    compact::Error add(
        std::span<const uint8_t> datagram,
        uint64_t receive_time_ns,
        std::vector<Frame>& out);

    /* Close the current frame if it received nothing for stale_frame_ns. */
    void flushStale(uint64_t now_ns, std::vector<Frame>& out);

    void setSegmentHook(SegmentHook hook)
    {
        this->segment_hook = std::move(hook);
    }

    /* Forget all state, e.g. after reconnecting to the sensor. */
    void reset();

    const Stats& stats() const { return this->statistics; }
    uint32_t expectedMask() const { return this->expected_mask; }

protected:
    /* A decoded segment's times on the host clock. */
    struct SegmentTimes
    {
        uint64_t stamp_ns;
        uint64_t time_min_us;
        uint64_t time_max_us;
    };

protected:
    // --- Steps of add() ------------------------------------------------------
    compact::Error decodeAtTail(
        std::span<const uint8_t> datagram,
        compact::ScanInfo& info,
        uint8_t*& decoded);
    SegmentTimes moveToHostClock(
        const compact::ScanInfo& info,
        uint64_t receive_time_ns,
        uint8_t* decoded);
    void countLostTelegrams(uint64_t telegram_counter);
    bool placeSegment(
        const compact::ScanInfo& info,
        uint8_t*& decoded,
        std::vector<Frame>& out);
    void acceptSegment(
        const compact::ScanInfo& info,
        const SegmentTimes& times,
        uint64_t receive_time_ns,
        std::span<const uint8_t> datagram,
        std::vector<Frame>& out);

    // --- Frames --------------------------------------------------------------
    void startFrame(uint64_t frame_number);
    void closeFrame(bool learn, std::vector<Frame>& out);
    void emit(bool complete, std::vector<Frame>& out);
    void forgetFrame();
    void learnExpectedSegments();

    // --- Buffer --------------------------------------------------------------
    uint8_t* tail(size_t& capacity_points);
    void ensureTail(size_t num_points);

protected:
    compact::ScanDecoder& decoder;
    Config config;
    ClockOffset clock_offset;
    SegmentHook segment_hook;
    uint32_t point_step;
    int32_t time_offset;  // byte offset of the point time, or -1
    uint32_t all_segments_mask;

    // --- The current frame ---------------------------------------------------
    bool frame_open{false};
    bool joined_mid_frame{false};
    uint64_t frame_number{0};
    uint32_t received_mask{0};
    uint32_t frame_points{0};
    uint64_t frame_stamp_ns{0};
    uint64_t frame_time_min_us{0};
    uint64_t frame_time_max_us{0};
    uint64_t last_segment_time_ns{0};
    std::vector<uint8_t> frame_buffer;

    // --- History -------------------------------------------------------------
    bool has_last_closed{false};
    uint64_t last_closed_frame{0};
    bool has_telegram_counter{false};
    uint64_t highest_telegram_counter{0};
    uint32_t expected_mask;
    /* The segments recent frames ended with, and how many in a row. */
    uint32_t incomplete_mask{0};
    uint32_t incomplete_streak{0};
    uint32_t frame_points_hint;
    uint32_t segment_points_hint;

    Stats statistics;
};

}  // namespace multiscan
