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

#include "frame_assembler.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <limits>


namespace multiscan
{

FrameAssembler::FrameAssembler(
    compact::ScanDecoder& decoder,
    Config config,
    ClockOffset clock_offset) :
    decoder{decoder},
    config{config},
    clock_offset{std::move(clock_offset)},
    point_step{decoder.layout().pointStep()},
    time_offset{decoder.layout().offset(Field::Time)}
{
    this->config.segments_per_frame =
        std::clamp<uint32_t>(this->config.segments_per_frame, 1, 32);
    this->config.learn_frames =
        std::max<uint32_t>(this->config.learn_frames, 1);
    this->all_segments_mask =
        (this->config.segments_per_frame == 32)
            ? 0xFFFFFFFFu
            : ((1u << this->config.segments_per_frame) - 1);
    this->expected_mask = this->all_segments_mask;
    this->segment_points_hint =
        std::max<uint32_t>(this->config.initial_segment_points, 1);
    this->frame_points_hint =
        this->segment_points_hint * this->config.segments_per_frame;
}

void FrameAssembler::reset()
{
    this->frame_open = false;
    this->frame_points = 0;
    this->received_mask = 0;
    this->has_last_closed = false;
    this->has_telegram_counter = false;
    this->expected_mask = this->all_segments_mask;
    this->incomplete_mask = 0;
    this->incomplete_streak = 0;
}

void FrameAssembler::flushStale(uint64_t now_ns, std::vector<Frame>& out)
{
    if (this->frame_open &&
        now_ns > this->last_segment_time_ns + this->config.stale_frame_ns)
    {
        this->closeFrame(false, out);
    }
}

compact::Error FrameAssembler::add(
    std::span<const uint8_t> datagram,
    uint64_t receive_time_ns,
    std::vector<Frame>& out)
{
    // Decode first, into the free space after the current frame's points:
    // only validated header fields decide where the segment belongs.
    compact::ScanInfo info;
    uint8_t* decoded = nullptr;
    const compact::Error error = this->decodeAtTail(datagram, info, decoded);
    if (error != compact::Error::None)
    {
        this->statistics.decode_errors++;
        this->statistics.last_decode_error = error;
        return error;
    }

    const SegmentTimes times =
        this->moveToHostClock(info, receive_time_ns, decoded);
    this->countLostTelegrams(info.telegram_counter);

    if (info.segment_index >= this->config.segments_per_frame)
    {
        this->statistics.segments_out_of_range++;
        return compact::Error::None;
    }
    if (this->placeSegment(info, decoded, out))
    {
        this->acceptSegment(info, times, receive_time_ns, datagram, out);
    }
    return compact::Error::None;
}



// --- Steps of add() ----------------------------------------------------------

compact::Error FrameAssembler::decodeAtTail(
    std::span<const uint8_t> datagram,
    compact::ScanInfo& info,
    uint8_t*& decoded)
{
    size_t capacity = 0;
    decoded = this->tail(capacity);
    compact::Error error =
        this->decoder.decode(datagram, decoded, capacity, info);
    if (error == compact::Error::BufferTooSmall)
    {
        this->segment_points_hint = info.num_points;
        decoded = this->tail(capacity);
        error = this->decoder.decode(datagram, decoded, capacity, info);
    }
    if (error == compact::Error::None)
    {
        // the segment shape only changes on reconfiguration (then the retry
        // above grows the buffer)
        this->segment_points_hint = info.num_points;
    }
    return error;
}

/* Adds the host - sensor offset to every point time (whole microseconds)
 * and returns the segment's times on the host clock. */
FrameAssembler::SegmentTimes FrameAssembler::moveToHostClock(
    const compact::ScanInfo& info,
    uint64_t receive_time_ns,
    uint8_t* decoded)
{
    const int64_t offset_ns = this->clock_offset(info, receive_time_ns);
    const int64_t offset_us = offset_ns >= 0 ? (offset_ns + 500) / 1000
                                             : -((-offset_ns + 500) / 1000);
    if (this->time_offset >= 0 && offset_us != 0)
    {
        uint8_t* time_field = decoded + this->time_offset;
        for (uint32_t i = 0; i < info.num_points;
             i++, time_field += this->point_step)
        {
            uint64_t time_us;
            std::memcpy(&time_us, time_field, sizeof(time_us));
            time_us += static_cast<uint64_t>(offset_us);
            std::memcpy(time_field, &time_us, sizeof(time_us));
        }
    }
    return SegmentTimes{
        .stamp_ns = static_cast<uint64_t>(
            static_cast<int64_t>(info.start_time_us * 1000) + offset_ns),
        .time_min_us = info.time_min_us + static_cast<uint64_t>(offset_us),
        .time_max_us = info.time_max_us + static_cast<uint64_t>(offset_us)};
}

/* highest_telegram_counter is the highest counter seen; a lower one arriving
 * later (reordered datagram) fills one of the gaps counted before. */
void FrameAssembler::countLostTelegrams(uint64_t telegram_counter)
{
    if (!this->has_telegram_counter ||
        telegram_counter > this->highest_telegram_counter)
    {
        if (this->has_telegram_counter)
        {
            this->statistics.telegrams_lost +=
                telegram_counter - this->highest_telegram_counter - 1;
        }
        this->highest_telegram_counter = telegram_counter;
        this->has_telegram_counter = true;
    }
    else if (telegram_counter + 1000 < this->highest_telegram_counter)
    {
        this->highest_telegram_counter = telegram_counter;  // sensor restarted
    }
    else if (
        telegram_counter < this->highest_telegram_counter &&
        this->statistics.telegrams_lost > 0)
    {
        this->statistics.telegrams_lost--;
    }
}

/* Decides which frame a decoded segment belongs to: the current one, a new
 * one (closing the current one incomplete), or none (duplicate or late).
 * Afterwards the segment's points sit right after the frame's points.
 * Returns false when the segment is dropped. */
bool FrameAssembler::placeSegment(
    const compact::ScanInfo& info,
    uint8_t*& decoded,
    std::vector<Frame>& out)
{
    const uint32_t segment_bit = 1u << info.segment_index;
    const uint64_t number = info.frame_number;
    const size_t segment_bytes =
        static_cast<size_t>(info.num_points) * this->point_step;

    // frame numbers restart from 0 when the sensor reboots
    const bool has_reference = this->frame_open || this->has_last_closed;
    const uint64_t reference =
        this->frame_open ? this->frame_number : this->last_closed_frame;
    if (has_reference && number + 2 < reference)
    {
        this->statistics.sensor_restarts++;
        if (this->frame_open)
        {
            // keep the just-decoded points: move them to the buffer start
            std::memmove(this->frame_buffer.data(), decoded, segment_bytes);
            decoded = this->frame_buffer.data();
        }
        this->reset();
    }

    if (this->frame_open && number == this->frame_number)
    {
        if (this->received_mask & segment_bit)
        {
            this->statistics.segments_duplicate++;
            return false;
        }
        return true;
    }
    if ((this->frame_open && number < this->frame_number) ||
        (!this->frame_open && this->has_last_closed &&
         number <= this->last_closed_frame))
    {
        this->statistics.segments_late++;
        return false;
    }

    if (this->frame_open)
    {
        // a later frame: the current one ended incomplete
        const std::vector<uint8_t> pending(decoded, decoded + segment_bytes);
        this->statistics.frames_missed += number - this->frame_number - 1;
        this->closeFrame(true, out);
        this->startFrame(number);
        this->ensureTail(info.num_points);
        std::memcpy(this->frame_buffer.data(), pending.data(), pending.size());
        decoded = this->frame_buffer.data();
        return true;
    }

    // no frame open; the segment already sits at the buffer start
    if (this->has_last_closed && number > this->last_closed_frame + 1)
    {
        this->statistics.frames_missed += number - this->last_closed_frame - 1;
    }
    const bool first_since_reset = !this->has_last_closed;
    this->startFrame(number);
    this->joined_mid_frame =
        first_since_reset &&
        (info.segment_index !=
         static_cast<uint64_t>(std::countr_zero(this->expected_mask)));
    return true;
}

void FrameAssembler::acceptSegment(
    const compact::ScanInfo& info,
    const SegmentTimes& times,
    uint64_t receive_time_ns,
    std::span<const uint8_t> datagram,
    std::vector<Frame>& out)
{
    const uint32_t segment_bit = 1u << info.segment_index;
    const uint32_t first_point = this->frame_points;
    this->received_mask |= segment_bit;
    this->frame_points += info.num_points;
    this->frame_stamp_ns = std::min(this->frame_stamp_ns, times.stamp_ns);
    this->frame_time_min_us =
        std::min(this->frame_time_min_us, times.time_min_us);
    this->frame_time_max_us =
        std::max(this->frame_time_max_us, times.time_max_us);
    this->last_segment_time_ns = receive_time_ns;
    this->statistics.segments++;

    if (!(this->expected_mask & segment_bit))
    {
        this->expected_mask |= segment_bit;
        this->statistics.expected_mask_changes++;
        this->incomplete_streak = 0;
    }
    // the frame joined mid-revolution is never emitted
    if (this->segment_hook && !this->joined_mid_frame)
    {
        this->segment_hook(info, datagram, this->frame_number, first_point);
    }
    if ((this->received_mask & this->expected_mask) == this->expected_mask)
    {
        this->emit(true, out);
    }
}



// --- Frames ------------------------------------------------------------------

void FrameAssembler::startFrame(uint64_t frame_number)
{
    this->frame_open = true;
    this->joined_mid_frame = false;
    this->frame_number = frame_number;
    this->received_mask = 0;
    this->frame_points = 0;
    this->frame_stamp_ns = std::numeric_limits<uint64_t>::max();
    this->frame_time_min_us = std::numeric_limits<uint64_t>::max();
    this->frame_time_max_us = 0;
}

void FrameAssembler::closeFrame(bool learn, std::vector<Frame>& out)
{
    if (this->joined_mid_frame)
    {
        // the revolution the driver started in the middle of: not a loss
        this->forgetFrame();
        return;
    }
    if (learn)
    {
        this->learnExpectedSegments();
    }
    if (this->config.publish_partial_frames)
    {
        this->emit(false, out);
    }
    else
    {
        this->statistics.frames_discarded++;
        this->forgetFrame();
    }
}

void FrameAssembler::emit(bool complete, std::vector<Frame>& out)
{
    Frame frame;
    this->frame_buffer.resize(
        static_cast<size_t>(this->frame_points) * this->point_step);
    frame.data = std::move(this->frame_buffer);
    this->frame_buffer = {};
    frame.num_points = this->frame_points;
    frame.frame_number = this->frame_number;
    frame.segment_mask = this->received_mask;
    frame.expected_mask = this->expected_mask;
    frame.complete = complete;
    frame.stamp_ns = this->frame_stamp_ns;
    frame.time_min_us = this->frame_time_min_us;
    frame.time_max_us = this->frame_time_max_us;
    out.push_back(std::move(frame));

    if (complete)
    {
        this->statistics.frames_complete++;
        // size the next frame's buffer exactly
        this->frame_points_hint = this->frame_points;
        this->incomplete_streak = 0;
    }
    else
    {
        this->statistics.frames_partial++;
    }
    this->forgetFrame();
}

/* Closes the current frame without emitting it. */
void FrameAssembler::forgetFrame()
{
    this->has_last_closed = true;
    this->last_closed_frame = this->frame_number;
    this->frame_open = false;
    this->frame_points = 0;
}

/* Frames that keep ending with the same segments missing: those segments
 * aren't sent (a narrowed output range omits whole segments). */
void FrameAssembler::learnExpectedSegments()
{
    if (this->received_mask == this->incomplete_mask)
    {
        this->incomplete_streak++;
    }
    else
    {
        this->incomplete_mask = this->received_mask;
        this->incomplete_streak = 1;
    }
    if (this->incomplete_streak >= this->config.learn_frames &&
        this->received_mask != 0)
    {
        this->expected_mask = this->received_mask;
        this->statistics.expected_mask_changes++;
        this->incomplete_streak = 0;
    }
}



// --- Buffer ------------------------------------------------------------------

void FrameAssembler::ensureTail(size_t num_points)
{
    const size_t needed =
        (static_cast<size_t>(this->frame_points) + num_points) *
        this->point_step;
    if (this->frame_buffer.size() < needed)
    {
        this->frame_buffer.resize(
            std::max(
                needed,
                static_cast<size_t>(this->frame_points_hint) *
                    this->point_step));
    }
}

/* The free space after the current frame's points, with room for at least
 * one segment of the last size seen. */
uint8_t* FrameAssembler::tail(size_t& capacity_points)
{
    this->ensureTail(this->segment_points_hint);
    capacity_points =
        this->frame_buffer.size() / this->point_step - this->frame_points;
    return this->frame_buffer.data() +
           static_cast<size_t>(this->frame_points) * this->point_step;
}

}  // namespace multiscan
