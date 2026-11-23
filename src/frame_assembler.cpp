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
    compact::ScanDecoder& dec,
    Config config,
    TimeConverter to_host_time) :
    decoder{dec},
    cfg{config},
    to_host{std::move(to_host_time)},
    step{dec.layout().pointStep()}
{
    cfg.segments_per_frame =
        std::clamp<uint32_t>(cfg.segments_per_frame, 1, 32);
    cfg.learn_frames = std::max<uint32_t>(cfg.learn_frames, 1);
    all_mask = (cfg.segments_per_frame == 32)
                   ? 0xFFFFFFFFu
                   : ((1u << cfg.segments_per_frame) - 1);
    expected_mask = all_mask;
    segment_points_hint = std::max<uint32_t>(cfg.initial_segment_points, 1);
    frame_points_hint = segment_points_hint * cfg.segments_per_frame;
}

void FrameAssembler::reset()
{
    active = false;
    used_points = 0;
    seen_mask = 0;
    have_last_emitted = false;
    have_counter = false;
    expected_mask = all_mask;
    partial_mask = 0;
    partial_streak = 0;
}

void FrameAssembler::ensureTail(size_t points)
{
    const size_t need = (static_cast<size_t>(used_points) + points) * step;
    if (buffer.size() < need)
    {
        buffer.resize(
            std::max(need, static_cast<size_t>(frame_points_hint) * step));
    }
}

uint8_t* FrameAssembler::tail(size_t& capacity_points)
{
    ensureTail(segment_points_hint);
    capacity_points = buffer.size() / step - used_points;
    return buffer.data() + static_cast<size_t>(used_points) * step;
}

void FrameAssembler::startFrame(uint64_t number)
{
    active = true;
    joined_mid_frame = false;
    frame_number = number;
    seen_mask = 0;
    used_points = 0;
    stamp_ns = std::numeric_limits<uint64_t>::max();
    time_min_us = std::numeric_limits<uint64_t>::max();
    time_max_us = 0;
}

void FrameAssembler::emit(bool complete, std::vector<Frame>& out)
{
    Frame f;
    buffer.resize(static_cast<size_t>(used_points) * step);
    f.data = std::move(buffer);
    buffer = {};
    f.points = used_points;
    f.frame_number = frame_number;
    f.segment_mask = seen_mask;
    f.expected_mask = expected_mask;
    f.complete = complete;
    f.stamp_ns = stamp_ns;
    f.time_min_us = time_min_us;
    f.time_max_us = time_max_us;
    out.push_back(std::move(f));

    if (complete)
    {
        counters.frames_complete++;
        frame_points_hint =
            used_points;  // size the next frame's buffer exactly
        partial_streak = 0;
    }
    else
    {
        counters.frames_partial++;
    }
    have_last_emitted = true;
    last_emitted_frame = frame_number;
    active = false;
    used_points = 0;
}

void FrameAssembler::closeFrame(bool learn, std::vector<Frame>& out)
{
    if (joined_mid_frame)
    {
        // the revolution the driver started in the middle of: not a loss
        have_last_emitted = true;
        last_emitted_frame = frame_number;
        active = false;
        used_points = 0;
        return;
    }
    if (learn)
    {
        if (seen_mask == partial_mask)
        {
            partial_streak++;
        }
        else
        {
            partial_mask = seen_mask;
            partial_streak = 1;
        }
        if (partial_streak >= cfg.learn_frames && seen_mask != 0)
        {
            expected_mask = seen_mask;
            counters.expected_mask_changes++;
            partial_streak = 0;
        }
    }
    if (cfg.publish_partial_frames)
    {
        emit(false, out);
    }
    else
    {
        counters.frames_discarded++;
        have_last_emitted = true;
        last_emitted_frame = frame_number;
        active = false;
        used_points = 0;
    }
}

void FrameAssembler::flushStale(uint64_t now_ns, std::vector<Frame>& out)
{
    if (active && now_ns > last_segment_ns + cfg.stale_frame_ns)
    {
        closeFrame(false, out);
    }
}

compact::Error FrameAssembler::add(
    std::span<const uint8_t> datagram,
    uint64_t recv_ns,
    std::vector<Frame>& out)
{
    // Decode first, into the free space after the current frame's points:
    // only validated header fields decide where the segment belongs.
    size_t capacity = 0;
    uint8_t* dst = this->tail(capacity);
    compact::ScanInfo info;
    compact::Error err = decoder.decode(datagram, dst, capacity, info);
    if (err == compact::Error::BufferTooSmall)
    {
        segment_points_hint = info.points;
        dst = this->tail(capacity);
        err = decoder.decode(datagram, dst, capacity, info);
    }
    if (err != compact::Error::None)
    {
        counters.decode_errors++;
        counters.last_decode_error = err;
        return err;
    }
    // the segment shape only changes on reconfiguration (then the retry above grows it)
    segment_points_hint = info.points;

    // last_counter is the highest counter seen; a lower one arriving later
    // (reordered datagram) fills one of the gaps counted before
    const uint64_t counter = info.telegram_counter;
    if (!have_counter || counter > last_counter)
    {
        if (have_counter)
        {
            counters.telegrams_lost += counter - last_counter - 1;
        }
        last_counter = counter;
        have_counter = true;
    }
    else if (counter + 1000 < last_counter)
    {
        last_counter = counter;  // sensor restarted
    }
    else if (counter < last_counter && counters.telegrams_lost > 0)
    {
        counters.telegrams_lost--;
    }

    if (info.segment >= cfg.segments_per_frame)
    {
        counters.segments_out_of_range++;
        return compact::Error::None;
    }
    const uint32_t bit = 1u << info.segment;
    const uint64_t f = info.frame;

    // frame numbers restart from 0 when the sensor reboots
    const bool have_ref = active || have_last_emitted;
    const uint64_t ref = active ? frame_number : last_emitted_frame;
    if (have_ref && f + 2 < ref)
    {
        counters.sensor_restarts++;
        if (active)
        {
            // keep the just-decoded points: move them to the buffer start
            std::memmove(
                buffer.data(),
                dst,
                static_cast<size_t>(info.points) * step);
            dst = buffer.data();
        }
        reset();
    }

    if (active && f == frame_number)
    {
        if (seen_mask & bit)
        {
            counters.segments_duplicate++;
            return compact::Error::None;
        }
    }
    else if (
        (active && f < frame_number) ||
        (!active && have_last_emitted && f <= last_emitted_frame))
    {
        counters.segments_late++;
        return compact::Error::None;
    }
    else if (active)  // f > frame_number: the current frame ended incomplete
    {
        const std::vector<uint8_t> pending(
            dst,
            dst + static_cast<size_t>(info.points) * step);
        counters.frames_missed += f - frame_number - 1;
        closeFrame(true, out);
        startFrame(f);
        ensureTail(info.points);
        std::memcpy(buffer.data(), pending.data(), pending.size());
    }
    else  // no frame open; the segment already sits at the buffer start
    {
        if (have_last_emitted && f > last_emitted_frame + 1)
        {
            counters.frames_missed += f - last_emitted_frame - 1;
        }
        const bool first_since_reset = !have_last_emitted;
        startFrame(f);
        joined_mid_frame =
            first_since_reset &&
            (info.segment !=
             static_cast<uint64_t>(std::countr_zero(expected_mask)));
    }

    seen_mask |= bit;
    used_points += info.points;
    stamp_ns = std::min(stamp_ns, to_host(info.start_time_us, recv_ns));
    time_min_us = std::min(time_min_us, info.time_min_us);
    time_max_us = std::max(time_max_us, info.time_max_us);
    last_segment_ns = recv_ns;
    counters.segments++;

    if (!(expected_mask & bit))
    {
        expected_mask |= bit;
        counters.expected_mask_changes++;
        partial_streak = 0;
    }
    if ((seen_mask & expected_mask) == expected_mask)
    {
        emit(true, out);
    }
    return compact::Error::None;
}

}  // namespace multiscan
