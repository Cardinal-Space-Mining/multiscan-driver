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

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>


namespace multiscan
{

namespace
{

constexpr double RAD_TO_DEG = 180. / 3.14159265358979323846;

/* The 0.125 degree layers, which aren't part of the model's grid. */
constexpr uint32_t HIGH_RESOLUTION_LAYER_A = 5;
constexpr uint32_t HIGH_RESOLUTION_LAYER_B = 13;

/* What the multiScan's models are made for: 14 low-resolution layers, 1
 * degree columns, 20 Hz. */
constexpr int SENSOR_RINGS = 14;
constexpr int SENSOR_COLUMNS = 360;
constexpr double SENSOR_SCAN_RATE_HZ = 20.;

PointLayout classifierLayout()
{
    std::string error;
    return *PointLayout::fromNames(
        {"x", "y", "z", "intensity", "azimuth", "elevation", "layer"},
        error);
}

template<typename T> T loadField(const uint8_t* point, int32_t offset)
{
    T value;
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}

bool isHighResolutionLayer(uint32_t layer)
{
    return layer == HIGH_RESOLUTION_LAYER_A || layer == HIGH_RESOLUTION_LAYER_B;
}

}  // namespace



// --- Setup -------------------------------------------------------------------

GhostFilter::GhostFilter(artifact_filter::Model model, Config config) :
    filter{std::move(model)},
    filter_config{config},
    flag_threshold{
        config.threshold >= 0. ? config.threshold
                               : this->filter.model().threshold},
    segment_decoder{
        classifierLayout(),
        compact::DecodeOptions{.echo_selection = config.echo_selection}}
{
    const artifact_filter::Model& loaded = this->filter.model();
    const bool rate_matches =
        loaded.scan_rate_hz <= 0. ||
        std::abs(loaded.scan_rate_hz - SENSOR_SCAN_RATE_HZ) <= 0.5;
    if (loaded.num_rings != SENSOR_RINGS ||
        loaded.num_columns != SENSOR_COLUMNS || !rate_matches)
    {
        throw std::runtime_error(
            "the model was made for another sensor (" +
            std::to_string(loaded.num_rings) + " rings, " +
            std::to_string(loaded.num_columns) + " columns, " +
            std::to_string(loaded.scan_rate_hz) +
            " Hz; the multiScan has 14, 360, 20)");
    }
    this->filter.setEarlyExit(config.early_exit);

    const PointLayout& layout = this->segment_decoder.layout();
    this->field_offsets = FieldOffsets{
        .intensity = layout.offset(Field::Intensity),
        .azimuth = layout.offset(Field::Azimuth),
        .elevation = layout.offset(Field::Elevation),
        .layer = layout.offset(Field::Layer)};
    this->point_step = layout.pointStep();
}

void GhostFilter::reset()
{
    // the rest of a frame in progress is left unfiltered
    const bool was_open = this->frame_open;
    const uint64_t frame_number = this->current_frame.frame_number;
    this->current_frame = {};
    this->current_frame.valid = this->current_frame.broken = was_open;
    this->current_frame.frame_number = frame_number;
    this->finished_frame = {};
    this->filter.reset();
}



// --- Segments ----------------------------------------------------------------

void GhostFilter::addSegment(
    std::span<const uint8_t> datagram,
    uint64_t frame_number,
    uint32_t first_point,
    uint32_t num_points,
    bool skip)
{
    if (!this->frame_open || frame_number != this->current_frame.frame_number)
    {
        this->startFrame(frame_number);
    }
    if (this->current_frame.broken)
    {
        return;
    }
    if (skip)
    {
        this->current_frame.broken = true;
        this->current_frame.skipped = true;
        return;
    }
    if (!this->decodeSegment(datagram, num_points))
    {
        this->current_frame.broken = true;  // not the points the frame holds
        return;
    }

    const size_t first_run = this->current_frame.runs.size();
    this->addLowResolutionPoints(first_point, num_points);
    // every beam of the segment, with or without a return, is complete now
    this->coverSegmentBeams(first_run, first_point);
    this->filter.classifyReady();
}

/* A new frame: the one before is complete (its frame, if the assembler
 * emits it, follows). */
void GhostFilter::startFrame(uint64_t frame_number)
{
    if (this->frame_open)
    {
        this->finishOpenFrame();
    }
    this->current_frame = {};
    this->current_frame.valid = true;
    this->current_frame.frame_number = frame_number;
    this->frame_open = true;
}

/* Decodes the segment into the classifier's layout, and begins the
 * classifier's scan on the frame's first segment. */
bool GhostFilter::decodeSegment(
    std::span<const uint8_t> datagram,
    uint32_t num_points)
{
    const size_t needed_bytes =
        static_cast<size_t>(num_points) * this->point_step;
    if (this->segment_points.size() < needed_bytes)
    {
        this->segment_points.resize(needed_bytes);
    }
    compact::ScanInfo info;
    const compact::Error error = this->segment_decoder.decode(
        datagram,
        this->segment_points.data(),
        this->segment_points.size() / this->point_step,
        info);
    if (error != compact::Error::None || info.num_points != num_points)
    {
        return false;
    }
    if (!this->current_frame.begun)
    {
        this->filter.begin(static_cast<int64_t>(info.start_time_us) * 1000);
        this->current_frame.begun = true;
    }
    return true;
}

/* Hands the segment's points to the classifier in runs of consecutive
 * low-resolution points (the decoder writes every line's beams together). */
void GhostFilter::addLowResolutionPoints(
    uint32_t first_point,
    uint32_t num_points)
{
    const uint8_t* points = this->segment_points.data();
    const auto layerOf = [&](uint32_t point)
    {
        return loadField<uint32_t>(
            points + static_cast<size_t>(point) * this->point_step,
            this->field_offsets.layer);
    };

    uint32_t point = 0;
    while (point < num_points)
    {
        while (point < num_points && isHighResolutionLayer(layerOf(point)))
        {
            point++;
        }
        const uint32_t run_start = point;
        while (point < num_points && !isHighResolutionLayer(layerOf(point)))
        {
            point++;
        }
        if (point == run_start)
        {
            continue;
        }
        const uint8_t* run =
            points + static_cast<size_t>(run_start) * this->point_step;
        this->filter.add(
            point - run_start,
            reinterpret_cast<const float*>(run),
            this->point_step,
            reinterpret_cast<const float*>(run + this->field_offsets.intensity),
            this->point_step);
        this->current_frame.runs.push_back(
            {first_point + run_start, point - run_start});
    }
}

/* Covers the cell of every beam added for this segment (the runs from
 * first_run on). */
void GhostFilter::coverSegmentBeams(size_t first_run, uint32_t first_point)
{
    const std::vector<PointRun>& runs = this->current_frame.runs;
    for (size_t run = first_run; run < runs.size(); run++)
    {
        const uint8_t* point =
            this->segment_points.data() +
            static_cast<size_t>(runs[run].first_frame_point - first_point) *
                this->point_step;
        for (uint32_t i = 0; i < runs[run].num_points;
             i++, point += this->point_step)
        {
            const float azimuth =
                loadField<float>(point, this->field_offsets.azimuth);
            const float elevation =
                loadField<float>(point, this->field_offsets.elevation);
            this->filter.cover(this->filter.cellAt(
                azimuth * RAD_TO_DEG,
                elevation * RAD_TO_DEG));
        }
    }
}

void GhostFilter::finishOpenFrame()
{
    if (this->current_frame.broken || !this->current_frame.begun)
    {
        // what was added is incomplete: don't compare the next frame with it
        this->filter.reset();
        this->current_frame.probability.clear();
        this->current_frame.broken = true;
    }
    else
    {
        this->filter.finish(
            this->current_frame.probability,
            this->flag_threshold);
    }
    this->finished_frame = std::move(this->current_frame);
    this->current_frame = {};
    this->frame_open = false;
}



// --- Frames ------------------------------------------------------------------

GhostFilter::Result GhostFilter::finishFrame(
    FrameAssembler::Frame& frame,
    const PointLayout& layout)
{
    if (this->frame_open &&
        this->current_frame.frame_number == frame.frame_number)
    {
        this->finishOpenFrame();
    }
    Result result;
    if (!this->finished_frame.valid ||
        this->finished_frame.frame_number != frame.frame_number)
    {
        return result;  // the filter never saw this frame
    }
    const FrameState state = std::move(this->finished_frame);
    this->finished_frame = {};
    result.skipped = state.skipped;
    if (state.broken || !this->runsFitFrame(state, frame) ||
        frame.data.size() <
            static_cast<size_t>(frame.num_points) * layout.pointStep())
    {
        return result;
    }

    if (this->filter_config.mode == Mode::Drop)
    {
        this->drop(state, frame, layout, result);
    }
    else
    {
        this->annotate(state, frame, layout, result);
    }
    result.filtered = true;
    return result;
}

bool GhostFilter::runsFitFrame(
    const FrameState& state,
    const FrameAssembler::Frame& frame) const
{
    for (const PointRun& run : state.runs)
    {
        if (static_cast<uint64_t>(run.first_frame_point) + run.num_points >
            frame.num_points)
        {
            return false;
        }
    }
    return true;
}

/* Writes every classified point's probability to the artifact field. */
void GhostFilter::annotate(
    const FrameState& state,
    FrameAssembler::Frame& frame,
    const PointLayout& layout,
    Result& result) const
{
    const uint32_t frame_step = layout.pointStep();
    const int32_t artifact_offset = layout.offset(Field::Artifact);
    size_t probability_index = 0;
    for (const PointRun& run : state.runs)
    {
        for (uint32_t i = 0; i < run.num_points; i++, probability_index++)
        {
            const float probability = state.probability[probability_index];
            result.points_classified += (probability >= 0.f);
            result.points_flagged += (probability >= this->flag_threshold);
            if (artifact_offset >= 0)
            {
                const size_t frame_point = run.first_frame_point + i;
                std::memcpy(
                    frame.data.data() + frame_point * frame_step +
                        artifact_offset,
                    &probability,
                    sizeof(probability));
            }
        }
    }
}

/* Removes the flagged points, keeping the order of the others. */
void GhostFilter::drop(
    const FrameState& state,
    FrameAssembler::Frame& frame,
    const PointLayout& layout,
    Result& result) const
{
    std::vector<uint8_t> flagged(frame.num_points, 0);
    size_t probability_index = 0;
    for (const PointRun& run : state.runs)
    {
        for (uint32_t i = 0; i < run.num_points; i++, probability_index++)
        {
            const float probability = state.probability[probability_index];
            const bool is_artifact = probability >= this->flag_threshold;
            result.points_classified += (probability >= 0.f);
            result.points_flagged += is_artifact;
            flagged[run.first_frame_point + i] = is_artifact;
        }
    }
    if (!result.points_flagged)
    {
        return;
    }

    const uint32_t frame_step = layout.pointStep();
    uint8_t* data = frame.data.data();
    uint32_t kept = 0;
    for (uint32_t point = 0; point < frame.num_points; point++)
    {
        if (flagged[point])
        {
            continue;
        }
        if (kept != point)
        {
            std::memcpy(
                data + static_cast<size_t>(kept) * frame_step,
                data + static_cast<size_t>(point) * frame_step,
                frame_step);
        }
        kept++;
    }
    frame.num_points = kept;
    frame.data.resize(static_cast<size_t>(kept) * frame_step);
}

}  // namespace multiscan
