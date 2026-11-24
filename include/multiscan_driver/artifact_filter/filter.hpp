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

/*  Artifact (ghost point) filter: runs the classifier trained in ghost-ml-ws
 *  on live scans.
 *
 *      artifact_filter::Filter filter{artifact_filter::Model::load(path)};
 *      ...for every scan, in order...
 *      filter.process(xyz, num_points, intensity_or_null, stamp_ns, prob);
 *      // prob[i] >= filter.model().threshold  ->  point i is an artifact
 *
 *  A scan can also be fed in pieces as the lidar sends it (begin, add, cover,
 *  classifyReady, finish; see Filter::begin): cells are classified as soon
 *  as their inputs are complete, so little work is left when the scan ends.
 *
 *  This file mirrors ghost-ml-ws annotator_app/scan_grid.py (points -> grid
 *  cells) and annotator_app/artifact_features.py (grid -> features). The two
 *  must stay in step: after changing either, run ghost-ml-ws
 *  artifact_filter_check against a check file from export_model.py.
 *
 *  Header only, C++ standard library only (OpenMP is used when the compiler
 *  has it), no ROS.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "model.hpp"


namespace artifact_filter
{

class Filter
{
public:
    explicit Filter(Model model);

public:
    const Model& model() const { return this->artifact_model; }

    /* Forget the earlier scans (after a gap in the stream this happens by
     * itself: scans too far apart in time are not compared). */
    void reset();

    /* Stop evaluating the trees early for returns that are clearly clean
     * (see Model::predict). Several times faster; flags practically the
     * same points; probabilities under the threshold are no longer exact.
     * Does nothing when the model file carries no checkpoints. */
    void setEarlyExit(bool enabled) { this->early_exit_enabled = enabled; }
    bool earlyExitActive() const
    {
        return this->early_exit_enabled && this->artifact_model.hasEarlyExit();
    }

    /* For checking against the Python: keep the classifier inputs of every
     * scan. lastFeatures() is then num_cells * num_features values, NaN for
     * cells with nothing to classify. Off by default (it costs time). */
    void setKeepFeatures(bool enabled) { this->keep_features = enabled; }
    const std::vector<float>& lastFeatures() const
    {
        return this->kept_features;
    }

    /* Probability per grid cell (ring * num_columns + column) of the last
     * scan; 0 where there was nothing to classify. */
    const std::vector<float>& cellProbabilities() const
    {
        return this->cell_probability;
    }

public:
    /* Classifies one scan. Scans must be given in the order they were taken.
     *
     *  xyz          num_points * 3 floats, in the lidar's own frame
     *  intensity    num_points raw intensity values, or nullptr. Required
     *               when model().use_intensity.
     *  stamp_ns     the scan's time stamp
     *  point_probability  out, num_points values: probability that the point
     *               is an artifact, or -1 when it was not classified
     *               (invalid, nearer than the minimum range, or not the
     *               nearest return of its beam in a multi-echo scan)
     *
     * Returns the number of points with probability >= threshold (threshold
     * < 0: the model's own). */
    size_t process(
        const float* xyz,
        size_t num_points,
        const float* intensity,
        int64_t stamp_ns,
        std::vector<float>& point_probability,
        double threshold = -1.);

public:
    // --- The same scan fed in pieces, as a lidar sends it --------------------
    //
    //     filter.begin(stamp_ns);
    //     for every piece (e.g. a network packet):
    //         filter.add(n, xyz, xyz_stride, intensity, intensity_stride);
    //         for every beam the piece carries, return or not:
    //             filter.cover(filter.cellAt(azimuth_deg, elevation_deg));
    //         filter.classifyReady();
    //     filter.finish(point_probability);
    //
    // A cell is classified as soon as every cell its inputs look at is
    // covered, so most of the work is done before the last piece arrives.
    // The result is identical to process() on all points in the same order:
    // cover() only tells the filter that no more returns will come for a
    // cell. Covering a cell that still gets returns later breaks that; cells
    // never covered are handled in finish().

    /* Starts a scan. */
    void begin(int64_t stamp_ns);

    /* Adds num_points points of the scan. xyz: x y z floats of the first
     * point, the next one xyz_stride bytes further; intensity likewise
     * (nullptr: none, which only a model without intensity accepts). Points
     * keep their order across calls: finish() reports them in the order
     * they were added. */
    void add(
        size_t num_points,
        const float* xyz,
        size_t xyz_stride,
        const float* intensity,
        size_t intensity_stride);

    /* Grid cell (ring * num_columns + column) of a direction in the lidar's
     * frame: the nearest layer by elevation, the column by azimuth. */
    int32_t cellAt(double azimuth_deg, double elevation_deg) const;

    /* No more returns will be added for this cell (see above). */
    void cover(int32_t cell);

    /* Classifies the cells whose inputs are complete. Returns how many. */
    size_t classifyReady();

    /* Ends the scan: classifies what is left and reports every point added
     * since begin() (see process()). Returns the number of points with
     * probability >= threshold (threshold < 0: the model's own). */
    size_t finish(
        std::vector<float>& point_probability,
        double threshold = -1.);

protected:
    struct Scan
    {
        std::vector<float> log_range;  // per cell, NaN: nothing there
        int64_t stamp_ns{0};
        bool valid{false};
    };

protected:
    static constexpr double RAD_TO_DEG = 180. / 3.14159265358979323846;
    static constexpr float NOT_A_NUMBER =
        std::numeric_limits<float>::quiet_NaN();

    static long wrap(long value, long modulus)
    {
        return ((value % modulus) + modulus) % modulus;
    }

protected:
    // --- Scan bookkeeping ----------------------------------------------------
    void resetCells(int64_t stamp_ns);
    void selectEarlierScans(int64_t stamp_ns);
    void finalizeCell(size_t cell);
    void classifyCells(const std::vector<int32_t>& cells);
    size_t reportPoints(std::vector<float>& point_probability, double threshold)
        const;

    // --- Features ------------------------------------------------------------
    size_t cellIndex(int ring, int column) const;
    float cellValue(const std::vector<float>& grid, int ring, int column) const;
    float clipped(float difference) const;
    float countAround(const std::vector<uint8_t>& grid, int ring, int column)
        const;

    void computeFeatures(int ring, int column, float* features) const;
    float* addNeighbourFeatures(int ring, int column, float* out) const;
    float* addEarlierScanFeatures(int ring, int column, float* out) const;
    float* addIntensityFeatures(int ring, int column, float* out) const;

protected:
    Model artifact_model;
    size_t num_cells;
    int history_depth{0};  // earlier scans needed
    int min_range_mm{200};
    // how far a cell's inputs reach (and so which cells wait for it)
    int neighbourhood_rings{2};
    int neighbourhood_columns{4};
    bool early_exit_enabled{false};
    bool keep_features{false};

    // --- The scan in progress, and the earlier ones --------------------------
    Scan current_scan;
    std::vector<Scan> earlier_scans;  // [0] = one scan back
    /* Per time offset: the earlier scan's log ranges, or null when it is
     * missing or too far apart in time. */
    std::vector<const std::vector<float>*> compared_log_range;
    /* Per time offset (only -1 is filled), as artifact_features.py names
     * them: unstable cells are present in both scans with another range,
     * gone cells are present now and absent then. */
    std::vector<std::vector<uint8_t>> unstable_cells;
    std::vector<std::vector<uint8_t>> gone_cells;

    // --- Per point -----------------------------------------------------------
    std::vector<int32_t> point_cell;  // -1: invalid
    std::vector<double> point_range_m;

    // --- Per cell ------------------------------------------------------------
    std::vector<double> cell_nearest_range_m;
    std::vector<uint16_t> cell_range_mm;
    std::vector<uint16_t> cell_intensity;
    std::vector<float> cell_range_m;  // from cell_range_mm; NaN: nothing there
    std::vector<float> cell_probability;
    std::vector<uint8_t> cell_covered;     // all its returns are in
    std::vector<uint8_t> cell_classified;  // its probability is final
    std::vector<int> cell_inputs_missing;  // uncovered cells it looks at
    std::vector<int32_t> ready_cells;      // complete, not yet classified
    std::vector<float> kept_features;
};



// --- Setup -------------------------------------------------------------------

inline Filter::Filter(Model model) :
    artifact_model{std::move(model)},
    num_cells{
        static_cast<size_t>(this->artifact_model.num_rings) *
        static_cast<size_t>(this->artifact_model.num_columns)}
{
    const Model& settings = this->artifact_model;
    for (const int offset : settings.time_offsets)
    {
        this->history_depth = std::max(this->history_depth, -offset);
    }
    this->earlier_scans.resize(static_cast<size_t>(this->history_depth));
    this->min_range_mm =
        static_cast<int>(std::lround(settings.min_range_m * 1000.));

    // cells whose neighbourhood contains a cell (and the other way round)
    this->neighbourhood_rings = 1;
    for (const int offset : settings.ring_offsets)
    {
        this->neighbourhood_rings =
            std::max(this->neighbourhood_rings, std::abs(offset));
    }
    this->neighbourhood_columns = 2;  // countAround reaches 2 columns
    for (const int offset : settings.column_offsets)
    {
        this->neighbourhood_columns =
            std::max(this->neighbourhood_columns, std::abs(offset));
    }
}

inline void Filter::reset()
{
    for (Scan& scan : this->earlier_scans)
    {
        scan.valid = false;
    }
}

inline size_t Filter::process(
    const float* xyz,
    size_t num_points,
    const float* intensity,
    int64_t stamp_ns,
    std::vector<float>& point_probability,
    double threshold)
{
    this->begin(stamp_ns);
    this->add(num_points, xyz, 3 * sizeof(float), intensity, sizeof(float));
    return this->finish(point_probability, threshold);
}



// --- Streaming ---------------------------------------------------------------

inline void Filter::begin(int64_t stamp_ns)
{
    this->resetCells(stamp_ns);
    this->selectEarlierScans(stamp_ns);
}

inline void Filter::resetCells(int64_t stamp_ns)
{
    const Model& settings = this->artifact_model;
    const size_t cells = this->num_cells;

    this->point_cell.clear();
    this->point_range_m.clear();
    this->cell_nearest_range_m.assign(
        cells,
        std::numeric_limits<double>::infinity());
    this->cell_range_mm.assign(cells, 0);
    this->cell_intensity.assign(settings.use_intensity ? cells : 0, 0);
    this->cell_range_m.assign(cells, NOT_A_NUMBER);
    this->cell_probability.assign(cells, 0.f);
    this->cell_covered.assign(cells, 0);
    this->cell_classified.assign(cells, 0);
    this->ready_cells.clear();

    this->current_scan.log_range.assign(cells, NOT_A_NUMBER);
    this->current_scan.stamp_ns = stamp_ns;
    this->current_scan.valid = true;

    // every cell waits for the cells around it, fewer at the top and bottom
    this->cell_inputs_missing.resize(cells);
    const int rings = settings.num_rings;
    for (int ring = 0; ring < rings; ring++)
    {
        const int rings_around =
            std::min(ring + this->neighbourhood_rings, rings - 1) -
            std::max(ring - this->neighbourhood_rings, 0) + 1;
        std::fill_n(
            this->cell_inputs_missing.begin() +
                static_cast<long>(ring) * settings.num_columns,
            settings.num_columns,
            rings_around * (2 * this->neighbourhood_columns + 1));
    }

    if (this->keep_features)
    {
        this->kept_features.assign(
            cells * static_cast<size_t>(settings.num_features),
            NOT_A_NUMBER);
    }
}

/* Earlier scans that are close enough in time to compare with
 * (earlier_scans[k - 1] is the scan k steps back). */
inline void Filter::selectEarlierScans(int64_t stamp_ns)
{
    const Model& settings = this->artifact_model;
    const size_t num_offsets = settings.time_offsets.size();
    this->compared_log_range.assign(num_offsets, nullptr);
    this->unstable_cells.resize(num_offsets);
    this->gone_cells.resize(num_offsets);

    for (size_t offset_index = 0; offset_index < num_offsets; offset_index++)
    {
        const int scans_back = -settings.time_offsets[offset_index];
        const Scan& earlier =
            this->earlier_scans[static_cast<size_t>(scans_back - 1)];
        const int64_t gap = stamp_ns > earlier.stamp_ns
                                ? stamp_ns - earlier.stamp_ns
                                : earlier.stamp_ns - stamp_ns;
        if (earlier.valid && gap <= scans_back * settings.max_scan_gap_ns)
        {
            this->compared_log_range[offset_index] = &earlier.log_range;
        }
        if (scans_back == 1)
        {
            this->unstable_cells[offset_index].assign(this->num_cells, 0);
            this->gone_cells[offset_index].assign(this->num_cells, 0);
        }
    }
}

inline void Filter::add(
    size_t num_points,
    const float* xyz,
    size_t xyz_stride,
    const float* intensity,
    size_t intensity_stride)
{
    const Model& settings = this->artifact_model;
    if (settings.use_intensity && !intensity && num_points)
    {
        throw std::invalid_argument(
            "artifact_filter: this model needs intensity, the scan has none "
            "(use the range-only model, artifact_xyz)");
    }

    // Points -> grid cells; the nearest return of a cell wins.
    const auto* xyz_bytes = reinterpret_cast<const uint8_t*>(xyz);
    const auto* intensity_bytes = reinterpret_cast<const uint8_t*>(intensity);
    for (size_t i = 0; i < num_points;
         i++, xyz_bytes += xyz_stride, intensity_bytes += intensity_stride)
    {
        float position[3];
        std::memcpy(position, xyz_bytes, sizeof(position));
        const double x = position[0];
        const double y = position[1];
        const double z = position[2];
        const double horizontal = std::hypot(x, y);
        const double range = std::hypot(horizontal, z);
        this->point_range_m.push_back(range);
        if (!std::isfinite(range) || !(range > 1e-4))
        {
            this->point_cell.push_back(-1);
            continue;
        }

        const int32_t cell = this->cellAt(
            std::atan2(y, x) * RAD_TO_DEG,
            std::atan2(z, horizontal) * RAD_TO_DEG);
        this->point_cell.push_back(cell);
        const size_t index = static_cast<size_t>(cell);
        if (range > this->cell_nearest_range_m[index])
        {
            continue;
        }
        this->cell_nearest_range_m[index] = range;
        this->cell_range_mm[index] = static_cast<uint16_t>(
            std::clamp(std::nearbyint(range * 1000.), 0., 65535.));
        if (settings.use_intensity)
        {
            float raw;
            std::memcpy(&raw, intensity_bytes, sizeof(raw));
            double value = raw;
            if (settings.intensity_max != 65535.)
            {
                value =
                    std::nearbyint(value * (65535. / settings.intensity_max));
            }
            this->cell_intensity[index] =
                static_cast<uint16_t>(std::clamp(value, 0., 65535.));
        }
    }
}

inline int32_t Filter::cellAt(double azimuth_deg, double elevation_deg) const
{
    const Model& settings = this->artifact_model;
    const int rings = settings.num_rings;

    int layer = 0;
    double best_distance =
        std::abs(elevation_deg - settings.layer_elevations_deg[0]);
    for (int candidate = 1; candidate < rings; candidate++)
    {
        const double distance =
            std::abs(elevation_deg - settings.layer_elevations_deg[candidate]);
        if (distance < best_distance)
        {
            best_distance = distance;
            layer = candidate;
        }
    }

    // nearbyint: round half to even, as numpy does
    const long beam = static_cast<long>(std::nearbyint(
        (azimuth_deg - settings.layer_first_azimuth_deg[layer]) /
        settings.column_width_deg));
    const int column = static_cast<int>(
        wrap(beam + settings.layer_first_column[layer], settings.num_columns));
    const int ring = (rings - 1) - layer;
    return ring * settings.num_columns + column;
}

inline void Filter::cover(int32_t cell)
{
    if (cell < 0 || static_cast<size_t>(cell) >= this->num_cells ||
        this->cell_covered[static_cast<size_t>(cell)])
    {
        return;
    }
    this->cell_covered[static_cast<size_t>(cell)] = 1;
    this->finalizeCell(static_cast<size_t>(cell));

    // every cell that looks at this one has one input less to wait for
    const int columns = this->artifact_model.num_columns;
    const int ring = cell / columns;
    const int column = cell % columns;
    const int first_ring = std::max(ring - this->neighbourhood_rings, 0);
    const int last_ring = std::min(
        ring + this->neighbourhood_rings,
        this->artifact_model.num_rings - 1);
    for (int other_ring = first_ring; other_ring <= last_ring; other_ring++)
    {
        for (int offset = -this->neighbourhood_columns;
             offset <= this->neighbourhood_columns;
             offset++)
        {
            const size_t other = this->cellIndex(other_ring, column + offset);
            if (--this->cell_inputs_missing[other] == 0 &&
                !std::isnan(this->current_scan.log_range[other]))
            {
                this->ready_cells.push_back(static_cast<int32_t>(other));
            }
        }
    }
}

inline size_t Filter::classifyReady()
{
    const size_t count = this->ready_cells.size();
    this->classifyCells(this->ready_cells);
    this->ready_cells.clear();
    return count;
}

inline size_t Filter::finish(
    std::vector<float>& point_probability,
    double threshold)
{
    if (threshold < 0.)
    {
        threshold = this->artifact_model.threshold;
    }

    // Cells never covered are complete now.
    this->ready_cells.clear();
    for (size_t cell = 0; cell < this->num_cells; cell++)
    {
        if (!this->cell_covered[cell])
        {
            this->cell_covered[cell] = 1;
            this->finalizeCell(cell);
        }
    }
    for (size_t cell = 0; cell < this->num_cells; cell++)
    {
        if (!this->cell_classified[cell] &&
            !std::isnan(this->current_scan.log_range[cell]))
        {
            this->ready_cells.push_back(static_cast<int32_t>(cell));
        }
    }
    this->classifyCells(this->ready_cells);
    this->ready_cells.clear();

    const size_t flagged = this->reportPoints(point_probability, threshold);

    // This scan becomes history
    if (this->history_depth > 0)
    {
        std::rotate(
            this->earlier_scans.rbegin(),
            this->earlier_scans.rbegin() + 1,
            this->earlier_scans.rend());
        std::swap(this->earlier_scans[0], this->current_scan);
    }
    return flagged;
}



// --- Scan bookkeeping --------------------------------------------------------

/* A cell's returns are all in: its log range (NaN = nothing to classify
 * there) and whether it changed against the scan before. */
inline void Filter::finalizeCell(size_t cell)
{
    const Model& settings = this->artifact_model;
    if (this->cell_range_mm[cell] < this->min_range_mm)
    {
        return;  // stays NaN
    }
    this->cell_range_m[cell] =
        static_cast<float>(this->cell_range_mm[cell]) / 1000.f;
    // In double, then rounded: a float log differs in the last bit between
    // libraries, and that bit can flip a tree's decision (the Python does
    // the same, see artifact_features._log).
    const float log_range = static_cast<float>(
        std::log(static_cast<double>(this->cell_range_m[cell])));
    this->current_scan.log_range[cell] = log_range;

    for (size_t offset_index = 0; offset_index < settings.time_offsets.size();
         offset_index++)
    {
        if (settings.time_offsets[offset_index] != -1)
        {
            continue;
        }
        // Present in both with another range, or present now and absent
        // then.
        const float earlier =
            this->compared_log_range[offset_index]
                ? (*this->compared_log_range[offset_index])[cell]
                : NOT_A_NUMBER;
        if (std::isnan(earlier))
        {
            this->gone_cells[offset_index][cell] = 1;
        }
        else if (std::abs(earlier - log_range) > settings.close_log)
        {
            this->unstable_cells[offset_index][cell] = 1;
        }
    }
}

/* Features and trees for the given cells. With OpenMP the cells are shared
 * out in small chunks: with early exit their cost varies a lot. */
inline void Filter::classifyCells(const std::vector<int32_t>& cells)
{
    const int columns = this->artifact_model.num_columns;
    const int num_features = this->artifact_model.num_features;
    const long num_cells_to_classify = static_cast<long>(cells.size());
#ifdef _OPENMP
    #pragma omp parallel if (num_cells_to_classify > 64)
#endif
    {
        std::vector<float> features(static_cast<size_t>(num_features));
#ifdef _OPENMP
    #pragma omp for schedule(dynamic, 8)
#endif
        for (long i = 0; i < num_cells_to_classify; i++)
        {
            const int32_t cell = cells[static_cast<size_t>(i)];
            const size_t index = static_cast<size_t>(cell);
            this->computeFeatures(
                cell / columns,
                cell % columns,
                features.data());
            this->cell_probability[index] = this->artifact_model.predict(
                features.data(),
                this->early_exit_enabled);
            this->cell_classified[index] = 1;
            if (this->keep_features)
            {
                std::copy(
                    features.begin(),
                    features.end(),
                    this->kept_features.begin() +
                        static_cast<long>(cell) * num_features);
            }
        }
    }
}

/* Cells back to points: the nearest return of a cell (at least min_range_m
 * away) gets the cell's probability, every other point -1. */
inline size_t Filter::reportPoints(
    std::vector<float>& point_probability,
    double threshold) const
{
    const size_t num_points = this->point_cell.size();
    point_probability.assign(num_points, -1.f);
    size_t flagged = 0;
    for (size_t i = 0; i < num_points; i++)
    {
        const int32_t cell = this->point_cell[i];
        if (cell < 0)
        {
            continue;
        }
        const size_t index = static_cast<size_t>(cell);
        const double range = this->point_range_m[i];
        if (range >= this->artifact_model.min_range_m &&
            range <= this->cell_nearest_range_m[index])
        {
            point_probability[i] = this->cell_probability[index];
            flagged += (point_probability[i] >= threshold);
        }
    }
    return flagged;
}



// --- Features ----------------------------------------------------------------

inline size_t Filter::cellIndex(int ring, int column) const
{
    const long columns = this->artifact_model.num_columns;
    return static_cast<size_t>(ring) * static_cast<size_t>(columns) +
           static_cast<size_t>(wrap(column, columns));
}

/* A cell's value; NaN beyond the top/bottom ring, wraps in azimuth. */
inline float Filter::cellValue(
    const std::vector<float>& grid,
    int ring,
    int column) const
{
    if (ring < 0 || ring >= this->artifact_model.num_rings)
    {
        return NOT_A_NUMBER;
    }
    return grid[this->cellIndex(ring, column)];
}

inline float Filter::clipped(float difference) const
{
    // NaN stays NaN
    const float clip = this->artifact_model.clip;
    return std::isnan(difference) ? difference
                                  : std::min(std::max(difference, -clip), clip);
}

/* Sum of a 0/1 grid over rings +-1, columns +-2. */
inline float Filter::countAround(
    const std::vector<uint8_t>& grid,
    int ring,
    int column) const
{
    int count = 0;
    for (int ring_offset = -1; ring_offset <= 1; ring_offset++)
    {
        const int other_ring = ring + ring_offset;
        if (other_ring < 0 || other_ring >= this->artifact_model.num_rings)
        {
            continue;
        }
        for (int column_offset = -2; column_offset <= 2; column_offset++)
        {
            count += grid[this->cellIndex(other_ring, column + column_offset)];
        }
    }
    return static_cast<float>(count);
}

/* The inputs of the classifier for one cell, in the order of
 * artifact_features.feature_names(). */
inline void Filter::computeFeatures(int ring, int column, float* features) const
{
    const size_t cell = this->cellIndex(ring, column);
    float* out = features;
    *out++ = this->cell_range_m[cell];
    *out++ = static_cast<float>(ring);
    out = this->addNeighbourFeatures(ring, column, out);
    if (!this->artifact_model.time_offsets.empty())
    {
        out = this->addEarlierScanFeatures(ring, column, out);
    }
    if (this->artifact_model.use_intensity)
    {
        out = this->addIntensityFeatures(ring, column, out);
    }
}

/* The log range difference to every neighbour, and summaries of the 3 x 5
 * patch around the cell. */
inline float* Filter::addNeighbourFeatures(int ring, int column, float* out)
    const
{
    const Model& settings = this->artifact_model;
    const std::vector<float>& log_range = this->current_scan.log_range;
    const float own = log_range[this->cellIndex(ring, column)];

    float agreeing_in_patch = 0;
    float valid_in_patch = 0;
    float agreeing_in_row = 0;
    float behind_in_patch = 0;
    float in_front_in_patch = 0;
    float patch_sum = 0;
    float patch_sum_of_squares = 0;
    float left_difference = NOT_A_NUMBER;
    float right_difference = NOT_A_NUMBER;

    for (const int ring_offset : settings.ring_offsets)
    {
        for (const int column_offset : settings.column_offsets)
        {
            if (ring_offset == 0 && column_offset == 0)
            {
                continue;
            }
            const float difference = this->clipped(
                this->cellValue(
                    log_range,
                    ring + ring_offset,
                    column + column_offset) -
                own);
            *out++ = difference;

            const bool valid = !std::isnan(difference);
            const bool agrees =
                valid && std::abs(difference) < settings.close_log;
            if (std::abs(ring_offset) <= 1 && std::abs(column_offset) <= 2)
            {
                agreeing_in_patch += agrees;
                valid_in_patch += valid;
                behind_in_patch += (valid && difference > settings.far_log);
                in_front_in_patch += (valid && difference < -settings.far_log);
                const float limited =
                    valid ? std::min(std::max(difference, -1.f), 1.f) : 0.f;
                patch_sum += limited;
                patch_sum_of_squares += limited * limited;
            }
            if (ring_offset == 0)
            {
                agreeing_in_row += agrees;
                if (column_offset == -1)
                {
                    left_difference = difference;
                }
                if (column_offset == 1)
                {
                    right_difference = difference;
                }
            }
        }
    }

    const float patch_count = std::max(valid_in_patch, 1.f);
    const float patch_mean = patch_sum / patch_count;
    *out++ = agreeing_in_patch;
    *out++ = valid_in_patch;
    *out++ = agreeing_in_row;
    // NaN only if both are
    *out++ = std::fmin(std::abs(left_difference), std::abs(right_difference));
    // > 0: strictly between its two row neighbours
    *out++ = -left_difference * right_difference;
    *out++ = behind_in_patch;
    *out++ = in_front_in_patch;
    *out++ = std::sqrt(
        std::max(
            patch_sum_of_squares / patch_count - patch_mean * patch_mean,
            0.f));
    return out;
}

/* The same and the adjacent cells in the earlier scans. */
inline float* Filter::addEarlierScanFeatures(int ring, int column, float* out)
    const
{
    const Model& settings = this->artifact_model;
    const size_t num_offsets = settings.time_offsets.size();
    const float own =
        this->current_scan.log_range[this->cellIndex(ring, column)];

    float agreeing_over_time = 0;
    float same_cell_differences[8];
    int num_same_cell = 0;
    for (size_t offset_index = 0; offset_index < num_offsets; offset_index++)
    {
        for (int column_offset = -1; column_offset <= 1; column_offset++)
        {
            const float earlier =
                this->compared_log_range[offset_index]
                    ? this->cellValue(
                          *this->compared_log_range[offset_index],
                          ring,
                          column + column_offset)
                    : NOT_A_NUMBER;
            const float difference = this->clipped(earlier - own);
            *out++ = difference;
            if (column_offset == 0 && !std::isnan(difference))
            {
                agreeing_over_time +=
                    (std::abs(difference) < settings.close_log);
                if (num_same_cell < 7)
                {
                    same_cell_differences[num_same_cell++] = difference;
                }
            }
        }
    }
    *out++ = agreeing_over_time;

    // |median| of the same-cell differences together with one zero; a
    // handful of values, so an insertion sort
    same_cell_differences[num_same_cell++] = 0.f;
    for (int i = 1; i < num_same_cell; i++)
    {
        const float value = same_cell_differences[i];
        int j = i - 1;
        for (; j >= 0 && same_cell_differences[j] > value; j--)
        {
            same_cell_differences[j + 1] = same_cell_differences[j];
        }
        same_cell_differences[j + 1] = value;
    }
    const int middle = num_same_cell / 2;
    const float median = (num_same_cell % 2)
                             ? same_cell_differences[middle]
                             : 0.5f * (same_cell_differences[middle - 1] +
                                       same_cell_differences[middle]);
    *out++ = std::abs(median);

    for (size_t offset_index = 0; offset_index < num_offsets; offset_index++)
    {
        if (settings.time_offsets[offset_index] == -1)
        {
            *out++ = this->countAround(
                this->unstable_cells[offset_index],
                ring,
                column);
        }
    }
    for (size_t offset_index = 0; offset_index < num_offsets; offset_index++)
    {
        if (settings.time_offsets[offset_index] == -1)
        {
            *out++ =
                this->countAround(this->gone_cells[offset_index], ring, column);
        }
    }
    return out;
}

/* The cell's intensity and its difference to the four direct neighbours. */
inline float* Filter::addIntensityFeatures(int ring, int column, float* out)
    const
{
    const auto intensityAt = [this](int other_ring, int other_column) -> float
    {
        if (other_ring < 0 || other_ring >= this->artifact_model.num_rings)
        {
            return NOT_A_NUMBER;
        }
        const size_t cell = this->cellIndex(other_ring, other_column);
        return std::isnan(this->cell_range_m[cell])
                   ? NOT_A_NUMBER
                   : static_cast<float>(this->cell_intensity[cell]) / 65535.f;
    };
    const float own = intensityAt(ring, column);
    *out++ = own;
    *out++ = intensityAt(ring, column - 1) - own;
    *out++ = intensityAt(ring, column + 1) - own;
    *out++ = intensityAt(ring - 1, column) - own;
    *out++ = intensityAt(ring + 1, column) - own;
    return out;
}

}  // namespace artifact_filter
