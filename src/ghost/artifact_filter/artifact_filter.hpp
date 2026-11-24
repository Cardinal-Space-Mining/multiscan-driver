/*
 * artifact_filter.hpp
 *
 * Runs the lidar artifact classifier trained in annotator_app on live scans.
 * Self-contained: the C++ standard library only (OpenMP is used when the
 * compiler has it), no ROS and no PCL, so it can be dropped into any node.
 *
 *     artifact_filter::Filter filter(artifact_filter::Model::load(path));
 *     ...for every scan, in order...
 *     filter.process(xyz, n_points, intensity_or_null, stamp_ns, point_prob);
 *     // point_prob[i] >= filter.model().threshold  ->  point i is an artifact
 *
 * A scan can also be fed in pieces as the lidar sends it (begin, add, cover,
 * classify_ready, finish; see Filter::begin): cells are classified as soon
 * as their inputs are complete, so little work is left when the scan ends.
 *
 * The model file is written by annotator_app (artifact_<variant>.model.txt
 * next to every trained model, or export_model.py). It carries the trees and
 * every setting used to build their inputs, so nothing here is specific to
 * one lidar.
 *
 * This file mirrors annotator_app/scan_grid.py (points -> grid cells) and
 * annotator_app/artifact_features.py (grid -> features). The two must stay in
 * step: after changing either, run artifact_filter_check against a check file
 * from export_model.py.
 *
 * Only models that look at earlier scans can run here ("before only
 * (realtime)" in the app). Model::load refuses one that needs later scans.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace artifact_filter
{

// ─────────────────────────────────────────────────────────────────────────────
// Model: settings and trees, as read from the exported text file
// ─────────────────────────────────────────────────────────────────────────────

struct Model
{
    // sensor grid
    int rings = 0;
    int cols = 0;
    std::vector<double> layer_elev_deg;  // ascending; ring 0 is the LAST entry
    std::vector<double> layer_az0_deg;
    std::vector<int> col0;
    double col_deg = 1.;
    double scan_rate_hz = 0.;
    std::string intensity_field;         // empty: the sensor has none
    double intensity_max = 65535.;

    // features
    bool use_intensity = false;
    double min_range_m = 0.2;
    std::vector<int> ring_offsets;
    std::vector<int> col_offsets;
    float close_log = 0.05f;
    float far_log = 0.4f;
    float clip = 4.f;
    int64_t max_dt_ns = 0;
    std::vector<int> time_offsets;       // all negative
    int n_features = 0;

    // classifier
    double threshold = 0.5;              // the default chosen at training
    bool compare_lt = true;              // split rule: x < t (else x <= t)
    double base_margin = 0.;
    /** One tree node, 24 bytes so a step down a tree touches one cache line.
      * Leaf: feature < 0 and `split` holds the leaf's output. */
    struct Node
    {
        double split;
        int32_t feature;
        int32_t left, right;     // absolute node indices
        uint32_t missing_left;
    };
    std::vector<int32_t> tree_root;      // index of every tree's first node
    std::vector<Node> nodes;

    // Early exit (optional, see predict): after exit_after[i] trees a return
    // whose running sum is below exit_below[i] is given up as clean.
    std::vector<int32_t> exit_after;
    std::vector<double> exit_below;

    std::string variant, trained;

    static Model load(const std::string& path);

    /**
     * Probability that the return described by `x` (n_features values, NaN
     * for missing) is an artifact.
     *
     * With early_exit, evaluation stops at the model's checkpoints once the
     * running sum is so low that the return practically cannot end up flagged
     * (the checkpoints are calibrated at training against the full model, for
     * thresholds of 0.05 and up). Most returns are clean and stop after a
     * fraction of the trees. A return that stopped early gets the probability
     * of its partial sum: below the threshold, but not the exact value.
     */
    float predict(const float* x, bool early_exit = false) const
    {
        const Node* const nd = this->nodes.data();
        const size_t n_trees = this->tree_root.size();
        const size_t n_exit = early_exit ? this->exit_after.size() : 0;
        size_t e = 0;
        size_t stop = n_exit ? static_cast<size_t>(this->exit_after[0]) : n_trees;
        double sum = this->base_margin;
        size_t t = 0;
        for (;;)
        {
            for (; t < stop; t++)
            {
                const Node* n = nd + this->tree_root[t];
                while (n->feature >= 0)
                {
                    const double v = static_cast<double>(x[n->feature]);
                    const bool go_left =
                        std::isnan(v) ? n->missing_left != 0
                                      : (this->compare_lt ? v < n->split : v <= n->split);
                    n = nd + (go_left ? n->left : n->right);
                }
                sum += n->split;
            }
            if (t >= n_trees) { break; }
            if (sum < this->exit_below[e]) { break; }
            e++;
            stop = e < n_exit ? static_cast<size_t>(this->exit_after[e]) : n_trees;
        }
        return static_cast<float>(1. / (1. + std::exp(-sum)));
    }

    /** Number of inputs the feature code below produces for these settings. */
    int expected_features() const
    {
        int n = 2 +
            static_cast<int>(this->ring_offsets.size() * this->col_offsets.size()) - 1 + 8;
        if (!this->time_offsets.empty())
        {
            n += 3 * static_cast<int>(this->time_offsets.size()) + 2;
            for (const int dt : this->time_offsets)
            {
                if (std::abs(dt) == 1) { n += 2; }
            }
        }
        return n + (this->use_intensity ? 5 : 0);
    }
};

inline Model Model::load(const std::string& path)
{
    std::ifstream in(path);
    if (!in) { throw std::runtime_error("cannot open model file " + path); }

    const auto fail = [&path](const std::string& what) -> void
    {
        throw std::runtime_error("model file " + path + ": " + what);
    };

    std::string line, key;
    if (!std::getline(in, line) || line.rfind("lidar_artifact_model 1", 0) != 0)
    {
        fail("not a 'lidar_artifact_model 1' file (export it with annotator_app)");
    }

    Model m;
    size_t n_trees = 0, n_nodes = 0;
    while (std::getline(in, line))
    {
        std::istringstream ss(line);
        ss >> key;
        const auto list = [&ss](auto& out)
        {
            size_t n = 0;
            ss >> n;
            out.resize(n);
            for (auto& v : out) { ss >> v; }
        };

        if (key == "variant") { ss >> m.variant; }
        else if (key == "trained") { ss >> m.trained; }
        else if (key == "threshold") { ss >> m.threshold; }
        else if (key == "cols") { ss >> m.cols; }
        else if (key == "layer_elevations_deg") { list(m.layer_elev_deg); }
        else if (key == "layer_azimuth0_deg") { list(m.layer_az0_deg); }
        else if (key == "scan_rate_hz") { ss >> m.scan_rate_hz; }
        else if (key == "use_intensity") { int v = 0; ss >> v; m.use_intensity = v != 0; }
        else if (key == "intensity_field")
        {
            ss >> m.intensity_field;
            if (m.intensity_field == "-") { m.intensity_field.clear(); }
        }
        else if (key == "intensity_max") { ss >> m.intensity_max; }
        else if (key == "min_range_m") { ss >> m.min_range_m; }
        else if (key == "ring_offsets") { list(m.ring_offsets); }
        else if (key == "col_offsets") { list(m.col_offsets); }
        else if (key == "close_log") { ss >> m.close_log; }
        else if (key == "far_log") { ss >> m.far_log; }
        else if (key == "clip") { ss >> m.clip; }
        else if (key == "max_dt_ns") { ss >> m.max_dt_ns; }
        else if (key == "time_offsets") { list(m.time_offsets); }
        else if (key == "n_features") { ss >> m.n_features; }
        else if (key == "compare") { std::string c; ss >> c; m.compare_lt = (c == "lt"); }
        else if (key == "base_margin") { ss >> m.base_margin; }
        else if (key == "early_exit")
        {
            size_t n = 0;
            ss >> n;
            m.exit_after.resize(n);
            m.exit_below.resize(n);
            for (size_t i = 0; i < n; i++) { ss >> m.exit_after[i] >> m.exit_below[i]; }
        }
        else if (key == "trees") { ss >> n_trees >> n_nodes; break; }
        // unknown keys (sensor_fingerprint, later additions) are skipped
    }
    if (!n_trees || !n_nodes) { fail("no trees"); }

    m.rings = static_cast<int>(m.layer_elev_deg.size());
    if (m.rings < 3 || m.cols < 16 || m.layer_az0_deg.size() != m.layer_elev_deg.size())
    {
        fail("incomplete sensor grid");
    }
    m.col_deg = 360. / m.cols;
    for (const double az0 : m.layer_az0_deg)
    {
        m.col0.push_back(static_cast<int>(std::floor((az0 + 180.) / m.col_deg)));
    }

    m.tree_root.reserve(n_trees);
    m.nodes.reserve(n_nodes);
    for (size_t t = 0; t < n_trees; t++)
    {
        size_t n = 0;
        if (!(in >> key >> n) || key != "tree" || !n) { fail("truncated (tree header)"); }
        const int32_t base = static_cast<int32_t>(m.nodes.size());
        m.tree_root.push_back(base);
        for (size_t i = 0; i < n; i++)
        {
            int f, l, r, ml;
            std::string thr_s, val_s;   // as text: a split can be "-inf", which
                                        // stream extraction does not read
            if (!(in >> f >> thr_s >> l >> r >> ml >> val_s)) { fail("truncated (tree nodes)"); }
            const double thr = std::strtod(thr_s.c_str(), nullptr);
            const double val = std::strtod(val_s.c_str(), nullptr);
            if (f >= m.n_features ||
                (f >= 0 && (l < 0 || r < 0 || l >= static_cast<int>(n) || r >= static_cast<int>(n))))
            {
                fail("corrupt tree node");
            }
            Node node;
            node.feature = f;
            node.split = f >= 0 ? thr : val;
            node.left = f >= 0 ? base + l : -1;
            node.right = f >= 0 ? base + r : -1;
            node.missing_left = static_cast<uint32_t>(ml != 0);
            m.nodes.push_back(node);
        }
    }
    for (size_t i = 0; i < m.exit_after.size(); i++)
    {
        if (m.exit_after[i] <= (i ? m.exit_after[i - 1] : 0) ||
            static_cast<size_t>(m.exit_after[i]) >= n_trees)
        {
            fail("bad early_exit checkpoints");
        }
    }

    for (const int dt : m.time_offsets)
    {
        if (dt >= 0)
        {
            fail(
                "this model looks at scans after the one it classifies, which a "
                "live filter does not have. Train a realtime model (\"before only "
                "(realtime)\" in the app, or --time-offsets -2 -1) and export that");
        }
    }
    const auto has = [](const std::vector<int>& v, int x)
    {
        return std::find(v.begin(), v.end(), x) != v.end();
    };
    if (!has(m.ring_offsets, 0) || !has(m.col_offsets, -1) || !has(m.col_offsets, 1))
    {
        fail("unsupported neighbourhood offsets");
    }
    if (m.expected_features() != m.n_features)
    {
        fail(
            "expects " + std::to_string(m.n_features) + " features but this code "
            "produces " + std::to_string(m.expected_features()) +
            " for its settings: artifact_filter.hpp is out of step with "
            "annotator_app/artifact_features.py");
    }
    return m;
}


// ─────────────────────────────────────────────────────────────────────────────
// Filter: scans in, probability per point out
// ─────────────────────────────────────────────────────────────────────────────

class Filter
{
public:
    explicit Filter(Model model) :
        m(std::move(model)),
        n_cells(static_cast<size_t>(m.rings) * static_cast<size_t>(m.cols))
    {
        for (const int dt : this->m.time_offsets)
        {
            this->depth = std::max(this->depth, -dt);
        }
        this->history.resize(static_cast<size_t>(this->depth));
        this->min_range_mm =
            static_cast<int>(std::lround(this->m.min_range_m * 1000.));
        // cells whose neighbourhood contains a cell (and the other way round)
        int span = 1;
        for (const int dr : this->m.ring_offsets) { span = std::max(span, std::abs(dr)); }
        this->nb_rings = span;
        span = 2;                                   // box3x5 reaches 2 columns
        for (const int dc : this->m.col_offsets) { span = std::max(span, std::abs(dc)); }
        this->nb_cols = span;
    }

    const Model& model() const { return this->m; }

    /** Forget the earlier scans (after a gap in the stream this happens by
      * itself: scans too far apart in time are not used as neighbours). */
    void reset()
    {
        for (auto& h : this->history) { h.valid = false; }
    }

    /**
     * Classifies one scan. Scans must be given in the order they were taken.
     *
     *  xyz         n_points * 3 floats, in the lidar's own frame
     *  intensity   n_points raw intensity values, or nullptr. Required when
     *              model().use_intensity.
     *  stamp_ns    the scan's time stamp
     *  point_prob  out, n_points values: probability that the point is an
     *              artifact, or -1 when it was not classified (invalid, nearer
     *              than the minimum range, or not the nearest return of its
     *              beam in a multi-echo scan)
     *
     * Returns the number of points with probability >= threshold (threshold
     * < 0: the model's own).
     */
    size_t process(
        const float* xyz,
        size_t n_points,
        const float* intensity,
        int64_t stamp_ns,
        std::vector<float>& point_prob,
        double threshold = -1.)
    {
        this->begin(stamp_ns);
        this->add(n_points, xyz, 3 * sizeof(float), intensity, sizeof(float));
        return this->finish(point_prob, threshold);
    }

    // ── The same scan fed in pieces, as a lidar sends it ────────────────────
    //
    //     filter.begin(stamp_ns);
    //     for every piece (e.g. a network packet):
    //         filter.add(n, xyz, xyz_step, intensity, intensity_step);
    //         for every beam the piece carries, return or not:
    //             filter.cover(filter.cell_at(azimuth_deg, elevation_deg));
    //         filter.classify_ready();
    //     filter.finish(point_prob);
    //
    // A cell is classified as soon as every cell its inputs look at is
    // covered, so most of the work is done before the last piece arrives.
    // The result is identical to process() on all points in the same order:
    // cover() only tells the filter that no more returns will come for a
    // cell. Covering a cell that still gets returns later breaks that; cells
    // never covered are handled in finish().

    /** Starts a scan. */
    void begin(int64_t stamp_ns)
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const int R = this->m.rings;
        this->cell.clear();
        this->range_pt.clear();
        this->nearest.assign(this->n_cells, std::numeric_limits<double>::infinity());
        this->range_mm.assign(this->n_cells, 0);
        this->inten.assign(this->m.use_intensity ? this->n_cells : 0, 0);
        this->current.logr.assign(this->n_cells, nan);
        this->current.stamp = stamp_ns;
        this->current.valid = true;
        this->rng_m.assign(this->n_cells, nan);
        this->cell_prob.assign(this->n_cells, 0.f);
        this->covered.assign(this->n_cells, 0);
        this->done.assign(this->n_cells, 0);
        this->missing.resize(this->n_cells);
        for (int r = 0; r < R; r++)
        {
            const int rings =
                std::min(r + this->nb_rings, R - 1) - std::max(r - this->nb_rings, 0) + 1;
            std::fill_n(this->missing.begin() + static_cast<long>(r) * this->m.cols,
                        this->m.cols, rings * (2 * this->nb_cols + 1));
        }
        this->ready.clear();
        if (this->keep_feats)
        {
            this->feats.assign(this->n_cells * static_cast<size_t>(this->m.n_features), nan);
        }

        // Earlier scans that are close enough in time to compare with
        // (history[k - 1] is the scan k steps back).
        const size_t n_dt = this->m.time_offsets.size();
        this->prev.assign(n_dt, nullptr);
        this->unstable.resize(n_dt);
        this->gone.resize(n_dt);
        for (size_t t = 0; t < n_dt; t++)
        {
            const int back = -this->m.time_offsets[t];
            const Scan& h = this->history[static_cast<size_t>(back - 1)];
            const int64_t gap = stamp_ns > h.stamp ? stamp_ns - h.stamp : h.stamp - stamp_ns;
            if (h.valid && gap <= back * this->m.max_dt_ns) { this->prev[t] = &h.logr; }
            if (back == 1)
            {
                this->unstable[t].assign(this->n_cells, 0);
                this->gone[t].assign(this->n_cells, 0);
            }
        }
    }

    /**
     * Adds n points of the scan. xyz: x y z floats of the first point, the
     * next one xyz_step bytes further; intensity likewise (nullptr: none,
     * which only a model without intensity accepts). Points keep their order
     * across calls: finish() reports them in the order they were added.
     */
    void add(
        size_t n,
        const float* xyz,
        size_t xyz_step,
        const float* intensity,
        size_t intensity_step)
    {
        if (this->m.use_intensity && !intensity && n)
        {
            throw std::invalid_argument(
                "artifact_filter: this model needs intensity, the scan has none "
                "(use the range-only model, artifact_xyz)");
        }
        const auto* px = reinterpret_cast<const uint8_t*>(xyz);
        const auto* pi = reinterpret_cast<const uint8_t*>(intensity);
        for (size_t i = 0; i < n; i++, px += xyz_step, pi += intensity_step)
        {
            // Points -> grid cells; the nearest return of a cell wins.
            float p[3];
            std::memcpy(p, px, sizeof(p));
            const double x = p[0], y = p[1], z = p[2];
            const double xy = std::hypot(x, y);
            const double rng = std::hypot(xy, z);
            this->range_pt.push_back(rng);
            if (!std::isfinite(rng) || !(rng > 1e-4))
            {
                this->cell.push_back(-1);
                continue;
            }
            const int32_t c = this->cell_at(std::atan2(y, x) * RAD2DEG,
                                            std::atan2(z, xy) * RAD2DEG);
            this->cell.push_back(c);
            if (rng <= this->nearest[static_cast<size_t>(c)])
            {
                this->nearest[static_cast<size_t>(c)] = rng;
                this->range_mm[static_cast<size_t>(c)] = static_cast<uint16_t>(
                    std::clamp(std::nearbyint(rng * 1000.), 0., 65535.));
                if (this->m.use_intensity)
                {
                    float f;
                    std::memcpy(&f, pi, sizeof(f));
                    double v = f;
                    if (this->m.intensity_max != 65535.)
                    {
                        v = std::nearbyint(v * (65535. / this->m.intensity_max));
                    }
                    this->inten[static_cast<size_t>(c)] =
                        static_cast<uint16_t>(std::clamp(v, 0., 65535.));
                }
            }
        }
    }

    /** Grid cell (ring * cols + col) of a direction in the lidar's frame:
      * the nearest layer by elevation, the column by azimuth. */
    int32_t cell_at(double azimuth_deg, double elevation_deg) const
    {
        const int R = this->m.rings, C = this->m.cols;
        int k = 0;
        double best = std::abs(elevation_deg - this->m.layer_elev_deg[0]);
        for (int j = 1; j < R; j++)
        {
            const double d = std::abs(elevation_deg - this->m.layer_elev_deg[j]);
            if (d < best) { best = d; k = j; }
        }
        // nearbyint: round half to even, as numpy does
        const long idx = static_cast<long>(
            std::nearbyint((azimuth_deg - this->m.layer_az0_deg[k]) / this->m.col_deg));
        const int col = static_cast<int>(mod(idx + this->m.col0[k], C));
        const int ring = (R - 1) - k;
        return ring * C + col;
    }

    /** No more returns will be added for cell c (see above). */
    void cover(int32_t c)
    {
        if (c < 0 || static_cast<size_t>(c) >= this->n_cells ||
            this->covered[static_cast<size_t>(c)])
        {
            return;
        }
        this->covered[static_cast<size_t>(c)] = 1;
        this->finalize(static_cast<size_t>(c));
        const int C = this->m.cols;
        const int ring = c / C, col = c % C;
        for (int r = std::max(ring - this->nb_rings, 0);
             r <= std::min(ring + this->nb_rings, this->m.rings - 1); r++)
        {
            for (int dc = -this->nb_cols; dc <= this->nb_cols; dc++)
            {
                const size_t n = static_cast<size_t>(r) * static_cast<size_t>(C) +
                    static_cast<size_t>(mod(col + dc, C));
                if (--this->missing[n] == 0 && !std::isnan(this->current.logr[n]))
                {
                    this->ready.push_back(static_cast<int32_t>(n));
                }
            }
        }
    }

    /** Classifies the cells whose inputs are complete. Returns how many. */
    size_t classify_ready()
    {
        const size_t n = this->ready.size();
        this->classify(this->ready);
        this->ready.clear();
        return n;
    }

    /**
     * Ends the scan: classifies what is left and reports every point added
     * since begin() (see process()). Returns the number of points with
     * probability >= threshold (threshold < 0: the model's own).
     */
    size_t finish(std::vector<float>& point_prob, double threshold = -1.)
    {
        if (threshold < 0.) { threshold = this->m.threshold; }

        // Cells never covered are complete now.
        this->ready.clear();
        for (size_t c = 0; c < this->n_cells; c++)
        {
            if (!this->covered[c])
            {
                this->covered[c] = 1;
                this->finalize(c);
            }
        }
        for (size_t c = 0; c < this->n_cells; c++)
        {
            if (!this->done[c] && !std::isnan(this->current.logr[c]))
            {
                this->ready.push_back(static_cast<int32_t>(c));
            }
        }
        this->classify(this->ready);
        this->ready.clear();

        // Back to points
        const size_t n_points = this->cell.size();
        point_prob.assign(n_points, -1.f);
        size_t flagged = 0;
        for (size_t i = 0; i < n_points; i++)
        {
            const int32_t c = this->cell[i];
            if (c < 0) { continue; }
            const double rng = this->range_pt[i];
            if (rng >= this->m.min_range_m && rng <= this->nearest[static_cast<size_t>(c)])
            {
                point_prob[i] = this->cell_prob[static_cast<size_t>(c)];
                flagged += (point_prob[i] >= threshold);
            }
        }

        // This scan becomes history
        if (this->depth > 0)
        {
            std::rotate(this->history.rbegin(), this->history.rbegin() + 1, this->history.rend());
            std::swap(this->history[0], this->current);
        }
        return flagged;
    }

    /** Probability per grid cell (ring * cols + col) of the last scan; 0
      * where there was nothing to classify. */
    const std::vector<float>& cell_probabilities() const { return this->cell_prob; }

    /** Stop evaluating the trees early for returns that are clearly clean
      * (see Model::predict). Several times faster; flags practically the same
      * points; probabilities under the threshold are no longer exact. Does
      * nothing when the model file carries no checkpoints. */
    void early_exit(bool on) { this->early = on; }
    bool early_exit() const { return this->early && !this->m.exit_after.empty(); }

    /** For checking against the Python: keep the classifier inputs of every
      * scan. last_features() is then cells * n_features values, NaN for cells
      * with nothing to classify. Off by default (it costs time). */
    void keep_features(bool on) { this->keep_feats = on; }
    const std::vector<float>& last_features() const { return this->feats; }

private:
    struct Scan
    {
        std::vector<float> logr;
        int64_t stamp = 0;
        bool valid = false;
    };

    static constexpr double RAD2DEG = 180. / 3.14159265358979323846;

    static long mod(long a, long n) { return ((a % n) + n) % n; }

    /** A cell's returns are all in: its log range (NaN = nothing to
      * classify there) and whether it changed against the scan before. */
    void finalize(size_t c)
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        if (this->range_mm[c] < this->min_range_mm) { return; }    // stays NaN
        this->rng_m[c] = static_cast<float>(this->range_mm[c]) / 1000.f;
        // In double, then rounded: a float log differs in the last bit
        // between libraries, and that bit can flip a tree's decision
        // (the Python does the same, see artifact_features._log).
        const float lr = static_cast<float>(std::log(static_cast<double>(this->rng_m[c])));
        this->current.logr[c] = lr;
        for (size_t t = 0; t < this->m.time_offsets.size(); t++)
        {
            if (this->m.time_offsets[t] != -1) { continue; }
            // Present in both with another range, or present now and
            // absent then.
            const float then = this->prev[t] ? (*this->prev[t])[c] : nan;
            if (std::isnan(then)) { this->gone[t][c] = 1; }
            else if (std::abs(then - lr) > this->m.close_log) { this->unstable[t][c] = 1; }
        }
    }

    /** Features and trees for the given cells. With OpenMP the cells are
      * shared out in small chunks: with early exit their cost varies a lot. */
    void classify(const std::vector<int32_t>& cells)
    {
        const int C = this->m.cols;
        const int F = this->m.n_features;
        const long n = static_cast<long>(cells.size());
#ifdef _OPENMP
        #pragma omp parallel if (n > 64)
#endif
        {
            std::vector<float> x(static_cast<size_t>(F));
#ifdef _OPENMP
            #pragma omp for schedule(dynamic, 8)
#endif
            for (long i = 0; i < n; i++)
            {
                const int32_t c = cells[static_cast<size_t>(i)];
                this->features(c / C, c % C, x.data());
                this->cell_prob[static_cast<size_t>(c)] = this->m.predict(x.data(), this->early);
                this->done[static_cast<size_t>(c)] = 1;
                if (this->keep_feats)
                {
                    std::copy(x.begin(), x.end(),
                              this->feats.begin() + static_cast<long>(c) * F);
                }
            }
        }
    }

    /** log range of a cell; NaN beyond the top/bottom ring, wraps in azimuth */
    float at(const std::vector<float>& g, int ring, int col) const
    {
        if (ring < 0 || ring >= this->m.rings) { return std::numeric_limits<float>::quiet_NaN(); }
        return g[static_cast<size_t>(ring) * static_cast<size_t>(this->m.cols) +
                 static_cast<size_t>(mod(col, this->m.cols))];
    }

    float clipped(float d) const
    {
        // NaN stays NaN
        return std::isnan(d) ? d : std::min(std::max(d, -this->m.clip), this->m.clip);
    }

    /** Sum of a 0/1 grid over rings +-1, cols +-2 */
    float box3x5(const std::vector<uint8_t>& g, int ring, int col) const
    {
        int n = 0;
        for (int dr = -1; dr <= 1; dr++)
        {
            const int r = ring + dr;
            if (r < 0 || r >= this->m.rings) { continue; }
            for (int dc = -2; dc <= 2; dc++)
            {
                n += g[static_cast<size_t>(r) * static_cast<size_t>(this->m.cols) +
                       static_cast<size_t>(mod(col + dc, this->m.cols))];
            }
        }
        return static_cast<float>(n);
    }

    /** The inputs of the classifier for one cell, in the order of
      * artifact_features.feature_names(). */
    void features(int ring, int col, float* x) const
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const std::vector<float>& logr = this->current.logr;
        const size_t c = static_cast<size_t>(ring) * static_cast<size_t>(this->m.cols) +
            static_cast<size_t>(col);
        const float own = logr[c];
        int k = 0;

        x[k++] = this->rng_m[c];
        x[k++] = static_cast<float>(ring);

        // log range ratio to every neighbour, and summaries of the 3 x 5 patch
        float close3x5 = 0, valid3x5 = 0, close_row = 0, behind = 0, front = 0, s1 = 0, s2 = 0;
        float l1 = nan, r1 = nan;
        for (const int dr : this->m.ring_offsets)
        {
            for (const int dc : this->m.col_offsets)
            {
                if (dr == 0 && dc == 0) { continue; }
                const float d = this->clipped(this->at(logr, ring + dr, col + dc) - own);
                x[k++] = d;
                const bool ok = !std::isnan(d);
                const bool agree = ok && std::abs(d) < this->m.close_log;
                if (std::abs(dr) <= 1 && std::abs(dc) <= 2)
                {
                    close3x5 += agree;
                    valid3x5 += ok;
                    behind += (ok && d > this->m.far_log);
                    front += (ok && d < -this->m.far_log);
                    const float dz = ok ? std::min(std::max(d, -1.f), 1.f) : 0.f;
                    s1 += dz;
                    s2 += dz * dz;
                }
                if (dr == 0)
                {
                    close_row += agree;
                    if (dc == -1) { l1 = d; }
                    if (dc == 1) { r1 = d; }
                }
            }
        }
        const float nv = std::max(valid3x5, 1.f);
        x[k++] = close3x5;
        x[k++] = valid3x5;
        x[k++] = close_row;
        x[k++] = std::fmin(std::abs(l1), std::abs(r1));     // NaN only if both are
        x[k++] = -l1 * r1;      // > 0: strictly between its two row neighbours
        x[k++] = behind;
        x[k++] = front;
        x[k++] = std::sqrt(std::max(s2 / nv - (s1 / nv) * (s1 / nv), 0.f));

        // the same and the adjacent cells in the earlier scans
        const size_t n_dt = this->m.time_offsets.size();
        if (n_dt)
        {
            float close_t = 0;
            float same[8];
            int n_same = 0;
            for (size_t t = 0; t < n_dt; t++)
            {
                for (int dc = -1; dc <= 1; dc++)
                {
                    const float then = this->prev[t] ? this->at(*this->prev[t], ring, col + dc) : nan;
                    const float d = this->clipped(then - own);
                    x[k++] = d;
                    if (dc == 0 && !std::isnan(d))
                    {
                        close_t += (std::abs(d) < this->m.close_log);
                        if (n_same < 7) { same[n_same++] = d; }
                    }
                }
            }
            x[k++] = close_t;
            // |median| of the same-cell differences together with one zero
            same[n_same++] = 0.f;
            for (int i = 1; i < n_same; i++)        // a handful of values: insertion sort
            {
                const float v = same[i];
                int j = i - 1;
                for (; j >= 0 && same[j] > v; j--) { same[j + 1] = same[j]; }
                same[j + 1] = v;
            }
            const float med = (n_same % 2) ? same[n_same / 2]
                                           : 0.5f * (same[n_same / 2 - 1] + same[n_same / 2]);
            x[k++] = std::abs(med);
            for (size_t t = 0; t < n_dt; t++)
            {
                if (this->m.time_offsets[t] == -1) { x[k++] = this->box3x5(this->unstable[t], ring, col); }
            }
            for (size_t t = 0; t < n_dt; t++)
            {
                if (this->m.time_offsets[t] == -1) { x[k++] = this->box3x5(this->gone[t], ring, col); }
            }
        }

        if (this->m.use_intensity)
        {
            const auto it = [this, nan](int r, int cc) -> float
            {
                if (r < 0 || r >= this->m.rings) { return nan; }
                const size_t j = static_cast<size_t>(r) * static_cast<size_t>(this->m.cols) +
                    static_cast<size_t>(mod(cc, this->m.cols));
                return std::isnan(this->rng_m[j]) ? nan
                                                  : static_cast<float>(this->inten[j]) / 65535.f;
            };
            const float i0 = it(ring, col);
            x[k++] = i0;
            x[k++] = it(ring, col - 1) - i0;
            x[k++] = it(ring, col + 1) - i0;
            x[k++] = it(ring - 1, col) - i0;
            x[k++] = it(ring + 1, col) - i0;
        }
    }

private:
    Model m;
    size_t n_cells;
    int depth = 0;
    int min_range_mm = 200;
    int nb_rings = 2, nb_cols = 4;      // neighbourhood half-size

    Scan current;
    std::vector<Scan> history;                       // [0] = one scan back
    std::vector<const std::vector<float>*> prev;     // per time offset, or null
    std::vector<std::vector<uint8_t>> unstable, gone;

    std::vector<int32_t> cell;          // per point
    std::vector<double> range_pt;       // per point
    std::vector<double> nearest;        // per cell
    std::vector<uint16_t> range_mm;     // per cell
    std::vector<uint16_t> inten;        // per cell
    std::vector<float> rng_m;           // per cell
    std::vector<float> cell_prob;
    std::vector<uint8_t> covered;       // per cell: all its returns are in
    std::vector<uint8_t> done;          // per cell: classified
    std::vector<int> missing;           // per cell: uncovered cells its inputs look at
    std::vector<int32_t> ready;
    bool keep_feats = false;
    bool early = false;
    std::vector<float> feats;
};

}   // namespace artifact_filter
