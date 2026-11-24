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

/*  Artifact (ghost point) classifier: the model file written by ghost-ml
 *  (annotator_app, export_model.py) and the trees it carries.
 *
 *  The file holds the trees and every setting needed to turn a scan into the
 *  classifier's inputs, so nothing here is specific to one lidar. See
 *  export_model.py for the format (version 1).
 *
 *  Header only, C++ standard library only (no ROS), so ghost-ml builds its
 *  tools against this same code. ghost-ml (https://github.com/S1ink/ghost-ml)
 *  is a separate project, not part of the LANCE workspace.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>


namespace artifact_filter
{

struct Model
{
public:
    static constexpr const char* FILE_HEADER = "lidar_artifact_model 1";

    /* One tree node, 24 bytes, so a step down a tree touches one cache
     * line. A leaf has feature < 0, and `split` holds the leaf's output. */
    struct Node
    {
        double split;
        int32_t feature;
        int32_t left_node;  // absolute node indices
        int32_t right_node;
        uint32_t missing_goes_left;
    };

public:
    /* Reads a model file. Throws std::runtime_error when the file can't be
     * read, is damaged, or doesn't fit the feature code (artifact_filter
     * .hpp out of step with ghost-ml artifact_features.py), or when the
     * model looks at later scans, which a live filter doesn't have. */
    static Model load(const std::string& path);

public:
    /* Probability that the return described by `features` (num_features
     * values, NaN for missing) is an artifact.
     *
     * With early_exit, evaluation stops at the model's checkpoints once the
     * running sum is so low that the return practically cannot end up
     * flagged (the checkpoints are calibrated at training against the full
     * model, for thresholds of 0.05 and up). Most returns are clean and stop
     * after a fraction of the trees. A return that stopped early gets the
     * probability of its partial sum: below the threshold, but not the exact
     * value. */
    float predict(const float* features, bool early_exit = false) const;

    /* Number of inputs the feature code produces for these settings. */
    int expectedFeatureCount() const;

    bool hasEarlyExit() const { return !this->early_exit_after_trees.empty(); }

public:
    // --- Sensor grid ---------------------------------------------------------
    int num_rings{0};
    int num_columns{0};
    /* Ascending; ring 0 is the LAST entry (the highest layer). */
    std::vector<double> layer_elevations_deg;
    /* Azimuth of each layer's first beam, and the column it falls in. */
    std::vector<double> layer_first_azimuth_deg;
    std::vector<int> layer_first_column;
    double column_width_deg{1.};
    double scan_rate_hz{0.};
    std::string intensity_field;  // empty: the sensor has none
    double intensity_max{65535.};

    // --- Features ------------------------------------------------------------
    bool use_intensity{false};
    double min_range_m{0.2};
    /* The neighbourhood compared with each cell. */
    std::vector<int> ring_offsets;
    std::vector<int> column_offsets;
    /* Log range differences: below close_log the ranges agree, above
     * far_log a neighbour is clearly behind or in front; differences are
     * clipped to +-clip. */
    float close_log{0.05f};
    float far_log{0.4f};
    float clip{4.f};
    /* Earlier scans compared with each scan (all negative: -1 is the scan
     * before), and the largest time between two scans that still counts. */
    std::vector<int> time_offsets;
    int64_t max_scan_gap_ns{0};
    int num_features{0};

    // --- Classifier ----------------------------------------------------------
    double threshold{0.5};       // the default chosen at training
    bool split_less_than{true};  // split rule: x < split (else x <= split)
    double base_margin{0.};
    std::vector<int32_t> tree_roots;  // index of every tree's first node
    std::vector<Node> nodes;
    /* Early exit (optional, see predict): after early_exit_after_trees[i]
     * trees, a return whose running sum is below early_exit_below_sum[i] is
     * given up as clean. */
    std::vector<int32_t> early_exit_after_trees;
    std::vector<double> early_exit_below_sum;

    // --- Provenance ----------------------------------------------------------
    std::string variant;  // "full" (range and intensity) or "xyz"
    std::string trained;  // ISO 8601 time of training

protected:
    struct TreeCounts
    {
        size_t trees{0};
        size_t nodes{0};
    };

protected:
    static TreeCounts readSettings(std::istream& in, Model& model);
    static void
        readTrees(std::istream& in, const TreeCounts& counts, Model& model);
    static void deriveGrid(Model& model);
    static void validate(const Model& model);
};



// --- Prediction --------------------------------------------------------------

inline float Model::predict(const float* features, bool early_exit) const
{
    const Node* const all_nodes = this->nodes.data();
    const size_t num_trees = this->tree_roots.size();
    const size_t num_checkpoints =
        early_exit ? this->early_exit_after_trees.size() : 0;

    size_t checkpoint = 0;
    size_t next_stop =
        num_checkpoints ? static_cast<size_t>(this->early_exit_after_trees[0])
                        : num_trees;
    double sum = this->base_margin;
    size_t tree = 0;
    for (;;)
    {
        for (; tree < next_stop; tree++)
        {
            const Node* node = all_nodes + this->tree_roots[tree];
            while (node->feature >= 0)
            {
                const double value =
                    static_cast<double>(features[node->feature]);
                const bool go_left =
                    std::isnan(value)
                        ? node->missing_goes_left != 0
                        : (this->split_less_than ? value < node->split
                                                 : value <= node->split);
                node =
                    all_nodes + (go_left ? node->left_node : node->right_node);
            }
            sum += node->split;
        }
        if (tree >= num_trees)
        {
            break;
        }
        if (sum < this->early_exit_below_sum[checkpoint])
        {
            break;
        }
        checkpoint++;
        next_stop =
            checkpoint < num_checkpoints
                ? static_cast<size_t>(this->early_exit_after_trees[checkpoint])
                : num_trees;
    }
    return static_cast<float>(1. / (1. + std::exp(-sum)));
}

inline int Model::expectedFeatureCount() const
{
    // range and ring, a log range difference per neighbour, 8 summaries
    int count = 2 +
                static_cast<int>(
                    this->ring_offsets.size() * this->column_offsets.size()) -
                1 + 8;
    if (!this->time_offsets.empty())
    {
        // 3 differences per earlier scan, agreement count and median; for
        // the scan before also the unstable and gone counts
        count += 3 * static_cast<int>(this->time_offsets.size()) + 2;
        for (const int offset : this->time_offsets)
        {
            if (std::abs(offset) == 1)
            {
                count += 2;
            }
        }
    }
    return count + (this->use_intensity ? 5 : 0);
}



// --- Loading -----------------------------------------------------------------

namespace detail
{
[[noreturn]]
inline void failModel(const std::string& path, const std::string& what)
{
    throw std::runtime_error("model file " + path + ": " + what);
}
}  // namespace detail

inline Model Model::load(const std::string& path)
{
    std::ifstream in(path);
    if (!in)
    {
        throw std::runtime_error("cannot open model file " + path);
    }
    std::string line;
    if (!std::getline(in, line) || line.rfind(FILE_HEADER, 0) != 0)
    {
        detail::failModel(
            path,
            std::string("not a '") + FILE_HEADER +
                "' file (export it with ghost-ml annotator_app)");
    }

    Model model;
    try
    {
        const TreeCounts counts = readSettings(in, model);
        readTrees(in, counts, model);
        deriveGrid(model);
        validate(model);
    }
    catch (const std::runtime_error& error)
    {
        detail::failModel(path, error.what());
    }
    return model;
}

/* The settings, one "<key> <values...>" line each, up to the "trees" line.
 * Unknown keys (sensor_fingerprint, later additions) are skipped. */
inline Model::TreeCounts Model::readSettings(std::istream& in, Model& model)
{
    TreeCounts counts;
    std::string line;
    std::string key;
    while (std::getline(in, line))
    {
        std::istringstream values(line);
        values >> key;
        const auto readList = [&values](auto& list)
        {
            size_t size = 0;
            values >> size;
            list.resize(size);
            for (auto& value : list)
            {
                values >> value;
            }
        };

        if (key == "variant")
        {
            values >> model.variant;
        }
        else if (key == "trained")
        {
            values >> model.trained;
        }
        else if (key == "threshold")
        {
            values >> model.threshold;
        }
        else if (key == "cols")
        {
            values >> model.num_columns;
        }
        else if (key == "layer_elevations_deg")
        {
            readList(model.layer_elevations_deg);
        }
        else if (key == "layer_azimuth0_deg")
        {
            readList(model.layer_first_azimuth_deg);
        }
        else if (key == "scan_rate_hz")
        {
            values >> model.scan_rate_hz;
        }
        else if (key == "use_intensity")
        {
            int flag = 0;
            values >> flag;
            model.use_intensity = flag != 0;
        }
        else if (key == "intensity_field")
        {
            values >> model.intensity_field;
            if (model.intensity_field == "-")
            {
                model.intensity_field.clear();
            }
        }
        else if (key == "intensity_max")
        {
            values >> model.intensity_max;
        }
        else if (key == "min_range_m")
        {
            values >> model.min_range_m;
        }
        else if (key == "ring_offsets")
        {
            readList(model.ring_offsets);
        }
        else if (key == "col_offsets")
        {
            readList(model.column_offsets);
        }
        else if (key == "close_log")
        {
            values >> model.close_log;
        }
        else if (key == "far_log")
        {
            values >> model.far_log;
        }
        else if (key == "clip")
        {
            values >> model.clip;
        }
        else if (key == "max_dt_ns")
        {
            values >> model.max_scan_gap_ns;
        }
        else if (key == "time_offsets")
        {
            readList(model.time_offsets);
        }
        else if (key == "n_features")
        {
            values >> model.num_features;
        }
        else if (key == "compare")
        {
            std::string rule;
            values >> rule;
            model.split_less_than = (rule == "lt");
        }
        else if (key == "base_margin")
        {
            values >> model.base_margin;
        }
        else if (key == "early_exit")
        {
            size_t num_checkpoints = 0;
            values >> num_checkpoints;
            model.early_exit_after_trees.resize(num_checkpoints);
            model.early_exit_below_sum.resize(num_checkpoints);
            for (size_t i = 0; i < num_checkpoints; i++)
            {
                values >> model.early_exit_after_trees[i] >>
                    model.early_exit_below_sum[i];
            }
        }
        else if (key == "trees")
        {
            values >> counts.trees >> counts.nodes;
            break;
        }
    }
    if (!counts.trees || !counts.nodes)
    {
        throw std::runtime_error("no trees");
    }
    return counts;
}

/* "tree <num_nodes>", then one line per node:
 * <feature> <split> <left> <right> <missing_left> <leaf value> */
inline void
    Model::readTrees(std::istream& in, const TreeCounts& counts, Model& model)
{
    model.tree_roots.reserve(counts.trees);
    model.nodes.reserve(counts.nodes);
    for (size_t tree = 0; tree < counts.trees; tree++)
    {
        std::string key;
        size_t tree_nodes = 0;
        if (!(in >> key >> tree_nodes) || key != "tree" || !tree_nodes)
        {
            throw std::runtime_error("truncated (tree header)");
        }
        const int32_t first_node = static_cast<int32_t>(model.nodes.size());
        model.tree_roots.push_back(first_node);

        for (size_t i = 0; i < tree_nodes; i++)
        {
            int feature;
            int left;
            int right;
            int missing_left;
            // as text: a split can be "-inf", which stream extraction
            // does not read
            std::string split_text;
            std::string value_text;
            if (!(in >> feature >> split_text >> left >> right >>
                  missing_left >> value_text))
            {
                throw std::runtime_error("truncated (tree nodes)");
            }
            const bool is_leaf = feature < 0;
            const int num_tree_nodes = static_cast<int>(tree_nodes);
            if (feature >= model.num_features ||
                (!is_leaf && (left < 0 || right < 0 || left >= num_tree_nodes ||
                              right >= num_tree_nodes)))
            {
                throw std::runtime_error("corrupt tree node");
            }

            Node node;
            node.feature = feature;
            node.split = is_leaf ? std::strtod(value_text.c_str(), nullptr)
                                 : std::strtod(split_text.c_str(), nullptr);
            node.left_node = is_leaf ? -1 : first_node + left;
            node.right_node = is_leaf ? -1 : first_node + right;
            node.missing_goes_left = static_cast<uint32_t>(missing_left != 0);
            model.nodes.push_back(node);
        }
    }
}

inline void Model::deriveGrid(Model& model)
{
    model.num_rings = static_cast<int>(model.layer_elevations_deg.size());
    if (model.num_rings < 3 || model.num_columns < 16 ||
        model.layer_first_azimuth_deg.size() !=
            model.layer_elevations_deg.size())
    {
        throw std::runtime_error("incomplete sensor grid");
    }
    model.column_width_deg = 360. / model.num_columns;
    for (const double azimuth : model.layer_first_azimuth_deg)
    {
        model.layer_first_column.push_back(
            static_cast<int>(
                std::floor((azimuth + 180.) / model.column_width_deg)));
    }
}

inline void Model::validate(const Model& model)
{
    const auto& checkpoints = model.early_exit_after_trees;
    for (size_t i = 0; i < checkpoints.size(); i++)
    {
        const int32_t previous = i ? checkpoints[i - 1] : 0;
        if (checkpoints[i] <= previous ||
            static_cast<size_t>(checkpoints[i]) >= model.tree_roots.size())
        {
            throw std::runtime_error("bad early_exit checkpoints");
        }
    }

    for (const int offset : model.time_offsets)
    {
        if (offset >= 0)
        {
            throw std::runtime_error(
                "this model looks at scans after the one it classifies, which "
                "a live filter does not have. Train a realtime model (\"before "
                "only (realtime)\" in the app, or --time-offsets -2 -1) and "
                "export that");
        }
    }

    const auto contains = [](const std::vector<int>& list, int value)
    { return std::find(list.begin(), list.end(), value) != list.end(); };
    if (!contains(model.ring_offsets, 0) ||
        !contains(model.column_offsets, -1) ||
        !contains(model.column_offsets, 1))
    {
        throw std::runtime_error("unsupported neighbourhood offsets");
    }

    if (model.expectedFeatureCount() != model.num_features)
    {
        throw std::runtime_error(
            "expects " + std::to_string(model.num_features) +
            " features but this code produces " +
            std::to_string(model.expectedFeatureCount()) +
            " for its settings: multiscan_driver artifact_filter is out of "
            "step with ghost-ml annotator_app/artifact_features.py");
    }
}

}  // namespace artifact_filter
