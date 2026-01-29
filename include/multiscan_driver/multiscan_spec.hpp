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

#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <cstdint>

#include <Eigen/Core>

namespace ms136
{

constexpr size_t SEGMENTS_PER_FRAME = 12U;
constexpr size_t NUM_LR_LAYERS = 14U;
constexpr size_t NUM_HD_LAYERS = 2U;
constexpr size_t POINTS_PER_SEGMENT_LR_LAYER = 30U;
constexpr size_t POINTS_PER_SEGMENT_HD_LAYER = 240U;
// echos get filterd when we apply different settings in the web dashboard
constexpr size_t MAX_ECHOES_PER_POINT = 3U;

// --- derived ---
constexpr size_t NUM_LAYERS = (NUM_LR_LAYERS + NUM_HD_LAYERS);
constexpr size_t MAX_POINTS_PER_SEGMENT_ECHO =
    (NUM_LR_LAYERS * POINTS_PER_SEGMENT_LR_LAYER +
     NUM_HD_LAYERS * POINTS_PER_SEGMENT_HD_LAYER);
constexpr size_t MAX_POINTS_PER_SCAN =
    (SEGMENTS_PER_FRAME * MAX_POINTS_PER_SEGMENT_ECHO);
constexpr size_t POINTS_PER_LR_LAYER =
    (SEGMENTS_PER_FRAME * POINTS_PER_SEGMENT_LR_LAYER);
constexpr size_t POINTS_PER_LR_SCAN = (POINTS_PER_LR_LAYER * NUM_LR_LAYERS);


inline constexpr bool isHdLayer(size_t raw_layer_idx)
{
    return raw_layer_idx == 5 || raw_layer_idx == 13;
}


namespace redux
{

// --- dense mapping ---
/* MAGIC NUMBERS TABLE!
Layer,  DLayer, Elevation (rad),    PhaseL​ (rad),  PhaseL​ (deg)
0,      0,      -0.390975,          2.961626,       169.69°
1,      1,      -0.303366,          2.774730,       158.98°
2,      2,      -0.218469,          2.715218,       155.57°
3,      3,      -0.132131,          3.222567,       184.64°
4,      4,      -0.048453,          2.724633,       156.11°
6,      5,      0.034470,           2.785353,       159.59°
7,      6,      0.117062,           2.958301,       169.50°
8,      7,      0.219243,           6.099781,       349.50°
9,      8,      0.294787,           5.912278,       338.75°
10,     9,      0.375273,           5.841743,       334.71°
11,     10,     0.458180,           0.103522,       5.93°
12,     11,     0.544040,           5.817581,       333.32°
14,     12,     0.634508,           5.873127,       336.51°
15,     13,     0.731123,           6.094876,       349.21°
*/

constexpr uint16_t RANGE_MASK = 0x7FFF;
constexpr uint16_t REFLECTOR_MASK = 0x8000;
constexpr uint16_t INDEX_MARKER_MASK = 0xC000;

constexpr std::array<size_t, NUM_LAYERS>
    DENSE_LAYER_IDX_LUT{0, 1, 2, 3, 4, 14, 5, 6, 7, 8, 9, 10, 11, 14, 12, 13};

constexpr std::array<float, NUM_LR_LAYERS> ELEVATION_LUT{
    -0.390975,
    -0.303366,
    -0.218469,
    -0.132131,
    -0.048453,
    0.034470,
    0.117062,
    0.219243,
    0.294787,
    0.375273,
    0.458180,
    0.544040,
    0.634508,
    0.731123};

constexpr std::array<float, NUM_LR_LAYERS> AZIMUTH_OFFSET_LUT{
    2.961626,
    2.774730,
    2.715218,
    3.222567,
    2.724633,
    2.785353,
    2.958301,
    6.099781,
    5.912278,
    5.841743,
    0.103522,
    5.817581,
    5.873127,
    6.094876};

using DenseBuffer = std::array<uint16_t, POINTS_PER_LR_SCAN>;
using PackedBuffer = std::vector<uint16_t>;


/* Quantize a range value to 15 bits (mm-precision) and append a reflector bit */
inline constexpr uint16_t
    reducePoint(float range, uint8_t reflector, float min_range_mm = 200.f)
{
    const float range_f_mm = range * 1000.f;
    if (range_f_mm < min_range_mm ||
        range_f_mm > static_cast<float>(RANGE_MASK))
    {
        return 0;
    }
    else
    {
        return (static_cast<uint16_t>(reflector) << 15) |
               (static_cast<uint16_t>(range_f_mm) & RANGE_MASK);
    }
}

/* Extract range bits from packed point */
inline constexpr uint16_t getRangeMMeters(uint16_t packed)
{
    return packed & RANGE_MASK;
}
/* Extract range bits from packed point and convert to meters */
inline constexpr float getRangeMeters(uint16_t packed)
{
    return static_cast<float>(getRangeMMeters(packed)) / 1000.f;
}
/* Extract reflector bit from packed representation */
inline constexpr bool getReflector(uint16_t packed)
{
    return packed & REFLECTOR_MASK;
}

/* Compute global dense-idx from layer, segment, and segment-line indices */
inline constexpr size_t computeDenseIdx(
    size_t raw_layer_idx,
    size_t layer_pt_idx)
{
    return (DENSE_LAYER_IDX_LUT[raw_layer_idx] * POINTS_PER_LR_LAYER) +
           layer_pt_idx;
}

/* Look up the elevation (phi) and azimuth (theta) spherical coordinates given global dense idx */
inline Eigen::Vector2f computeDirection(size_t dense_i)
{
    assert(dense_i < POINTS_PER_LR_SCAN);
    const size_t layer_i = dense_i / POINTS_PER_LR_LAYER;
    const size_t line_i = dense_i - (layer_i * POINTS_PER_LR_LAYER);
    return Eigen::Vector2f{
        ELEVATION_LUT[layer_i],
        AZIMUTH_OFFSET_LUT[layer_i] + static_cast<float>(line_i) * 0.0174533f};
}

/* Project a dense point given it's range */
inline Eigen::Vector3f projectPoint(size_t dense_i, float range)
{
    const Eigen::Vector2f dir = computeDirection(dense_i);

#define PHI   dir.x()
#define THETA dir.y()
    const float sin_phi = std::sin(PHI);
    const float cos_phi = std::cos(PHI);
    const float sin_theta = std::sin(THETA);
    const float cos_theta = std::cos(THETA);
#undef PHI
#undef THETA

    return Eigen::Vector3f{
        range * cos_phi * cos_theta,
        range * cos_phi * sin_theta,
        range * sin_phi};
}

inline Eigen::Vector3f projectPoint(size_t dense_i, uint16_t dense_pt)
{
    return projectPoint(dense_i, getRangeMeters(dense_pt));
}

/* Insert raw point into buffer using calculators */
inline constexpr bool addPointToBuffer(
    DenseBuffer& buff,
    size_t raw_layer_idx,
    size_t layer_pt_idx,
    float range,
    uint8_t reflector)
{
    if (!isHdLayer(raw_layer_idx))
    {
        buff[computeDenseIdx(raw_layer_idx, layer_pt_idx)] =
            reducePoint(range, reflector);
        return true;
    }
    return false;
}

/* Condense buffer to be more packed by skipping sections of null data */
inline void packBuffer(PackedBuffer& packed, const DenseBuffer& dense)
{
    packed.clear();
    packed.reserve(dense.size());

    for (size_t i = 0; i < dense.size(); i++)
    {
        if (!(dense[i] & RANGE_MASK))
        {
            while (i < dense.size() && !(dense[i] & RANGE_MASK))
            {
                i++;
            }
            if (i < dense.size())
            {
                packed.push_back(INDEX_MARKER_MASK | static_cast<uint16_t>(i));
                // assert(
                //     i < dense.size() &&
                //     i == static_cast<size_t>(
                //              packed.back() & ~INDEX_MARKER_MASK));
                i--;
            }
        }
        else
        {
            // Erase reflector bit if last range bit is used (> ~16 meters) such
            // as to keep index marker unique to marker packets.
            // This doesn't degrade usability since reflectors at this distance are
            // not likely accurate or useful.
            packed.push_back(dense[i] & ~((dense[i] & INDEX_MARKER_MASK) << 1));
            // assert(
            //     (packed.back() & 0x7000) || (packed.back() & REFLECTOR_MASK) ==
            //                                     (dense[i] & REFLECTOR_MASK));
            // assert((packed.back() & INDEX_MARKER_MASK) != INDEX_MARKER_MASK);
        }
    }
}

inline void unpackBuffer(DenseBuffer& dense, const PackedBuffer& packed)
{
    size_t dense_i = 0;
    for (const uint16_t p : packed)
    {
        if ((p & INDEX_MARKER_MASK) == INDEX_MARKER_MASK)
        {
            const size_t new_dense_i =
                static_cast<size_t>(p & ~INDEX_MARKER_MASK);
            // assert(new_dense_i < dense.size());
            for (; dense_i < new_dense_i; dense_i++)
            {
                dense[dense_i] = 0;
            }
        }
        else
        {
            // assert(dense_i < dense.size());
            dense[dense_i] = p;
            dense_i++;
        }
    }
    for (; dense_i < dense.size(); dense_i++)
    {
        dense[dense_i] = 0;
    }
}

};  // namespace redux
};  // namespace ms136
