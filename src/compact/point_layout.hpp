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
#include <cstdint>
#include <optional>
#include <string>
#include <vector>


namespace multiscan
{

/* Point fields the driver can publish. Selected fields are always packed in
 * this order, which keeps every subset compatible with the layouts the
 * compile-time MS_POINT_FIELD_* options used to produce. */
enum class Field : uint8_t
{
    X,
    Y,
    Z,
    Intensity,  // RSSI of the published echo, float32
    Range,      // m, float32
    Azimuth,    // rad, float32
    Elevation,  // rad, float32
    Layer,      // 0..15, uint32
    Echo,       // 0..2, uint32
    Index,      // beam index within the layer over a full revolution, uint32
    Time,  // us, uint64 published as uint32 fields "tl" (low) and "th" (high)
    Reflective,  // 0 or 1, float32
    COUNT
};

/* sensor_msgs/PointField datatype codes (kept here so this stays ROS free) */
constexpr uint8_t POINT_FIELD_UINT32 = 6;
constexpr uint8_t POINT_FIELD_FLOAT32 = 7;

struct FieldDescriptor
{
    std::string name;
    uint32_t offset;
    uint8_t datatype;
};

class PointLayout
{
public:
    static constexpr int32_t ABSENT = -1;

    /* Every field: x y z intensity range azimuth elevation layer echo index tl th reflective (52 bytes) */
    static PointLayout all();

    /* Parse field names ("x", "y", "z", "intensity", "range", "azimuth",
     * "elevation", "layer", "echo", "index", "tl" or "th" or "time",
     * "reflective"). x, y and z are required; order and duplicates don't
     * matter. Returns nullopt and sets `error` for unknown names. */
    static std::optional<PointLayout> fromNames(
        const std::vector<std::string>& names,
        std::string& error);

    int32_t offset(Field f) const { return offsets[static_cast<size_t>(f)]; }
    bool has(Field f) const { return offset(f) != ABSENT; }
    uint32_t pointStep() const { return point_step; }
    const std::vector<FieldDescriptor>& fields() const { return descriptors; }

private:
    explicit PointLayout(
        const std::array<bool, static_cast<size_t>(Field::COUNT)>& enabled);

    std::array<int32_t, static_cast<size_t>(Field::COUNT)> offsets;
    uint32_t point_step = 0;
    std::vector<FieldDescriptor> descriptors;
};

}  // namespace multiscan
