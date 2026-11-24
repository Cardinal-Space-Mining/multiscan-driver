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

#include "point_layout.hpp"


namespace multiscan
{

namespace
{

constexpr size_t NUM_FIELDS = static_cast<size_t>(Field::COUNT);

struct FieldFormat
{
    const char* name;
    uint32_t size_bytes;
    uint8_t datatype;
};

// in the order of Field
constexpr std::array<FieldFormat, NUM_FIELDS> FIELD_FORMATS = {
    {
     {"x", 4, POINT_FIELD_FLOAT32},
     {"y", 4, POINT_FIELD_FLOAT32},
     {"z", 4, POINT_FIELD_FLOAT32},
     {"intensity", 4, POINT_FIELD_FLOAT32},
     {"range", 4, POINT_FIELD_FLOAT32},
     {"azimuth", 4, POINT_FIELD_FLOAT32},
     {"elevation", 4, POINT_FIELD_FLOAT32},
     {"layer", 4, POINT_FIELD_UINT32},
     {"echo", 4, POINT_FIELD_UINT32},
     {"index", 4, POINT_FIELD_UINT32},
     {"time", 8, POINT_FIELD_UINT32},  // published as "tl" + "th"
        {"reflective", 4, POINT_FIELD_FLOAT32},
     {"artifact", 4, POINT_FIELD_FLOAT32},
     }
};

}  // namespace


PointLayout::PointLayout(const FieldFlags& enabled)
{
    this->field_offsets.fill(ABSENT);
    uint32_t offset = 0;
    for (size_t i = 0; i < NUM_FIELDS; i++)
    {
        if (!enabled[i])
        {
            continue;
        }
        this->field_offsets[i] = static_cast<int32_t>(offset);
        if (static_cast<Field>(i) == Field::Time)
        {
            this->field_descriptors.push_back(
                {"tl", offset, POINT_FIELD_UINT32});
            this->field_descriptors.push_back(
                {"th", offset + 4, POINT_FIELD_UINT32});
        }
        else
        {
            this->field_descriptors.push_back(
                {FIELD_FORMATS[i].name, offset, FIELD_FORMATS[i].datatype});
        }
        offset += FIELD_FORMATS[i].size_bytes;
    }
    this->point_step = offset;
}

PointLayout PointLayout::all()
{
    FieldFlags enabled;
    enabled.fill(true);
    enabled[static_cast<size_t>(Field::Artifact)] = false;
    return PointLayout{enabled};
}

std::optional<PointLayout> PointLayout::fromNames(
    const std::vector<std::string>& names,
    std::string& error)
{
    FieldFlags enabled{};
    for (const std::string& requested : names)
    {
        const std::string name = (requested == "tl" || requested == "th")
                                     ? std::string("time")
                                     : requested;
        bool found = false;
        for (size_t i = 0; i < NUM_FIELDS; i++)
        {
            if (name == FIELD_FORMATS[i].name)
            {
                enabled[i] = found = true;
                break;
            }
        }
        if (!found)
        {
            error = "unknown point field '" + requested + "'";
            return std::nullopt;
        }
    }
    if (!enabled[static_cast<size_t>(Field::X)] ||
        !enabled[static_cast<size_t>(Field::Y)] ||
        !enabled[static_cast<size_t>(Field::Z)])
    {
        error = "point fields must include x, y and z";
        return std::nullopt;
    }
    return PointLayout{enabled};
}

}  // namespace multiscan
