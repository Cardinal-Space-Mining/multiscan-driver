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
constexpr size_t N_FIELDS = static_cast<size_t>(Field::COUNT);

struct FieldInfo
{
    const char* name;
    uint32_t bytes;
    uint8_t datatype;
};

constexpr std::array<FieldInfo, N_FIELDS> FIELD_INFO = {
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
     }
};
}  // namespace


PointLayout::PointLayout(const std::array<bool, N_FIELDS>& enabled)
{
    offsets.fill(ABSENT);
    uint32_t off = 0;
    for (size_t i = 0; i < N_FIELDS; i++)
    {
        if (!enabled[i])
        {
            continue;
        }
        offsets[i] = static_cast<int32_t>(off);
        if (static_cast<Field>(i) == Field::Time)
        {
            descriptors.push_back({"tl", off, POINT_FIELD_UINT32});
            descriptors.push_back({"th", off + 4, POINT_FIELD_UINT32});
        }
        else
        {
            descriptors.push_back(
                {FIELD_INFO[i].name, off, FIELD_INFO[i].datatype});
        }
        off += FIELD_INFO[i].bytes;
    }
    point_step = off;
}

PointLayout PointLayout::all()
{
    std::array<bool, N_FIELDS> e;
    e.fill(true);
    return PointLayout{e};
}

std::optional<PointLayout> PointLayout::fromNames(
    const std::vector<std::string>& names,
    std::string& error)
{
    std::array<bool, N_FIELDS> e{};
    for (const std::string& n : names)
    {
        const std::string name =
            (n == "tl" || n == "th") ? std::string("time") : n;
        bool found = false;
        for (size_t i = 0; i < N_FIELDS; i++)
        {
            if (name == FIELD_INFO[i].name)
            {
                e[i] = found = true;
                break;
            }
        }
        if (!found)
        {
            error = "unknown point field '" + n + "'";
            return std::nullopt;
        }
    }
    if (!e[0] || !e[1] || !e[2])
    {
        error = "point fields must include x, y and z";
        return std::nullopt;
    }
    return PointLayout{e};
}

}  // namespace multiscan
