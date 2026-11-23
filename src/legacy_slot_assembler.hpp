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

/*  The driver's original frame assembly: one slot per segment index, the
 *  newest segment wins, and a frame is emitted once all 12 slots are filled.
 *  Kept only so the new decoder can be verified against the old driver end to
 *  end; it is replaced by the frame-number based assembler next (it never
 *  publishes when the sensor omits segments, see test_legacy_oracle.cpp).
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>


namespace multiscan
{

class LegacySlotAssembler
{
public:
    static constexpr size_t SEGMENTS = 12;

    /* Buffer to decode segment `index` into (room for `capacity` points). */
    uint8_t* slotBuffer(size_t index, size_t capacity, size_t point_step)
    {
        Slot& s = slots[index];
        s.data.resize(capacity * point_step);
        return s.data.data();
    }

    /* Commit a decoded segment. Returns true when a frame is complete. */
    bool commit(size_t index, size_t points, uint64_t stamp_ns)
    {
        Slot& s = slots[index];
        s.points = points;
        s.stamp_ns = stamp_ns;
        fill_mask |= (1u << index);
        return fill_mask == (1u << SEGMENTS) - 1;
    }

    /* Concatenate the slots in segment order into `out` and reset. Returns
     * the earliest segment stamp. */
    uint64_t take(std::vector<uint8_t>& out, size_t point_step)
    {
        size_t total = 0;
        for (const Slot& s : slots) total += s.points;
        out.resize(total * point_step);
        uint64_t earliest = std::numeric_limits<uint64_t>::max();
        size_t off = 0;
        for (const Slot& s : slots)
        {
            std::memcpy(out.data() + off, s.data.data(), s.points * point_step);
            off += s.points * point_step;
            earliest = std::min(earliest, s.stamp_ns);
        }
        fill_mask = 0;
        return earliest;
    }

private:
    struct Slot
    {
        std::vector<uint8_t> data;
        size_t points = 0;
        uint64_t stamp_ns = 0;
    };
    std::array<Slot, SEGMENTS> slots;
    uint32_t fill_mask = 0;
};

}  // namespace multiscan
