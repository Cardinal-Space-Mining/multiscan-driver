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

/*  Decoder for the multiScan136 "Compact" UDP format (telegram type 1, version
 *  4: scan data; telegram type 2, version 1: IMU), as specified in the
 *  multiScan136 operating instructions (8027119), section 12.4.1.4.
 *
 *  A scan telegram is validated completely (start sequence, CRC32, module
 *  chain and module sizes) before any point is written, then decoded in one
 *  pass straight into a caller-provided point buffer using a PointLayout.
 *
 *  Output order and values are those multiscan_driver has always published:
 *  one point per beam and layer (beams without a return included), layers in
 *  ascending layer id, beams in ascending order. test/support/expected_points.hpp
 *  states the exact semantics and the tests hold this decoder to them.
 */
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "point_layout.hpp"


namespace multiscan::compact
{

enum class TelegramType : uint32_t
{
    Unknown = 0,
    Scan = 1,
    Imu = 2,
};

enum class Error
{
    None,
    TooShort,
    BadStartSequence,
    UnknownTelegramType,
    BadCrc,
    UnsupportedVersion,
    BadModuleChain,   // module sizes don't add up to the datagram size
    BadModuleHeader,  // implausible lines/beams/echoes counts
    BadModuleSize,    // module size doesn't match its metadata
    BufferTooSmall,
};
const char* toString(Error e);

constexpr size_t SCAN_HEADER_BYTES = 32;
constexpr size_t IMU_TELEGRAM_BYTES = 64;
constexpr uint32_t SCAN_TELEGRAM_VERSION = 4;
constexpr uint32_t IMU_TELEGRAM_VERSION = 1;
constexpr uint32_t NUM_LAYERS = 16;
constexpr uint32_t MAX_MODULES = 8;

struct ScanInfo
{
    uint64_t telegram_counter = 0;
    uint64_t transmit_time_us = 0;  // sensor clock
    uint64_t segment = 0;
    uint64_t frame = 0;
    uint32_t sender_id = 0;
    uint32_t points = 0;  // points written (or required, on BufferTooSmall)
    uint64_t start_time_us =
        0;  // first beam of the lowest layer id (sensor clock)
    uint64_t time_min_us = 0;  // min/max point time (sensor clock)
    uint64_t time_max_us = 0;
};

struct ImuSample
{
    std::array<float, 3> acceleration{};      // m/s^2, gravity included
    std::array<float, 3> angular_velocity{};  // rad/s
    std::array<float, 4> orientation{};       // quaternion w, x, y, z
    uint64_t time_us = 0;                     // sensor clock
};

struct DecodeOptions
{
    /* 1-based echo to publish; falls back to the first echo when the
     * telegram carries fewer. */
    uint32_t echo_selection = 1;
};

/* Telegram type from the 8-byte prefix, without further validation. */
TelegramType telegramType(std::span<const uint8_t> datagram);

Error decodeImu(std::span<const uint8_t> datagram, ImuSample& out);

class ScanDecoder
{
public:
    ScanDecoder(const PointLayout& layout, DecodeOptions options = {});

    /* Validate a scan telegram and write its points to `out`, which must
     * have room for `capacity` points of layout().pointStep() bytes each.
     * On BufferTooSmall, info.points holds the number of points required. */
    Error decode(
        std::span<const uint8_t> datagram,
        uint8_t* out,
        size_t capacity,
        ScanInfo& info);

    const PointLayout& layout() const { return point_layout; }
    const DecodeOptions& options() const { return opts; }

private:
    struct ModuleView
    {
        const uint8_t* base;
        uint32_t lines, beams, echoes;
        uint8_t content_echoes, content_beams;
        float distance_scale;
        uint32_t tuple_bytes;
        const uint8_t* data;
    };

    struct LineShape
    {
        uint32_t layer_id;
        uint32_t out_offset;  // first output point of this line
        uint32_t
            index_stride;  // points per segment in this layer (index = segment * stride + beam)
        float elevation, sin_el, cos_el;
    };

    // Per-line geometry depends only on the modules' shape and elevations,
    // which only change when the sensor is reconfigured, so it's cached.
    struct ShapeKey
    {
        std::array<uint32_t, MAX_MODULES * 3> dims{};
        std::array<uint32_t, MAX_MODULES * NUM_LAYERS> phi_bits{};
        bool operator==(const ShapeKey&) const = default;
    };

    void updateShape(const ModuleView* modules, size_t n_modules);

    PointLayout point_layout;
    DecodeOptions opts;
    ShapeKey shape_key;
    bool shape_valid = false;
    std::vector<std::vector<LineShape>> shape;  // [module][line]
    uint32_t shape_points = 0;
    uint32_t shape_first_module = 0,
             shape_first_line = 0;  // line holding the lowest layer id
};

}  // namespace multiscan::compact
