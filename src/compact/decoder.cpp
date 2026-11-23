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

#include "decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>

#include <zlib.h>


namespace multiscan::compact
{

namespace
{

template<typename T> inline T load(const uint8_t* p)
{
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

template<typename T> inline void store(uint8_t* p, int32_t offset, T v)
{
    if (offset >= 0)
    {
        std::memcpy(p + offset, &v, sizeof(T));
    }
}

inline bool hasStartSequence(std::span<const uint8_t> b)
{
    return b.size() >= 4 && b[0] == 0x02 && b[1] == 0x02 && b[2] == 0x02 &&
           b[3] == 0x02;
}

inline bool crcValid(std::span<const uint8_t> b)
{
    const uint32_t stored = load<uint32_t>(b.data() + b.size() - 4);
    return stored ==
           static_cast<uint32_t>(
               ::crc32(0L, b.data(), static_cast<uInt>(b.size() - 4)));
}

/* Elevation of the 16 layers (mdeg, sign of phi) used to assign layer ids,
 * nearest match. Layer ids are what consumers (net_adapter codec) key on. */
constexpr std::array<int, NUM_LAYERS> LAYER_ELEVATION_MDEG = {
    22710,
    17560,
    12480,
    7510,
    2490,
    70,
    -2430,
    -7290,
    -12790,
    -17280,
    -21940,
    -26730,
    -31860,
    -34420,
    -37180,
    -42790};

uint32_t layerIdFromPhi(float phi_rad)
{
    const int mdeg = static_cast<int>(
        std::lround(phi_rad * 180000 / 3.14159265358979323846));
    uint32_t best = 0;
    for (uint32_t i = 1; i < NUM_LAYERS; i++)
    {
        if (std::abs(mdeg - LAYER_ELEVATION_MDEG[i]) <
            std::abs(mdeg - LAYER_ELEVATION_MDEG[best]))
        {
            best = i;
        }
    }
    return best;
}

// module metadata layout (telegram version 4), relative to the module start
constexpr size_t M_SEGMENT = 0, M_FRAME = 8, M_SENDER = 16, M_LINES = 20,
                 M_BEAMS = 24, M_ECHOES = 28, M_ARRAYS = 32;
inline size_t metadataBytes(uint32_t lines)
{
    return M_ARRAYS + lines * 28 + 12;
}

constexpr uint8_t ECHO_HAS_DISTANCE = 0x01, ECHO_HAS_RSSI = 0x02;
constexpr uint8_t BEAM_HAS_PROPERTIES = 0x01, BEAM_HAS_AZIMUTH = 0x02;

}  // namespace


const char* toString(Error e)
{
    switch (e)
    {
        case Error::None:
            return "ok";
        case Error::TooShort:
            return "datagram too short";
        case Error::BadStartSequence:
            return "missing start sequence";
        case Error::UnknownTelegramType:
            return "unknown telegram type";
        case Error::BadCrc:
            return "CRC mismatch";
        case Error::UnsupportedVersion:
            return "unsupported telegram version";
        case Error::BadModuleChain:
            return "module sizes don't match datagram size";
        case Error::BadModuleHeader:
            return "implausible module header";
        case Error::BadModuleSize:
            return "module size doesn't match its metadata";
        case Error::BufferTooSmall:
            return "output buffer too small";
    }
    return "?";
}

TelegramType telegramType(std::span<const uint8_t> b)
{
    if (b.size() < 8 || !hasStartSequence(b))
    {
        return TelegramType::Unknown;
    }
    const uint32_t t = load<uint32_t>(b.data() + 4);
    return (t == 1 || t == 2) ? static_cast<TelegramType>(t)
                              : TelegramType::Unknown;
}

Error decodeImu(std::span<const uint8_t> b, ImuSample& out)
{
    if (b.size() < 8)
    {
        return Error::TooShort;
    }
    if (!hasStartSequence(b))
    {
        return Error::BadStartSequence;
    }
    if (load<uint32_t>(b.data() + 4) !=
        static_cast<uint32_t>(TelegramType::Imu))
    {
        return Error::UnknownTelegramType;
    }
    if (b.size() != IMU_TELEGRAM_BYTES)
    {
        return Error::BadModuleChain;
    }
    if (!crcValid(b))
    {
        return Error::BadCrc;
    }
    if (load<uint32_t>(b.data() + 8) != IMU_TELEGRAM_VERSION)
    {
        return Error::UnsupportedVersion;
    }
    const uint8_t* p = b.data() + 12;
    for (float& v : out.acceleration)
    {
        v = load<float>(p);
        p += 4;
    }
    for (float& v : out.angular_velocity)
    {
        v = load<float>(p);
        p += 4;
    }
    for (float& v : out.orientation)
    {
        v = load<float>(p);
        p += 4;
    }
    out.time_us = load<uint64_t>(p);
    return Error::None;
}


ScanDecoder::ScanDecoder(const PointLayout& layout, DecodeOptions options) :
    point_layout{layout},
    opts{options}
{
}

void ScanDecoder::updateShape(const ModuleView* modules, size_t n_modules)
{
    ShapeKey key;
    for (size_t m = 0; m < n_modules; m++)
    {
        key.dims[m * 3 + 0] = modules[m].lines;
        key.dims[m * 3 + 1] = modules[m].beams;
        key.dims[m * 3 + 2] = modules[m].echoes;
        const uint8_t* phi = modules[m].base + M_ARRAYS + modules[m].lines * 16;
        std::memcpy(&key.phi_bits[m * NUM_LAYERS], phi, modules[m].lines * 4);
    }
    if (shape_valid && key == shape_key)
    {
        return;
    }

    struct Ref
    {
        uint32_t module, line, layer, beams;
    };
    std::vector<Ref> refs;
    shape.assign(n_modules, {});
    for (uint32_t m = 0; m < n_modules; m++)
    {
        const ModuleView& mv = modules[m];
        const uint8_t* phi = mv.base + M_ARRAYS + mv.lines * 16;
        shape[m].resize(mv.lines);
        for (uint32_t l = 0; l < mv.lines; l++)
        {
            const float p = load<float>(phi + l * 4);
            LineShape& ls = shape[m][l];
            ls.layer_id = layerIdFromPhi(p);
            ls.elevation =
                -p;  // DIN ISO 8855 phi is positive below the horizon
            ls.sin_el = std::sin(ls.elevation);
            ls.cos_el = std::cos(ls.elevation);
            refs.push_back({m, l, ls.layer_id, mv.beams});
        }
    }
    // layers in ascending id; lines sharing an id stay in packet order
    std::stable_sort(
        refs.begin(),
        refs.end(),
        [](const Ref& a, const Ref& b) { return a.layer < b.layer; });
    std::array<uint32_t, NUM_LAYERS> layer_points{};
    for (const Ref& r : refs)
    {
        layer_points[r.layer] += r.beams;
    }
    uint32_t off = 0;
    for (const Ref& r : refs)
    {
        LineShape& ls = shape[r.module][r.line];
        ls.out_offset = off;
        ls.index_stride = layer_points[r.layer];
        off += r.beams;
    }
    shape_points = off;
    shape_first_module = refs.empty() ? 0 : refs.front().module;
    shape_first_line = refs.empty() ? 0 : refs.front().line;
    shape_key = key;
    shape_valid = true;
}

Error ScanDecoder::decode(
    std::span<const uint8_t> b,
    uint8_t* out,
    size_t capacity,
    ScanInfo& info)
{
    // --- framing -------------------------------------------------------------
    if (b.size() < SCAN_HEADER_BYTES + 4)
    {
        return Error::TooShort;
    }
    if (!hasStartSequence(b))
    {
        return Error::BadStartSequence;
    }
    if (load<uint32_t>(b.data() + 4) !=
        static_cast<uint32_t>(TelegramType::Scan))
    {
        return Error::UnknownTelegramType;
    }
    if (!crcValid(b))
    {
        return Error::BadCrc;
    }
    if (load<uint32_t>(b.data() + 24) != SCAN_TELEGRAM_VERSION)
    {
        return Error::UnsupportedVersion;
    }

    info = ScanInfo{};
    info.telegram_counter = load<uint64_t>(b.data() + 8);
    info.transmit_time_us = load<uint64_t>(b.data() + 16);

    // --- module chain ----------------------------------------------------------
    std::array<ModuleView, MAX_MODULES> modules;
    size_t n_modules = 0;
    const size_t end = b.size() - 4;
    size_t off = SCAN_HEADER_BYTES;
    uint64_t module_size = load<uint32_t>(b.data() + 28);
    while (module_size)
    {
        if (n_modules == MAX_MODULES || off + module_size > end ||
            module_size < M_ARRAYS)
        {
            return Error::BadModuleChain;
        }
        ModuleView& mv = modules[n_modules++];
        mv.base = b.data() + off;
        mv.lines = load<uint32_t>(mv.base + M_LINES);
        mv.beams = load<uint32_t>(mv.base + M_BEAMS);
        mv.echoes = load<uint32_t>(mv.base + M_ECHOES);
        if (mv.lines < 1 || mv.lines > NUM_LAYERS || mv.beams < 1 ||
            mv.beams > 4096 || mv.echoes < 1 || mv.echoes > 3)
        {
            return Error::BadModuleHeader;
        }
        const size_t meta = metadataBytes(mv.lines);
        if (module_size < meta)
        {
            return Error::BadModuleSize;
        }
        const uint8_t* tail = mv.base + M_ARRAYS + mv.lines * 28;
        mv.distance_scale = load<float>(tail);
        const uint32_t next = load<uint32_t>(tail + 4);
        mv.content_echoes = tail[9];
        mv.content_beams = tail[10];
        const uint32_t per_echo =
            ((mv.content_echoes & ECHO_HAS_DISTANCE) ? 2u : 0u) +
            ((mv.content_echoes & ECHO_HAS_RSSI) ? 2u : 0u);
        mv.tuple_bytes = per_echo * mv.echoes +
                         ((mv.content_beams & BEAM_HAS_PROPERTIES) ? 1u : 0u) +
                         ((mv.content_beams & BEAM_HAS_AZIMUTH) ? 2u : 0u);
        if (meta +
                static_cast<uint64_t>(mv.tuple_bytes) * mv.lines * mv.beams !=
            module_size)
        {
            return Error::BadModuleSize;
        }
        mv.data = mv.base + meta;
        off += module_size;
        module_size = next;
    }
    if (off != end || n_modules == 0)
    {
        return Error::BadModuleChain;
    }

    info.segment = load<uint64_t>(modules[0].base + M_SEGMENT);
    info.frame = load<uint64_t>(modules[0].base + M_FRAME);
    info.sender_id = load<uint32_t>(modules[0].base + M_SENDER);

    updateShape(modules.data(), n_modules);
    info.points = shape_points;
    {
        const ModuleView& fm = modules[shape_first_module];
        info.start_time_us =
            load<uint64_t>(fm.base + M_ARRAYS + shape_first_line * 8);
    }
    if (capacity < shape_points)
    {
        return Error::BufferTooSmall;
    }

    // --- points ----------------------------------------------------------------
    const PointLayout& L = point_layout;
    const uint32_t step = L.pointStep();
    const int32_t o_x = L.offset(Field::X), o_y = L.offset(Field::Y),
                  o_z = L.offset(Field::Z), o_int = L.offset(Field::Intensity),
                  o_rng = L.offset(Field::Range),
                  o_az = L.offset(Field::Azimuth),
                  o_el = L.offset(Field::Elevation),
                  o_layer = L.offset(Field::Layer),
                  o_echo = L.offset(Field::Echo),
                  o_idx = L.offset(Field::Index), o_t = L.offset(Field::Time),
                  o_refl = L.offset(Field::Reflective);

    uint64_t t_min = std::numeric_limits<uint64_t>::max(), t_max = 0;
    for (size_t m = 0; m < n_modules; m++)
    {
        const ModuleView& mv = modules[m];
        const std::vector<LineShape>& lines = shape[m];
        const uint8_t* ts_start = mv.base + M_ARRAYS;
        const uint8_t* ts_stop = ts_start + mv.lines * 8;
        const uint8_t* theta_start =
            ts_stop + mv.lines * 12;  // after time stop and phi
        const uint8_t* theta_stop = theta_start + mv.lines * 4;

        const uint32_t echo =
            (opts.echo_selection >= 1 && opts.echo_selection <= mv.echoes)
                ? opts.echo_selection - 1
                : 0;
        const bool has_dist = mv.content_echoes & ECHO_HAS_DISTANCE;
        const bool has_rssi = mv.content_echoes & ECHO_HAS_RSSI;
        const bool has_prop = mv.content_beams & BEAM_HAS_PROPERTIES;
        const bool has_az = mv.content_beams & BEAM_HAS_AZIMUTH;
        const uint32_t per_echo = (has_dist ? 2u : 0u) + (has_rssi ? 2u : 0u);
        const uint32_t dist_off = per_echo * echo;
        const uint32_t rssi_off = dist_off + (has_dist ? 2u : 0u);
        const uint32_t prop_off = per_echo * mv.echoes;
        const uint32_t az_off = prop_off + (has_prop ? 1u : 0u);
        const float beams_m1 =
            static_cast<float>(std::max(1, static_cast<int>(mv.beams) - 1));
        const uint64_t beams_m1_u = std::max<uint64_t>(1, mv.beams - 1);

        std::array<float, NUM_LAYERS> az_start{}, az_delta{};
        std::array<uint64_t, NUM_LAYERS> t0{}, dt{};
        for (uint32_t l = 0; l < mv.lines; l++)
        {
            az_start[l] = load<float>(theta_start + l * 4);
            az_delta[l] =
                (load<float>(theta_stop + l * 4) - az_start[l]) / beams_m1;
            t0[l] = load<uint64_t>(ts_start + l * 8);
            dt[l] = load<uint64_t>(ts_stop + l * 8) - t0[l];
            t_min = std::min({t_min, t0[l], t0[l] + dt[l]});
            t_max = std::max({t_max, t0[l], t0[l] + dt[l]});
        }

        const uint8_t* tuple = mv.data;
        for (uint32_t beam = 0; beam < mv.beams; beam++)
        {
            for (uint32_t l = 0; l < mv.lines; l++, tuple += mv.tuple_bytes)
            {
                const LineShape& ls = lines[l];
                const float range =
                    has_dist ? (mv.distance_scale *
                                static_cast<float>(
                                    load<uint16_t>(tuple + dist_off))) /
                                   1000.0f
                             : 0.f;
                const float azimuth =
                    has_az
                        ? (static_cast<float>(load<uint16_t>(tuple + az_off)) -
                           16384.0f) /
                              5215.0f
                        : az_start[l] + static_cast<float>(beam) * az_delta[l];
                const float sin_az = std::sin(azimuth);
                const float cos_az = std::cos(azimuth);

                uint8_t* p =
                    out + static_cast<size_t>(ls.out_offset + beam) * step;
                store<float>(p, o_x, range * cos_az * ls.cos_el);
                store<float>(p, o_y, range * sin_az * ls.cos_el);
                store<float>(p, o_z, range * ls.sin_el);
                store<float>(
                    p,
                    o_int,
                    has_rssi
                        ? static_cast<float>(load<uint16_t>(tuple + rssi_off))
                        : 0.f);
                store<float>(p, o_rng, range);
                store<float>(p, o_az, azimuth);
                store<float>(p, o_el, ls.elevation);
                store<uint32_t>(p, o_layer, ls.layer_id);
                store<uint32_t>(p, o_echo, echo);
                store<uint32_t>(
                    p,
                    o_idx,
                    static_cast<uint32_t>(info.segment) * ls.index_stride +
                        beam);
                store<uint64_t>(p, o_t, (beam * dt[l]) / beams_m1_u + t0[l]);
                store<float>(
                    p,
                    o_refl,
                    has_prop ? static_cast<float>(tuple[prop_off] & 0x01)
                             : 0.f);
            }
        }
    }
    info.time_min_us = t_min;
    info.time_max_us = t_max;
    return Error::None;
}

}  // namespace multiscan::compact
