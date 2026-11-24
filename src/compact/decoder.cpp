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

#include <zlib.h>


namespace multiscan::compact
{

namespace
{

// --- Byte access -------------------------------------------------------------

template<typename T> inline T load(const uint8_t* bytes)
{
    T value;
    std::memcpy(&value, bytes, sizeof(T));
    return value;
}

/* Stores a field of a point unless the layout doesn't have it. */
template<typename T> inline void store(uint8_t* point, int32_t offset, T value)
{
    if (offset >= 0)
    {
        std::memcpy(point + offset, &value, sizeof(T));
    }
}

inline bool hasStartSequence(std::span<const uint8_t> datagram)
{
    return datagram.size() >= 4 && datagram[0] == 0x02 && datagram[1] == 0x02 &&
           datagram[2] == 0x02 && datagram[3] == 0x02;
}

inline bool crcValid(std::span<const uint8_t> datagram)
{
    const size_t covered_bytes = datagram.size() - 4;
    const uint32_t stored = load<uint32_t>(datagram.data() + covered_bytes);
    return stored ==
           static_cast<uint32_t>(
               ::crc32(0L, datagram.data(), static_cast<uInt>(covered_bytes)));
}


// --- Telegram layout ---------------------------------------------------------

// scan telegram header
constexpr size_t HEADER_TYPE_OFFSET = 4;
constexpr size_t HEADER_COUNTER_OFFSET = 8;
constexpr size_t HEADER_TRANSMIT_TIME_OFFSET = 16;
constexpr size_t HEADER_VERSION_OFFSET = 24;
constexpr size_t HEADER_FIRST_MODULE_SIZE_OFFSET = 28;

// IMU telegram
constexpr size_t IMU_VERSION_OFFSET = 8;
constexpr size_t IMU_DATA_OFFSET = 12;

// module metadata (telegram version 4), relative to the module start
constexpr size_t MODULE_SEGMENT_OFFSET = 0;
constexpr size_t MODULE_FRAME_OFFSET = 8;
constexpr size_t MODULE_SENDER_OFFSET = 16;
constexpr size_t MODULE_LINES_OFFSET = 20;
constexpr size_t MODULE_BEAMS_OFFSET = 24;
constexpr size_t MODULE_ECHOES_OFFSET = 28;
/* Per-line arrays follow: start time (8 bytes), stop time (8), elevation
 * phi (4), azimuth theta start (4) and stop (4); then the module tail. */
constexpr size_t MODULE_ARRAYS_OFFSET = 32;
constexpr size_t LINE_ARRAY_BYTES = 28;
constexpr size_t MODULE_TAIL_BYTES = 12;

inline size_t metadataBytes(uint32_t num_lines)
{
    return MODULE_ARRAYS_OFFSET + num_lines * LINE_ARRAY_BYTES +
           MODULE_TAIL_BYTES;
}

constexpr uint8_t ECHO_HAS_DISTANCE = 0x01;
constexpr uint8_t ECHO_HAS_RSSI = 0x02;
constexpr uint8_t BEAM_HAS_PROPERTIES = 0x01;
constexpr uint8_t BEAM_HAS_AZIMUTH = 0x02;

constexpr uint32_t MAX_BEAMS = 4096;
constexpr uint32_t MAX_ECHOES = 3;


// --- Layer ids ---------------------------------------------------------------

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
    const int millidegrees = static_cast<int>(
        std::lround(phi_rad * 180000 / 3.14159265358979323846));
    uint32_t best = 0;
    for (uint32_t layer = 1; layer < NUM_LAYERS; layer++)
    {
        if (std::abs(millidegrees - LAYER_ELEVATION_MDEG[layer]) <
            std::abs(millidegrees - LAYER_ELEVATION_MDEG[best]))
        {
            best = layer;
        }
    }
    return best;
}

}  // namespace



// --- Free functions ----------------------------------------------------------

const char* toString(Error error)
{
    switch (error)
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

TelegramType telegramType(std::span<const uint8_t> datagram)
{
    if (datagram.size() < 8 || !hasStartSequence(datagram))
    {
        return TelegramType::Unknown;
    }
    const uint32_t type = load<uint32_t>(datagram.data() + HEADER_TYPE_OFFSET);
    return (type == 1 || type == 2) ? static_cast<TelegramType>(type)
                                    : TelegramType::Unknown;
}

Error decodeImu(std::span<const uint8_t> datagram, ImuSample& sample)
{
    if (datagram.size() < 8)
    {
        return Error::TooShort;
    }
    if (!hasStartSequence(datagram))
    {
        return Error::BadStartSequence;
    }
    if (load<uint32_t>(datagram.data() + HEADER_TYPE_OFFSET) !=
        static_cast<uint32_t>(TelegramType::Imu))
    {
        return Error::UnknownTelegramType;
    }
    if (datagram.size() != IMU_TELEGRAM_BYTES)
    {
        return Error::BadModuleChain;
    }
    if (!crcValid(datagram))
    {
        return Error::BadCrc;
    }
    if (load<uint32_t>(datagram.data() + IMU_VERSION_OFFSET) !=
        IMU_TELEGRAM_VERSION)
    {
        return Error::UnsupportedVersion;
    }

    const uint8_t* value = datagram.data() + IMU_DATA_OFFSET;
    const auto readFloats = [&value](auto& out)
    {
        for (float& element : out)
        {
            element = load<float>(value);
            value += 4;
        }
    };
    readFloats(sample.acceleration);
    readFloats(sample.angular_velocity);
    readFloats(sample.orientation);
    sample.time_us = load<uint64_t>(value);
    return Error::None;
}



// --- ScanDecoder -------------------------------------------------------------

ScanDecoder::ScanDecoder(const PointLayout& layout, DecodeOptions options) :
    point_layout{layout},
    decode_options{options},
    field_offsets{
        .x = layout.offset(Field::X),
        .y = layout.offset(Field::Y),
        .z = layout.offset(Field::Z),
        .intensity = layout.offset(Field::Intensity),
        .range = layout.offset(Field::Range),
        .azimuth = layout.offset(Field::Azimuth),
        .elevation = layout.offset(Field::Elevation),
        .layer = layout.offset(Field::Layer),
        .echo = layout.offset(Field::Echo),
        .index = layout.offset(Field::Index),
        .time = layout.offset(Field::Time),
        .reflective = layout.offset(Field::Reflective),
        .artifact = layout.offset(Field::Artifact)}
{
}

Error ScanDecoder::decode(
    std::span<const uint8_t> datagram,
    uint8_t* points,
    size_t capacity,
    ScanInfo& info)
{
    // --- Framing ---
    if (datagram.size() < SCAN_HEADER_BYTES + 4)
    {
        return Error::TooShort;
    }
    if (!hasStartSequence(datagram))
    {
        return Error::BadStartSequence;
    }
    if (load<uint32_t>(datagram.data() + HEADER_TYPE_OFFSET) !=
        static_cast<uint32_t>(TelegramType::Scan))
    {
        return Error::UnknownTelegramType;
    }
    if (!crcValid(datagram))
    {
        return Error::BadCrc;
    }
    if (load<uint32_t>(datagram.data() + HEADER_VERSION_OFFSET) !=
        SCAN_TELEGRAM_VERSION)
    {
        return Error::UnsupportedVersion;
    }

    info = ScanInfo{};
    info.telegram_counter =
        load<uint64_t>(datagram.data() + HEADER_COUNTER_OFFSET);
    info.transmit_time_us =
        load<uint64_t>(datagram.data() + HEADER_TRANSMIT_TIME_OFFSET);

    // --- Modules ---
    ModuleList modules;
    size_t num_modules = 0;
    if (const Error error = this->parseModules(datagram, modules, num_modules);
        error != Error::None)
    {
        return error;
    }
    const uint8_t* first_module = modules[0].start;
    info.segment_index = load<uint64_t>(first_module + MODULE_SEGMENT_OFFSET);
    info.frame_number = load<uint64_t>(first_module + MODULE_FRAME_OFFSET);
    info.sender_id = load<uint32_t>(first_module + MODULE_SENDER_OFFSET);

    this->updateGeometry(modules, num_modules);
    info.num_points = this->geometry_points;
    info.start_time_us = load<uint64_t>(
        modules[this->first_layer_module].start + MODULE_ARRAYS_OFFSET +
        this->first_layer_line * 8);
    if (capacity < this->geometry_points)
    {
        return Error::BufferTooSmall;
    }

    // --- Points ---
    uint64_t time_min_us = std::numeric_limits<uint64_t>::max();
    uint64_t time_max_us = 0;
    for (size_t module = 0; module < num_modules; module++)
    {
        this->writeModulePoints(
            modules[module],
            this->line_geometry[module],
            info.segment_index,
            points,
            time_min_us,
            time_max_us);
    }
    info.time_min_us = time_min_us;
    info.time_max_us = time_max_us;
    return Error::None;
}

/* Walks the module chain and checks that every module's size matches its
 * metadata, and that the modules exactly fill the datagram. */
Error ScanDecoder::parseModules(
    std::span<const uint8_t> datagram,
    ModuleList& modules,
    size_t& num_modules) const
{
    const size_t end = datagram.size() - 4;  // before the CRC
    size_t offset = SCAN_HEADER_BYTES;
    uint64_t module_size =
        load<uint32_t>(datagram.data() + HEADER_FIRST_MODULE_SIZE_OFFSET);
    num_modules = 0;
    while (module_size)
    {
        if (num_modules == MAX_MODULES || offset + module_size > end ||
            module_size < MODULE_ARRAYS_OFFSET)
        {
            return Error::BadModuleChain;
        }
        ModuleView& module = modules[num_modules++];
        module.start = datagram.data() + offset;
        module.num_lines = load<uint32_t>(module.start + MODULE_LINES_OFFSET);
        module.num_beams = load<uint32_t>(module.start + MODULE_BEAMS_OFFSET);
        module.num_echoes = load<uint32_t>(module.start + MODULE_ECHOES_OFFSET);
        if (module.num_lines < 1 || module.num_lines > NUM_LAYERS ||
            module.num_beams < 1 || module.num_beams > MAX_BEAMS ||
            module.num_echoes < 1 || module.num_echoes > MAX_ECHOES)
        {
            return Error::BadModuleHeader;
        }
        const size_t metadata_size = metadataBytes(module.num_lines);
        if (module_size < metadata_size)
        {
            return Error::BadModuleSize;
        }

        const uint8_t* tail = module.start + MODULE_ARRAYS_OFFSET +
                              module.num_lines * LINE_ARRAY_BYTES;
        module.distance_scale = load<float>(tail);
        const uint32_t next_module_size = load<uint32_t>(tail + 4);
        module.echo_content = tail[9];
        module.beam_content = tail[10];
        const uint32_t echo_bytes =
            ((module.echo_content & ECHO_HAS_DISTANCE) ? 2u : 0u) +
            ((module.echo_content & ECHO_HAS_RSSI) ? 2u : 0u);
        module.beam_bytes =
            echo_bytes * module.num_echoes +
            ((module.beam_content & BEAM_HAS_PROPERTIES) ? 1u : 0u) +
            ((module.beam_content & BEAM_HAS_AZIMUTH) ? 2u : 0u);
        if (metadata_size + static_cast<uint64_t>(module.beam_bytes) *
                                module.num_lines * module.num_beams !=
            module_size)
        {
            return Error::BadModuleSize;
        }
        module.beam_data = module.start + metadata_size;

        offset += module_size;
        module_size = next_module_size;
    }
    if (offset != end || num_modules == 0)
    {
        return Error::BadModuleChain;
    }
    return Error::None;
}

/* Output position, layer id and elevation of every line: layers in
 * ascending id, lines sharing an id in packet order. */
void ScanDecoder::updateGeometry(const ModuleList& modules, size_t num_modules)
{
    GeometryKey key;
    for (size_t module = 0; module < num_modules; module++)
    {
        const ModuleView& view = modules[module];
        key.dimensions[module * 3 + 0] = view.num_lines;
        key.dimensions[module * 3 + 1] = view.num_beams;
        key.dimensions[module * 3 + 2] = view.num_echoes;
        const uint8_t* phi =
            view.start + MODULE_ARRAYS_OFFSET + view.num_lines * 16;
        std::memcpy(
            &key.elevation_bits[module * NUM_LAYERS],
            phi,
            view.num_lines * 4);
    }
    if (this->geometry_valid && key == this->geometry_key)
    {
        return;
    }

    struct LineReference
    {
        uint32_t module;
        uint32_t line;
        uint32_t layer_id;
        uint32_t num_beams;
    };
    std::vector<LineReference> lines;
    this->line_geometry.assign(num_modules, {});
    for (uint32_t module = 0; module < num_modules; module++)
    {
        const ModuleView& view = modules[module];
        const uint8_t* phi =
            view.start + MODULE_ARRAYS_OFFSET + view.num_lines * 16;
        this->line_geometry[module].resize(view.num_lines);
        for (uint32_t line = 0; line < view.num_lines; line++)
        {
            const float phi_rad = load<float>(phi + line * 4);
            LineGeometry& geometry = this->line_geometry[module][line];
            geometry.layer_id = layerIdFromPhi(phi_rad);
            // DIN ISO 8855 phi is positive below the horizon
            geometry.elevation = -phi_rad;
            geometry.sin_elevation = std::sin(geometry.elevation);
            geometry.cos_elevation = std::cos(geometry.elevation);
            lines.push_back({module, line, geometry.layer_id, view.num_beams});
        }
    }

    std::stable_sort(
        lines.begin(),
        lines.end(),
        [](const LineReference& a, const LineReference& b)
        { return a.layer_id < b.layer_id; });
    std::array<uint32_t, NUM_LAYERS> layer_points{};
    for (const LineReference& line : lines)
    {
        layer_points[line.layer_id] += line.num_beams;
    }
    uint32_t next_point = 0;
    for (const LineReference& line : lines)
    {
        LineGeometry& geometry = this->line_geometry[line.module][line.line];
        geometry.first_output_point = next_point;
        geometry.index_stride = layer_points[line.layer_id];
        next_point += line.num_beams;
    }

    this->geometry_points = next_point;
    this->first_layer_module = lines.empty() ? 0 : lines.front().module;
    this->first_layer_line = lines.empty() ? 0 : lines.front().line;
    this->geometry_key = key;
    this->geometry_valid = true;
}

/* Every beam of every line of one module, with or without a return. */
void ScanDecoder::writeModulePoints(
    const ModuleView& module,
    const std::vector<LineGeometry>& lines,
    uint64_t segment_index,
    uint8_t* points,
    uint64_t& time_min_us,
    uint64_t& time_max_us) const
{
    const FieldOffsets& field = this->field_offsets;
    const uint32_t point_step = this->point_layout.pointStep();

    // --- Line arrays: times and azimuths of the first and last beam ---
    const uint8_t* start_times = module.start + MODULE_ARRAYS_OFFSET;
    const uint8_t* stop_times = start_times + module.num_lines * 8;
    const uint8_t* start_azimuths = stop_times + module.num_lines * 12;
    const uint8_t* stop_azimuths = start_azimuths + module.num_lines * 4;

    const float beam_steps =
        static_cast<float>(std::max(1, static_cast<int>(module.num_beams) - 1));
    const uint64_t beam_steps_integer =
        std::max<uint64_t>(1, module.num_beams - 1);
    std::array<float, NUM_LAYERS> first_azimuth{};
    std::array<float, NUM_LAYERS> azimuth_step{};
    std::array<uint64_t, NUM_LAYERS> first_time_us{};
    std::array<uint64_t, NUM_LAYERS> duration_us{};
    for (uint32_t line = 0; line < module.num_lines; line++)
    {
        first_azimuth[line] = load<float>(start_azimuths + line * 4);
        azimuth_step[line] =
            (load<float>(stop_azimuths + line * 4) - first_azimuth[line]) /
            beam_steps;
        first_time_us[line] = load<uint64_t>(start_times + line * 8);
        duration_us[line] =
            load<uint64_t>(stop_times + line * 8) - first_time_us[line];
        const uint64_t last_time_us = first_time_us[line] + duration_us[line];
        time_min_us =
            std::min({time_min_us, first_time_us[line], last_time_us});
        time_max_us =
            std::max({time_max_us, first_time_us[line], last_time_us});
    }

    // --- Where the selected echo's values are within a beam ---
    const uint32_t echo =
        (this->decode_options.echo_selection >= 1 &&
         this->decode_options.echo_selection <= module.num_echoes)
            ? this->decode_options.echo_selection - 1
            : 0;
    const bool has_distance = module.echo_content & ECHO_HAS_DISTANCE;
    const bool has_rssi = module.echo_content & ECHO_HAS_RSSI;
    const bool has_properties = module.beam_content & BEAM_HAS_PROPERTIES;
    const bool has_azimuth = module.beam_content & BEAM_HAS_AZIMUTH;
    const uint32_t echo_bytes = (has_distance ? 2u : 0u) + (has_rssi ? 2u : 0u);
    const uint32_t distance_offset = echo_bytes * echo;
    const uint32_t rssi_offset = distance_offset + (has_distance ? 2u : 0u);
    const uint32_t properties_offset = echo_bytes * module.num_echoes;
    const uint32_t azimuth_offset =
        properties_offset + (has_properties ? 1u : 0u);

    // --- Points ---
    const uint8_t* beam_values = module.beam_data;
    for (uint32_t beam = 0; beam < module.num_beams; beam++)
    {
        for (uint32_t line = 0; line < module.num_lines;
             line++, beam_values += module.beam_bytes)
        {
            const LineGeometry& geometry = lines[line];
            const float range = has_distance
                                    ? (module.distance_scale *
                                       static_cast<float>(load<uint16_t>(
                                           beam_values + distance_offset))) /
                                          1000.0f
                                    : 0.f;
            const float azimuth =
                has_azimuth ? (static_cast<float>(load<uint16_t>(
                                   beam_values + azimuth_offset)) -
                               16384.0f) /
                                  5215.0f
                            : first_azimuth[line] +
                                  static_cast<float>(beam) * azimuth_step[line];
            const float sin_azimuth = std::sin(azimuth);
            const float cos_azimuth = std::cos(azimuth);

            uint8_t* point = points + static_cast<size_t>(
                                          geometry.first_output_point + beam) *
                                          point_step;
            store<float>(
                point,
                field.x,
                range * cos_azimuth * geometry.cos_elevation);
            store<float>(
                point,
                field.y,
                range * sin_azimuth * geometry.cos_elevation);
            store<float>(point, field.z, range * geometry.sin_elevation);
            store<float>(
                point,
                field.intensity,
                has_rssi ? static_cast<float>(
                               load<uint16_t>(beam_values + rssi_offset))
                         : 0.f);
            store<float>(point, field.range, range);
            store<float>(point, field.azimuth, azimuth);
            store<float>(point, field.elevation, geometry.elevation);
            store<uint32_t>(point, field.layer, geometry.layer_id);
            store<uint32_t>(point, field.echo, echo);
            store<uint32_t>(
                point,
                field.index,
                static_cast<uint32_t>(segment_index) * geometry.index_stride +
                    beam);
            store<uint64_t>(
                point,
                field.time,
                (beam * duration_us[line]) / beam_steps_integer +
                    first_time_us[line]);
            store<float>(
                point,
                field.reflective,
                has_properties
                    ? static_cast<float>(beam_values[properties_offset] & 0x01)
                    : 0.f);
            store<float>(
                point,
                field.artifact,
                -1.f);  // set by the ghost filter
        }
    }
}

}  // namespace multiscan::compact
