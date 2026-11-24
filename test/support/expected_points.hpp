/*  Executable specification of the points multiscan_driver publishes for one
 *  segment, derived from a parsed telegram (compact_packet.hpp) rather than
 *  from any decoder implementation.
 *
 *  Semantics locked in by phase 1 (behaviour of the driver at 184806e):
 *    - one point per beam per layer, for the selected echo only; beams without
 *      a return are kept (range 0, xyz 0)
 *    - points ordered by layer id ascending, then beam index
 *    - layer      = index into the 16 nominal layer elevations nearest to phi
 *    - index      = segment_counter * beams_in_layer + beam
 *    - echo       = selected echo (0-based; echo_selection falls back to 0)
 *    - range [m]  = distance * distance_scale / 1000
 *    - azimuth    = per-beam azimuth if present, else linear theta_start..theta_stop
 *    - elevation  = -phi
 *    - xyz        = range * (cos az cos el, sin az cos el, sin el)
 *    - intensity  = RSSI of the selected echo
 *    - reflective = beam property bit 0
 *    - time [us]  = time_start + beam * (time_stop - time_start) / max(1, beams - 1)
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "compact_packet.hpp"


namespace ms_test
{

struct ExpectedPoint
{
    float x;
    float y;
    float z;
    float intensity;
    float range;
    float azimuth;
    float elevation;
    uint32_t layer;
    uint32_t echo;
    uint32_t index;
    uint64_t time_us;
    float reflective;
};


/* The layer table the driver maps elevations onto (mdeg, phi sign). */
inline constexpr std::array<int, 16> LAYER_ELEVATION_MDEG = {
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

inline uint32_t layerIdFromPhi(float phi_rad)
{
    const int millidegrees = static_cast<int>(
        std::lround(phi_rad * 180000.0 / 3.14159265358979323846));
    uint32_t best = 0;
    for (uint32_t layer = 1; layer < LAYER_ELEVATION_MDEG.size(); layer++)
    {
        if (std::abs(millidegrees - LAYER_ELEVATION_MDEG[layer]) <
            std::abs(millidegrees - LAYER_ELEVATION_MDEG[best]))
        {
            best = layer;
        }
    }
    return best;
}


namespace detail
{

/* The points of one line (layer) of a module, in beam order. */
inline void appendLinePoints(
    const Module& module,
    uint32_t line,
    uint32_t layer_id,
    uint32_t echo,
    std::vector<ExpectedPoint>& out)
{
    const float elevation = -module.phi[line];
    const float sin_elevation = std::sin(elevation);
    const float cos_elevation = std::cos(elevation);
    const float azimuth_step =
        (module.theta_stop[line] - module.theta_start[line]) /
        static_cast<float>(std::max(1, static_cast<int>(module.num_beams) - 1));
    const uint64_t start_us = module.time_start_us[line];
    const uint64_t stop_us = module.time_stop_us[line];
    const bool has_distance = module.echo_content & ECHO_HAS_DISTANCE;
    const bool has_rssi = module.echo_content & ECHO_HAS_RSSI;
    const bool has_azimuth = module.beam_content & BEAM_HAS_AZIMUTH;
    const bool has_properties = module.beam_content & BEAM_HAS_PROPERTIES;

    for (uint32_t beam_index = 0; beam_index < module.num_beams; beam_index++)
    {
        const Beam& beam = module.at(line, beam_index);
        ExpectedPoint point;
        point.range = has_distance
                          ? (module.distance_scale *
                             static_cast<float>(beam.echoes[echo].distance)) /
                                1000.f
                          : 0.f;
        point.intensity =
            has_rssi ? static_cast<float>(beam.echoes[echo].rssi) : 0.f;
        point.azimuth = has_azimuth
                            ? decodeAzimuth(beam.azimuth)
                            : module.theta_start[line] +
                                  static_cast<float>(beam_index) * azimuth_step;
        point.elevation = elevation;
        point.x = point.range * std::cos(point.azimuth) * cos_elevation;
        point.y = point.range * std::sin(point.azimuth) * cos_elevation;
        point.z = point.range * sin_elevation;
        point.layer = layer_id;
        point.echo = echo;
        point.index =
            static_cast<uint32_t>(module.segment_counter) * module.num_beams +
            beam_index;
        point.time_us = (beam_index * (stop_us - start_us)) /
                            std::max<uint64_t>(1, module.num_beams - 1) +
                        start_us;
        point.reflective =
            has_properties ? static_cast<float>(beam.properties & 1) : 0.f;
        out.push_back(point);
    }
}

}  // namespace detail

inline std::vector<ExpectedPoint> expectedPoints(
    const ScanTelegram& telegram,
    int echo_selection = 1)
{
    struct LineReference
    {
        const Module* module;
        uint32_t line;
        uint32_t layer_id;
    };
    std::vector<LineReference> lines;
    for (const Module& module : telegram.modules)
    {
        for (uint32_t line = 0; line < module.num_lines; line++)
        {
            lines.push_back({&module, line, layerIdFromPhi(module.phi[line])});
        }
    }
    std::stable_sort(
        lines.begin(),
        lines.end(),
        [](const LineReference& a, const LineReference& b)
        { return a.layer_id < b.layer_id; });

    std::vector<ExpectedPoint> out;
    for (const LineReference& line : lines)
    {
        const Module& module = *line.module;
        const uint32_t echo =
            (echo_selection >= 1 &&
             static_cast<uint32_t>(echo_selection) <= module.num_echoes)
                ? static_cast<uint32_t>(echo_selection - 1)
                : 0;
        detail::appendLinePoints(module, line.line, line.layer_id, echo, out);
    }
    return out;
}


/* Field offsets of the driver's layout with every field
 * (PointLayout::all()). */
struct AllFieldsLayout
{
    static constexpr uint32_t x = 0;
    static constexpr uint32_t y = 4;
    static constexpr uint32_t z = 8;
    static constexpr uint32_t intensity = 12;
    static constexpr uint32_t range = 16;
    static constexpr uint32_t azimuth = 20;
    static constexpr uint32_t elevation = 24;
    static constexpr uint32_t layer = 28;
    static constexpr uint32_t echo = 32;
    static constexpr uint32_t index = 36;
    static constexpr uint32_t tl = 40;  // time, low and high 32 bits
    static constexpr uint32_t th = 44;
    static constexpr uint32_t reflective = 48;
    static constexpr uint32_t point_step = 52;
};

inline ExpectedPoint readAllFieldsPoint(const uint8_t* point)
{
    using Layout = AllFieldsLayout;
    ExpectedPoint fields;
    std::memcpy(&fields.x, point + Layout::x, 4);
    std::memcpy(&fields.y, point + Layout::y, 4);
    std::memcpy(&fields.z, point + Layout::z, 4);
    std::memcpy(&fields.intensity, point + Layout::intensity, 4);
    std::memcpy(&fields.range, point + Layout::range, 4);
    std::memcpy(&fields.azimuth, point + Layout::azimuth, 4);
    std::memcpy(&fields.elevation, point + Layout::elevation, 4);
    std::memcpy(&fields.layer, point + Layout::layer, 4);
    std::memcpy(&fields.echo, point + Layout::echo, 4);
    std::memcpy(&fields.index, point + Layout::index, 4);
    std::memcpy(&fields.time_us, point + Layout::tl, 8);
    std::memcpy(&fields.reflective, point + Layout::reflective, 4);
    return fields;
}

}  // namespace ms_test
