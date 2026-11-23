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
    float x, y, z, intensity, range, azimuth, elevation;
    uint32_t layer, echo, index;
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
    const int mdeg = static_cast<int>(
        std::lround(phi_rad * 180000.0 / 3.14159265358979323846));
    uint32_t best = 0;
    for (uint32_t i = 1; i < LAYER_ELEVATION_MDEG.size(); i++)
    {
        if (std::abs(mdeg - LAYER_ELEVATION_MDEG[i]) <
            std::abs(mdeg - LAYER_ELEVATION_MDEG[best]))
        {
            best = i;
        }
    }
    return best;
}

inline std::vector<ExpectedPoint> expectedPoints(
    const ScanTelegram& t,
    int echo_selection = 1)
{
    struct Line
    {
        const Module* m;
        uint32_t line;
        uint32_t layer;
    };
    std::vector<Line> lines;
    for (const Module& m : t.modules)
    {
        for (uint32_t l = 0; l < m.lines; l++)
        {
            lines.push_back({&m, l, layerIdFromPhi(m.phi[l])});
        }
    }
    std::stable_sort(
        lines.begin(),
        lines.end(),
        [](const Line& a, const Line& b) { return a.layer < b.layer; });

    std::vector<ExpectedPoint> out;
    for (const Line& ln : lines)
    {
        const Module& m = *ln.m;
        const uint32_t echo =
            (echo_selection >= 1 &&
             static_cast<uint32_t>(echo_selection) <= m.echoes)
                ? static_cast<uint32_t>(echo_selection - 1)
                : 0;
        const float el = -m.phi[ln.line];
        const float sin_el = std::sin(el), cos_el = std::cos(el);
        const float az_delta =
            (m.theta_stop[ln.line] - m.theta_start[ln.line]) /
            static_cast<float>(std::max(1, static_cast<int>(m.beams) - 1));
        const uint64_t t0 = m.time_start_us[ln.line],
                       t1 = m.time_stop_us[ln.line];
        for (uint32_t b = 0; b < m.beams; b++)
        {
            const Beam& beam = m.at(ln.line, b);
            ExpectedPoint p;
            p.range = (m.content_echoes & ECHO_HAS_DISTANCE)
                          ? (m.distance_scale *
                             static_cast<float>(beam.echoes[echo].distance)) /
                                1000.f
                          : 0.f;
            p.intensity = (m.content_echoes & ECHO_HAS_RSSI)
                              ? static_cast<float>(beam.echoes[echo].rssi)
                              : 0.f;
            p.azimuth =
                (m.content_beams & BEAM_HAS_AZIMUTH)
                    ? decodeAzimuth(beam.azimuth)
                    : m.theta_start[ln.line] + static_cast<float>(b) * az_delta;
            p.elevation = el;
            p.x = p.range * std::cos(p.azimuth) * cos_el;
            p.y = p.range * std::sin(p.azimuth) * cos_el;
            p.z = p.range * sin_el;
            p.layer = ln.layer;
            p.echo = echo;
            p.index = static_cast<uint32_t>(m.segment_counter) * m.beams + b;
            p.time_us =
                (b * (t1 - t0)) / std::max<uint64_t>(1, m.beams - 1) + t0;
            p.reflective = (m.content_beams & BEAM_HAS_PROPERTIES)
                               ? static_cast<float>(beam.properties & 1)
                               : 0.f;
            out.push_back(p);
        }
    }
    return out;
}

/* Field offsets of the driver's default (MS_POINT_FIELD_ENABLE_ALL) layout. */
struct AllFieldsLayout
{
    static constexpr uint32_t x = 0, y = 4, z = 8, intensity = 12, range = 16,
                              azimuth = 20, elevation = 24, layer = 28,
                              echo = 32, index = 36, tl = 40, th = 44,
                              reflective = 48, point_step = 52;
};

inline ExpectedPoint readAllFieldsPoint(const uint8_t* p)
{
    using L = AllFieldsLayout;
    ExpectedPoint e;
    std::memcpy(&e.x, p + L::x, 4);
    std::memcpy(&e.y, p + L::y, 4);
    std::memcpy(&e.z, p + L::z, 4);
    std::memcpy(&e.intensity, p + L::intensity, 4);
    std::memcpy(&e.range, p + L::range, 4);
    std::memcpy(&e.azimuth, p + L::azimuth, 4);
    std::memcpy(&e.elevation, p + L::elevation, 4);
    std::memcpy(&e.layer, p + L::layer, 4);
    std::memcpy(&e.echo, p + L::echo, 4);
    std::memcpy(&e.index, p + L::index, 4);
    std::memcpy(&e.time_us, p + L::tl, 8);
    std::memcpy(&e.reflective, p + L::reflective, 4);
    return e;
}

}  // namespace ms_test
