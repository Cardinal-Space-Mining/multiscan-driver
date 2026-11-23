/*  Independent reader/writer for the multiScan "Compact" UDP format (telegram
 *  types 1 and 2), written from the data format description in the
 *  multiScan136 operating instructions (8027119, section 12.4.1.4), not from
 *  the vendored sick_scan_xd code. Tests use it to
 *    - build synthetic segments for any sensor configuration, and
 *    - read captured packets into plain structs, as an executable statement of
 *      what every decoder (vendored or new) has to produce.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

#include <zlib.h>


namespace ms_test
{

constexpr uint32_t TELEGRAM_SCAN = 1;
constexpr uint32_t TELEGRAM_IMU = 2;
constexpr size_t SCAN_HEADER_BYTES = 32;
constexpr size_t IMU_TELEGRAM_BYTES = 64;  // incl. CRC

constexpr uint8_t ECHO_HAS_DISTANCE = 0x01;
constexpr uint8_t ECHO_HAS_RSSI = 0x02;
constexpr uint8_t BEAM_HAS_PROPERTIES = 0x01;
constexpr uint8_t BEAM_HAS_AZIMUTH = 0x02;

// Azimuth encoding of the per-beam theta field: a_rad = (a_uint - 16384) / 5215
inline float decodeAzimuth(uint16_t a)
{
    return (static_cast<float>(a) - 16384.f) / 5215.f;
}
inline uint16_t encodeAzimuth(float rad)
{
    return static_cast<uint16_t>(std::lround(rad * 5215.f + 16384.f));
}

struct Echo
{
    uint16_t distance =
        0;  // multiply by the module's distance scaling factor for mm
    uint16_t rssi = 0;
};

struct Beam
{
    std::array<Echo, 3> echoes{};
    uint8_t properties = 0;  // bit 0: reflector
    uint16_t azimuth = 0;    // encoded, see decodeAzimuth()
};

struct Module
{
    uint64_t segment_counter = 0;
    uint64_t frame_number = 0;
    uint32_t sender_id = 0;
    uint32_t lines = 0;
    uint32_t beams = 0;
    uint32_t echoes = 1;
    std::vector<uint64_t> time_start_us;  // [lines]
    std::vector<uint64_t> time_stop_us;   // [lines]
    std::vector<float>
        phi;  // [lines], rad, DIN ISO 8855 (positive = below horizon)
    std::vector<float> theta_start;  // [lines], rad
    std::vector<float> theta_stop;   // [lines], rad
    float distance_scale = 1.f;      // telegram version >= 4
    uint8_t availability = 0;
    uint8_t content_echoes = ECHO_HAS_DISTANCE | ECHO_HAS_RSSI;
    uint8_t content_beams = BEAM_HAS_PROPERTIES | BEAM_HAS_AZIMUTH;
    uint8_t reserved = 0;
    std::vector<Beam>
        data;  // [beams * lines], beam-major: data[beam * lines + line]

    const Beam& at(uint32_t line, uint32_t beam) const
    {
        return data[beam * lines + line];
    }
    Beam& at(uint32_t line, uint32_t beam) { return data[beam * lines + line]; }

    size_t tupleBytes() const
    {
        size_t per_echo = ((content_echoes & ECHO_HAS_DISTANCE) ? 2 : 0) +
                          ((content_echoes & ECHO_HAS_RSSI) ? 2 : 0);
        return per_echo * echoes +
               ((content_beams & BEAM_HAS_PROPERTIES) ? 1 : 0) +
               ((content_beams & BEAM_HAS_AZIMUTH) ? 2 : 0);
    }
    size_t metadataBytes(uint32_t version) const
    {
        return 32 + lines * (8 + 8 + 4 + 4 + 4) + (version >= 4 ? 4 : 0) + 4 +
               4;
    }
    size_t bytes(uint32_t version) const
    {
        return metadataBytes(version) + tupleBytes() * lines * beams;
    }
};

struct ScanTelegram
{
    uint64_t telegram_counter = 1;
    uint64_t transmit_time_us = 0;
    uint32_t version = 4;
    std::vector<Module> modules;
};

struct ImuTelegram
{
    uint32_t version = 1;
    std::array<float, 3> acceleration{};      // m/s^2, gravity included
    std::array<float, 3> angular_velocity{};  // rad/s
    std::array<float, 4> orientation{};       // w, x, y, z
    uint64_t time_us = 0;
};


namespace detail
{
template<typename T> inline void put(std::vector<uint8_t>& b, T v)
{
    uint8_t tmp[sizeof(T)];
    std::memcpy(tmp, &v, sizeof(T));
    b.insert(b.end(), tmp, tmp + sizeof(T));
}

class Reader
{
public:
    explicit Reader(std::span<const uint8_t> b) : buf{b} {}
    template<typename T> bool get(T& v)
    {
        if (off + sizeof(T) > buf.size())
        {
            return false;
        }
        std::memcpy(&v, buf.data() + off, sizeof(T));
        off += sizeof(T);
        return true;
    }
    size_t off = 0;
    std::span<const uint8_t> buf;
};

inline uint32_t crc32(std::span<const uint8_t> b)
{
    return static_cast<uint32_t>(
        ::crc32(0L, b.data(), static_cast<uInt>(b.size())));
}
}  // namespace detail


/* Serialize a scan telegram, including the start sequence and trailing CRC32
 * (computed over the whole frame, header included). */
inline std::vector<uint8_t> serialize(const ScanTelegram& t)
{
    using detail::put;
    std::vector<uint8_t> b = {0x02, 0x02, 0x02, 0x02};
    put<uint32_t>(b, TELEGRAM_SCAN);
    put<uint64_t>(b, t.telegram_counter);
    put<uint64_t>(b, t.transmit_time_us);
    put<uint32_t>(b, t.version);
    put<uint32_t>(
        b,
        t.modules.empty()
            ? 0
            : static_cast<uint32_t>(t.modules[0].bytes(t.version)));
    for (size_t mi = 0; mi < t.modules.size(); mi++)
    {
        const Module& m = t.modules[mi];
        put<uint64_t>(b, m.segment_counter);
        put<uint64_t>(b, m.frame_number);
        put<uint32_t>(b, m.sender_id);
        put<uint32_t>(b, m.lines);
        put<uint32_t>(b, m.beams);
        put<uint32_t>(b, m.echoes);
        for (auto v : m.time_start_us)
        {
            put<uint64_t>(b, v);
        }
        for (auto v : m.time_stop_us)
        {
            put<uint64_t>(b, v);
        }
        for (auto v : m.phi)
        {
            put<float>(b, v);
        }
        for (auto v : m.theta_start)
        {
            put<float>(b, v);
        }
        for (auto v : m.theta_stop)
        {
            put<float>(b, v);
        }
        if (t.version >= 4)
        {
            put<float>(b, m.distance_scale);
        }
        const bool last = (mi + 1 == t.modules.size());
        put<uint32_t>(
            b,
            last ? 0
                 : static_cast<uint32_t>(t.modules[mi + 1].bytes(t.version)));
        put<uint8_t>(b, m.availability);
        put<uint8_t>(b, m.content_echoes);
        put<uint8_t>(b, m.content_beams);
        put<uint8_t>(b, m.reserved);
        for (const Beam& beam : m.data)
        {
            for (uint32_t e = 0; e < m.echoes; e++)
            {
                if (m.content_echoes & ECHO_HAS_DISTANCE)
                {
                    put<uint16_t>(b, beam.echoes[e].distance);
                }
                if (m.content_echoes & ECHO_HAS_RSSI)
                {
                    put<uint16_t>(b, beam.echoes[e].rssi);
                }
            }
            // telegram version 4: properties before azimuth (version 3 had them swapped)
            if (t.version >= 4)
            {
                if (m.content_beams & BEAM_HAS_PROPERTIES)
                {
                    put<uint8_t>(b, beam.properties);
                }
                if (m.content_beams & BEAM_HAS_AZIMUTH)
                {
                    put<uint16_t>(b, beam.azimuth);
                }
            }
            else
            {
                if (m.content_beams & BEAM_HAS_AZIMUTH)
                {
                    put<uint16_t>(b, beam.azimuth);
                }
                if (m.content_beams & BEAM_HAS_PROPERTIES)
                {
                    put<uint8_t>(b, beam.properties);
                }
            }
        }
    }
    put<uint32_t>(b, detail::crc32(b));
    return b;
}

inline std::vector<uint8_t> serialize(const ImuTelegram& t)
{
    using detail::put;
    std::vector<uint8_t> b = {0x02, 0x02, 0x02, 0x02};
    put<uint32_t>(b, TELEGRAM_IMU);
    put<uint32_t>(b, t.version);
    for (float v : t.acceleration)
    {
        put<float>(b, v);
    }
    for (float v : t.angular_velocity)
    {
        put<float>(b, v);
    }
    for (float v : t.orientation)
    {
        put<float>(b, v);
    }
    put<uint64_t>(b, t.time_us);
    put<uint32_t>(b, detail::crc32(b));
    return b;
}


inline std::optional<uint32_t> telegramType(std::span<const uint8_t> b)
{
    static constexpr uint8_t STX[4] = {2, 2, 2, 2};
    if (b.size() < 8 || std::memcmp(b.data(), STX, 4) != 0)
    {
        return std::nullopt;
    }
    uint32_t t;
    std::memcpy(&t, b.data() + 4, 4);
    return t;
}

inline bool crcValid(std::span<const uint8_t> b)
{
    if (b.size() < 8)
    {
        return false;
    }
    uint32_t stored;
    std::memcpy(&stored, b.data() + b.size() - 4, 4);
    return stored == detail::crc32(b.first(b.size() - 4));
}

/* Parse a complete scan telegram (one datagram). Returns nullopt on any
 * framing, size or CRC error. */
inline std::optional<ScanTelegram> parseScan(std::span<const uint8_t> b)
{
    if (telegramType(b) != TELEGRAM_SCAN || !crcValid(b))
    {
        return std::nullopt;
    }
    detail::Reader r{b.first(b.size() - 4)};
    r.off = 8;
    ScanTelegram t;
    uint32_t module_size = 0;
    if (!r.get(t.telegram_counter) || !r.get(t.transmit_time_us) ||
        !r.get(t.version) || !r.get(module_size) ||
        (t.version != 3 && t.version != 4))
    {
        return std::nullopt;
    }
    while (module_size)
    {
        const size_t module_begin = r.off;
        Module m;
        uint32_t next = 0;
        if (!r.get(m.segment_counter) || !r.get(m.frame_number) ||
            !r.get(m.sender_id) || !r.get(m.lines) || !r.get(m.beams) ||
            !r.get(m.echoes) || m.lines > 16 || m.echoes < 1 || m.echoes > 3)
        {
            return std::nullopt;
        }
        auto arr = [&r](auto& v, uint32_t n)
        {
            v.resize(n);
            for (auto& x : v)
            {
                if (!r.get(x))
                {
                    return false;
                }
            }
            return true;
        };
        if (!arr(m.time_start_us, m.lines) || !arr(m.time_stop_us, m.lines) ||
            !arr(m.phi, m.lines) || !arr(m.theta_start, m.lines) ||
            !arr(m.theta_stop, m.lines))
        {
            return std::nullopt;
        }
        if (t.version >= 4 && !r.get(m.distance_scale))
        {
            return std::nullopt;
        }
        if (!r.get(next) || !r.get(m.availability) ||
            !r.get(m.content_echoes) || !r.get(m.content_beams) ||
            !r.get(m.reserved))
        {
            return std::nullopt;
        }
        if (module_begin + m.bytes(t.version) != module_begin + module_size)
        {
            return std::nullopt;
        }
        m.data.resize(static_cast<size_t>(m.lines) * m.beams);
        for (Beam& beam : m.data)
        {
            for (uint32_t e = 0; e < m.echoes; e++)
            {
                if ((m.content_echoes & ECHO_HAS_DISTANCE) &&
                    !r.get(beam.echoes[e].distance))
                {
                    return std::nullopt;
                }
                if ((m.content_echoes & ECHO_HAS_RSSI) &&
                    !r.get(beam.echoes[e].rssi))
                {
                    return std::nullopt;
                }
            }
            const bool v4 = t.version >= 4;
            if (v4 && (m.content_beams & BEAM_HAS_PROPERTIES) &&
                !r.get(beam.properties))
            {
                return std::nullopt;
            }
            if ((m.content_beams & BEAM_HAS_AZIMUTH) && !r.get(beam.azimuth))
            {
                return std::nullopt;
            }
            if (!v4 && (m.content_beams & BEAM_HAS_PROPERTIES) &&
                !r.get(beam.properties))
            {
                return std::nullopt;
            }
        }
        t.modules.push_back(std::move(m));
        module_size = next;
    }
    if (r.off != b.size() - 4)
    {
        return std::nullopt;
    }
    return t;
}

inline std::optional<ImuTelegram> parseImu(std::span<const uint8_t> b)
{
    if (telegramType(b) != TELEGRAM_IMU || b.size() != IMU_TELEGRAM_BYTES ||
        !crcValid(b))
    {
        return std::nullopt;
    }
    detail::Reader r{b};
    r.off = 8;
    ImuTelegram t;
    r.get(t.version);
    for (float& v : t.acceleration)
    {
        r.get(v);
    }
    for (float& v : t.angular_velocity)
    {
        r.get(v);
    }
    for (float& v : t.orientation)
    {
        r.get(v);
    }
    r.get(t.time_us);
    return t;
}


// --- Synthetic segments ------------------------------------------------------

/* Nominal elevations (phi, rad) of the 16 layers in packet order, measured on
 * the team's sensor (serial 23170008). Module 0 holds layers 1-8, module 1
 * holds layers 9-16; layers 6 and 14 are the 0.125 deg high-resolution layers. */
inline constexpr std::array<float, 16> NOMINAL_PHI = {
    0.390975f,
    0.303366f,
    0.218469f,
    0.132131f,
    0.048454f,
    0.005757f,
    -0.034470f,
    -0.117062f,
    -0.219243f,
    -0.294787f,
    -0.375273f,
    -0.458180f,
    -0.544040f,
    -0.590062f,
    -0.634508f,
    -0.731123f};

struct SyntheticConfig
{
    std::array<bool, 16> layers;  // layer filter (true = output)
    uint32_t echoes = 1;
    uint8_t content_echoes = ECHO_HAS_DISTANCE | ECHO_HAS_RSSI;
    uint8_t content_beams = BEAM_HAS_PROPERTIES | BEAM_HAS_AZIMUTH;
    float distance_scale = 1.f;
    uint32_t version = 4;

    SyntheticConfig() { layers.fill(true); }
    static SyntheticConfig live()  // the team's current sensor setup
    {
        SyntheticConfig c;
        c.layers[5] = c.layers[13] = false;
        return c;
    }
};

/* Build segment `seg` (0..11) of frame `frame` with a deterministic but
 * non-trivial range/RSSI/reflector pattern. Module order follows the sensor:
 * 1 deg module of unit 0, 1 deg module of unit 1, then the high-res modules. */
inline ScanTelegram
    makeSegment(const SyntheticConfig& cfg, uint32_t seg, uint64_t frame)
{
    constexpr float PI = 3.14159265358979f;
    const uint64_t frame_t0 = 1'000'000'000ULL + frame * 50'000ULL;
    const uint64_t seg_t0 = frame_t0 + seg * 4'167ULL;

    ScanTelegram t;
    t.telegram_counter = frame * 12 + seg + 1;
    t.transmit_time_us = seg_t0 + 7'000;
    t.version = cfg.version;

    struct Group
    {
        uint32_t unit;
        bool hd;
    };
    for (Group g : {
             Group{0, false},
             Group{1, false},
             Group{0,  true},
             Group{1,  true}
    })
    {
        Module m;
        m.segment_counter = seg;
        m.frame_number = frame;
        m.sender_id = 23170008;
        m.echoes = cfg.echoes;
        m.beams = g.hd ? 240 : 30;
        m.distance_scale = cfg.distance_scale;
        m.content_echoes = cfg.content_echoes;
        m.content_beams = cfg.content_beams;
        std::vector<uint32_t> layer_ids;
        for (uint32_t l = g.unit * 8; l < g.unit * 8 + 8; l++)
        {
            const bool is_hd = (l == 5 || l == 13);
            if (is_hd == g.hd && cfg.layers[l])
            {
                layer_ids.push_back(l);
            }
        }
        if (layer_ids.empty())
        {
            continue;
        }
        m.lines = static_cast<uint32_t>(layer_ids.size());
        // unit 1 looks the other way; each layer has a small azimuth phase offset
        const float unit_theta =
            (g.unit ? 0.f : PI) - PI / 6.f * 0.95f + seg * (PI / 6.f);
        for (uint32_t li = 0; li < m.lines; li++)
        {
            const uint32_t l = layer_ids[li];
            const float th0 = unit_theta + 0.01f * static_cast<float>(l % 8);
            const float step = (PI / 6.f) / static_cast<float>(m.beams);
            m.phi.push_back(NOMINAL_PHI[l]);
            m.theta_start.push_back(th0);
            m.theta_stop.push_back(
                th0 + step * static_cast<float>(m.beams - 1));
            m.time_start_us.push_back(seg_t0 + li * 3);
            m.time_stop_us.push_back(seg_t0 + 4'100 + li * 3);
        }
        m.data.resize(static_cast<size_t>(m.lines) * m.beams);
        for (uint32_t b = 0; b < m.beams; b++)
        {
            for (uint32_t li = 0; li < m.lines; li++)
            {
                Beam& beam = m.at(li, b);
                const uint32_t l = layer_ids[li];
                const uint32_t h = (b * 2654435761u) ^ (l * 40503u) ^
                                   (seg * 9176u) ^ static_cast<uint32_t>(frame);
                const bool empty = (h % 23) == 0;  // ~4% no return
                for (uint32_t e = 0; e < cfg.echoes; e++)
                {
                    beam.echoes[e].distance =
                        empty ? 0
                              : static_cast<uint16_t>(
                                    300 + (h % 25000) + e * 1500);
                    beam.echoes[e].rssi =
                        empty ? 0 : static_cast<uint16_t>((h >> 7) % 60000);
                }
                beam.properties = (!empty && (h % 31) == 0) ? 1 : 0;
                beam.azimuth = encodeAzimuth(
                    m.theta_start[li] + (m.theta_stop[li] - m.theta_start[li]) *
                                            static_cast<float>(b) /
                                            static_cast<float>(m.beams - 1));
            }
        }
        t.modules.push_back(std::move(m));
    }
    return t;
}

inline ImuTelegram makeImu(uint64_t time_us)
{
    ImuTelegram t;
    const float s = static_cast<float>(time_us % 1000) * 1e-3f;
    t.acceleration = {0.1f * s, -0.2f, 9.81f};
    t.angular_velocity = {0.01f, 0.02f * s, -0.03f};
    t.orientation = {0.999f, 0.01f, -0.02f, 0.03f * s};
    t.time_us = time_us;
    return t;
}

}  // namespace ms_test
