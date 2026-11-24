/*  Independent reader/writer for the multiScan "Compact" UDP format (telegram
 *  types 1 and 2), written from the data format description in the
 *  multiScan136 operating instructions (8027119, section 12.4.1.4), not from
 *  the driver's decoder. Tests use it to
 *    - build synthetic segments for any sensor configuration, and
 *    - read captured packets into plain structs, as an executable statement of
 *      what the decoder has to produce.
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

// --- Format constants --------------------------------------------------------

constexpr uint32_t TELEGRAM_SCAN = 1;
constexpr uint32_t TELEGRAM_IMU = 2;
constexpr size_t SCAN_HEADER_BYTES = 32;
constexpr size_t IMU_TELEGRAM_BYTES = 64;  // incl. CRC

constexpr uint8_t ECHO_HAS_DISTANCE = 0x01;
constexpr uint8_t ECHO_HAS_RSSI = 0x02;
constexpr uint8_t BEAM_HAS_PROPERTIES = 0x01;
constexpr uint8_t BEAM_HAS_AZIMUTH = 0x02;

/* Azimuth encoding of the per-beam theta field:
 * radians = (encoded - 16384) / 5215 */
inline float decodeAzimuth(uint16_t encoded)
{
    return (static_cast<float>(encoded) - 16384.f) / 5215.f;
}
inline uint16_t encodeAzimuth(float radians)
{
    return static_cast<uint16_t>(std::lround(radians * 5215.f + 16384.f));
}


// --- Telegrams ---------------------------------------------------------------

struct Echo
{
    /* Multiply by the module's distance scaling factor for mm. */
    uint16_t distance{0};
    uint16_t rssi{0};
};

struct Beam
{
    std::array<Echo, 3> echoes{};
    uint8_t properties{0};  // bit 0: reflector
    uint16_t azimuth{0};    // encoded, see decodeAzimuth()
};

/* One module of a scan telegram. phi and theta are the operating
 * instructions' names: elevation (DIN ISO 8855, positive = below the
 * horizon) and azimuth, in radians. */
struct Module
{
public:
    const Beam& at(uint32_t line, uint32_t beam) const
    {
        return this->beam_data[beam * this->num_lines + line];
    }
    Beam& at(uint32_t line, uint32_t beam)
    {
        return this->beam_data[beam * this->num_lines + line];
    }

    /* One beam of one line, all echoes. */
    size_t beamBytes() const
    {
        const size_t echo_bytes =
            ((this->echo_content & ECHO_HAS_DISTANCE) ? 2 : 0) +
            ((this->echo_content & ECHO_HAS_RSSI) ? 2 : 0);
        return echo_bytes * this->num_echoes +
               ((this->beam_content & BEAM_HAS_PROPERTIES) ? 1 : 0) +
               ((this->beam_content & BEAM_HAS_AZIMUTH) ? 2 : 0);
    }
    size_t metadataBytes(uint32_t version) const
    {
        // header, per-line arrays (2 times, phi, 2 thetas), distance scale
        // (version 4), next module size, 4 content bytes
        return 32 + this->num_lines * (8 + 8 + 4 + 4 + 4) +
               (version >= 4 ? 4 : 0) + 4 + 4;
    }
    size_t bytes(uint32_t version) const
    {
        return this->metadataBytes(version) +
               this->beamBytes() * this->num_lines * this->num_beams;
    }

public:
    uint64_t segment_counter{0};
    uint64_t frame_number{0};
    uint32_t sender_id{0};
    uint32_t num_lines{0};
    uint32_t num_beams{0};
    uint32_t num_echoes{1};
    std::vector<uint64_t> time_start_us;  // [num_lines]
    std::vector<uint64_t> time_stop_us;   // [num_lines]
    std::vector<float> phi;               // [num_lines], rad
    std::vector<float> theta_start;       // [num_lines], rad
    std::vector<float> theta_stop;        // [num_lines], rad
    float distance_scale{1.f};            // telegram version >= 4
    uint8_t availability{0};
    uint8_t echo_content{ECHO_HAS_DISTANCE | ECHO_HAS_RSSI};
    uint8_t beam_content{BEAM_HAS_PROPERTIES | BEAM_HAS_AZIMUTH};
    uint8_t reserved{0};
    /* [num_beams * num_lines], beam-major: beam_data[beam * num_lines +
     * line]. */
    std::vector<Beam> beam_data;
};

struct ScanTelegram
{
    uint64_t telegram_counter{1};
    uint64_t transmit_time_us{0};
    uint32_t version{4};
    std::vector<Module> modules;
};

struct ImuTelegram
{
    uint32_t version{1};
    std::array<float, 3> acceleration{};      // m/s^2, gravity included
    std::array<float, 3> angular_velocity{};  // rad/s
    std::array<float, 4> orientation{};       // w, x, y, z
    uint64_t time_us{0};
};


// --- Byte access -------------------------------------------------------------

namespace detail
{

template<typename T> inline void put(std::vector<uint8_t>& bytes, T value)
{
    uint8_t encoded[sizeof(T)];
    std::memcpy(encoded, &value, sizeof(T));
    bytes.insert(bytes.end(), encoded, encoded + sizeof(T));
}

template<typename T, typename List>
inline void putAll(std::vector<uint8_t>& bytes, const List& values)
{
    for (const auto value : values)
    {
        put<T>(bytes, value);
    }
}

class Reader
{
public:
    explicit Reader(std::span<const uint8_t> bytes) : bytes{bytes} {}

public:
    template<typename T> bool get(T& value)
    {
        if (this->offset + sizeof(T) > this->bytes.size())
        {
            return false;
        }
        std::memcpy(&value, this->bytes.data() + this->offset, sizeof(T));
        this->offset += sizeof(T);
        return true;
    }

    template<typename List> bool getAll(List& values, uint32_t count)
    {
        values.resize(count);
        for (auto& value : values)
        {
            if (!this->get(value))
            {
                return false;
            }
        }
        return true;
    }

public:
    size_t offset{0};
    std::span<const uint8_t> bytes;
};

inline uint32_t crc32(std::span<const uint8_t> bytes)
{
    return static_cast<uint32_t>(
        ::crc32(0L, bytes.data(), static_cast<uInt>(bytes.size())));
}

/* One beam: the echoes, then properties and azimuth (telegram version 4;
 * version 3 had those two swapped). */
inline void putBeam(
    std::vector<uint8_t>& bytes,
    const Module& module,
    const Beam& beam,
    uint32_t version)
{
    for (uint32_t echo = 0; echo < module.num_echoes; echo++)
    {
        if (module.echo_content & ECHO_HAS_DISTANCE)
        {
            put<uint16_t>(bytes, beam.echoes[echo].distance);
        }
        if (module.echo_content & ECHO_HAS_RSSI)
        {
            put<uint16_t>(bytes, beam.echoes[echo].rssi);
        }
    }
    const bool has_properties = module.beam_content & BEAM_HAS_PROPERTIES;
    const bool has_azimuth = module.beam_content & BEAM_HAS_AZIMUTH;
    if (version >= 4 && has_properties)
    {
        put<uint8_t>(bytes, beam.properties);
    }
    if (has_azimuth)
    {
        put<uint16_t>(bytes, beam.azimuth);
    }
    if (version < 4 && has_properties)
    {
        put<uint8_t>(bytes, beam.properties);
    }
}

inline void putModule(
    std::vector<uint8_t>& bytes,
    const Module& module,
    uint32_t version,
    uint32_t next_module_bytes)
{
    put<uint64_t>(bytes, module.segment_counter);
    put<uint64_t>(bytes, module.frame_number);
    put<uint32_t>(bytes, module.sender_id);
    put<uint32_t>(bytes, module.num_lines);
    put<uint32_t>(bytes, module.num_beams);
    put<uint32_t>(bytes, module.num_echoes);
    putAll<uint64_t>(bytes, module.time_start_us);
    putAll<uint64_t>(bytes, module.time_stop_us);
    putAll<float>(bytes, module.phi);
    putAll<float>(bytes, module.theta_start);
    putAll<float>(bytes, module.theta_stop);
    if (version >= 4)
    {
        put<float>(bytes, module.distance_scale);
    }
    put<uint32_t>(bytes, next_module_bytes);
    put<uint8_t>(bytes, module.availability);
    put<uint8_t>(bytes, module.echo_content);
    put<uint8_t>(bytes, module.beam_content);
    put<uint8_t>(bytes, module.reserved);
    for (const Beam& beam : module.beam_data)
    {
        putBeam(bytes, module, beam, version);
    }
}

inline bool
    getBeam(Reader& reader, const Module& module, Beam& beam, uint32_t version)
{
    for (uint32_t echo = 0; echo < module.num_echoes; echo++)
    {
        if ((module.echo_content & ECHO_HAS_DISTANCE) &&
            !reader.get(beam.echoes[echo].distance))
        {
            return false;
        }
        if ((module.echo_content & ECHO_HAS_RSSI) &&
            !reader.get(beam.echoes[echo].rssi))
        {
            return false;
        }
    }
    const bool has_properties = module.beam_content & BEAM_HAS_PROPERTIES;
    const bool has_azimuth = module.beam_content & BEAM_HAS_AZIMUTH;
    return !(version >= 4 && has_properties && !reader.get(beam.properties)) &&
           !(has_azimuth && !reader.get(beam.azimuth)) &&
           !(version < 4 && has_properties && !reader.get(beam.properties));
}

/* One module; next_module_bytes is the size of the module after it (0:
 * the last). */
inline bool getModule(
    Reader& reader,
    uint32_t version,
    uint32_t module_bytes,
    Module& module,
    uint32_t& next_module_bytes)
{
    const size_t module_start = reader.offset;
    if (!reader.get(module.segment_counter) ||
        !reader.get(module.frame_number) || !reader.get(module.sender_id) ||
        !reader.get(module.num_lines) || !reader.get(module.num_beams) ||
        !reader.get(module.num_echoes) || module.num_lines > 16 ||
        module.num_echoes < 1 || module.num_echoes > 3)
    {
        return false;
    }
    if (!reader.getAll(module.time_start_us, module.num_lines) ||
        !reader.getAll(module.time_stop_us, module.num_lines) ||
        !reader.getAll(module.phi, module.num_lines) ||
        !reader.getAll(module.theta_start, module.num_lines) ||
        !reader.getAll(module.theta_stop, module.num_lines))
    {
        return false;
    }
    if (version >= 4 && !reader.get(module.distance_scale))
    {
        return false;
    }
    if (!reader.get(next_module_bytes) || !reader.get(module.availability) ||
        !reader.get(module.echo_content) || !reader.get(module.beam_content) ||
        !reader.get(module.reserved))
    {
        return false;
    }
    if (module_start + module.bytes(version) != module_start + module_bytes)
    {
        return false;
    }
    module.beam_data.resize(
        static_cast<size_t>(module.num_lines) * module.num_beams);
    for (Beam& beam : module.beam_data)
    {
        if (!getBeam(reader, module, beam, version))
        {
            return false;
        }
    }
    return true;
}

}  // namespace detail


// --- Writing -----------------------------------------------------------------

/* Serialize a scan telegram, including the start sequence and trailing CRC32
 * (computed over the whole frame, header included). */
inline std::vector<uint8_t> serialize(const ScanTelegram& telegram)
{
    using detail::put;
    const uint32_t version = telegram.version;
    const std::vector<Module>& modules = telegram.modules;
    const auto moduleBytes = [&](size_t index) -> uint32_t
    {
        return index < modules.size()
                   ? static_cast<uint32_t>(modules[index].bytes(version))
                   : 0;
    };

    std::vector<uint8_t> bytes = {0x02, 0x02, 0x02, 0x02};
    put<uint32_t>(bytes, TELEGRAM_SCAN);
    put<uint64_t>(bytes, telegram.telegram_counter);
    put<uint64_t>(bytes, telegram.transmit_time_us);
    put<uint32_t>(bytes, version);
    put<uint32_t>(bytes, moduleBytes(0));
    for (size_t index = 0; index < modules.size(); index++)
    {
        detail::putModule(
            bytes,
            modules[index],
            version,
            moduleBytes(index + 1));
    }
    put<uint32_t>(bytes, detail::crc32(bytes));
    return bytes;
}

inline std::vector<uint8_t> serialize(const ImuTelegram& telegram)
{
    using detail::put;
    std::vector<uint8_t> bytes = {0x02, 0x02, 0x02, 0x02};
    put<uint32_t>(bytes, TELEGRAM_IMU);
    put<uint32_t>(bytes, telegram.version);
    detail::putAll<float>(bytes, telegram.acceleration);
    detail::putAll<float>(bytes, telegram.angular_velocity);
    detail::putAll<float>(bytes, telegram.orientation);
    put<uint64_t>(bytes, telegram.time_us);
    put<uint32_t>(bytes, detail::crc32(bytes));
    return bytes;
}


// --- Reading -----------------------------------------------------------------

inline std::optional<uint32_t> telegramType(std::span<const uint8_t> bytes)
{
    static constexpr uint8_t START_SEQUENCE[4] = {2, 2, 2, 2};
    if (bytes.size() < 8 || std::memcmp(bytes.data(), START_SEQUENCE, 4) != 0)
    {
        return std::nullopt;
    }
    uint32_t type;
    std::memcpy(&type, bytes.data() + 4, 4);
    return type;
}

inline bool crcValid(std::span<const uint8_t> bytes)
{
    if (bytes.size() < 8)
    {
        return false;
    }
    uint32_t stored;
    std::memcpy(&stored, bytes.data() + bytes.size() - 4, 4);
    return stored == detail::crc32(bytes.first(bytes.size() - 4));
}

/* Parse a complete scan telegram (one datagram). Returns nullopt on any
 * framing, size or CRC error. */
inline std::optional<ScanTelegram> parseScan(std::span<const uint8_t> bytes)
{
    if (telegramType(bytes) != TELEGRAM_SCAN || !crcValid(bytes))
    {
        return std::nullopt;
    }
    detail::Reader reader{bytes.first(bytes.size() - 4)};
    reader.offset = 8;
    ScanTelegram telegram;
    uint32_t module_bytes = 0;
    if (!reader.get(telegram.telegram_counter) ||
        !reader.get(telegram.transmit_time_us) ||
        !reader.get(telegram.version) || !reader.get(module_bytes) ||
        (telegram.version != 3 && telegram.version != 4))
    {
        return std::nullopt;
    }
    while (module_bytes)
    {
        Module module;
        uint32_t next_module_bytes = 0;
        if (!detail::getModule(
                reader,
                telegram.version,
                module_bytes,
                module,
                next_module_bytes))
        {
            return std::nullopt;
        }
        telegram.modules.push_back(std::move(module));
        module_bytes = next_module_bytes;
    }
    if (reader.offset != bytes.size() - 4)
    {
        return std::nullopt;
    }
    return telegram;
}

inline std::optional<ImuTelegram> parseImu(std::span<const uint8_t> bytes)
{
    if (telegramType(bytes) != TELEGRAM_IMU ||
        bytes.size() != IMU_TELEGRAM_BYTES || !crcValid(bytes))
    {
        return std::nullopt;
    }
    detail::Reader reader{bytes};
    reader.offset = 8;
    ImuTelegram telegram;
    reader.get(telegram.version);
    for (float& value : telegram.acceleration)
    {
        reader.get(value);
    }
    for (float& value : telegram.angular_velocity)
    {
        reader.get(value);
    }
    for (float& value : telegram.orientation)
    {
        reader.get(value);
    }
    reader.get(telegram.time_us);
    return telegram;
}


// --- Synthetic segments ------------------------------------------------------

/* Nominal elevations (phi, rad) of the 16 layers in packet order, measured on
 * the team's sensor (serial 23170008). Module 0 holds layers 1-8, module 1
 * holds layers 9-16; layers 6 and 14 are the 0.125 deg high-resolution
 * layers. */
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
public:
    SyntheticConfig() { this->layers.fill(true); }

    /* The team's current sensor setup. */
    static SyntheticConfig live()
    {
        SyntheticConfig config;
        config.layers[5] = config.layers[13] = false;
        return config;
    }

public:
    std::array<bool, 16> layers;  // layer filter (true = output)
    uint32_t num_echoes{1};
    uint8_t echo_content{ECHO_HAS_DISTANCE | ECHO_HAS_RSSI};
    uint8_t beam_content{BEAM_HAS_PROPERTIES | BEAM_HAS_AZIMUTH};
    float distance_scale{1.f};
    uint32_t version{4};
};

namespace detail
{

constexpr float PI = 3.14159265358979f;

/* A deterministic but non-trivial range/RSSI/reflector pattern; about 4 % of
 * beams have no return. */
inline void fillBeams(
    const SyntheticConfig& config,
    const std::vector<uint32_t>& layer_ids,
    uint32_t segment,
    uint64_t frame,
    Module& module)
{
    module.beam_data.resize(
        static_cast<size_t>(module.num_lines) * module.num_beams);
    for (uint32_t beam_index = 0; beam_index < module.num_beams; beam_index++)
    {
        for (uint32_t line = 0; line < module.num_lines; line++)
        {
            Beam& beam = module.at(line, beam_index);
            const uint32_t hash =
                (beam_index * 2654435761u) ^ (layer_ids[line] * 40503u) ^
                (segment * 9176u) ^ static_cast<uint32_t>(frame);
            const bool empty = (hash % 23) == 0;
            for (uint32_t echo = 0; echo < config.num_echoes; echo++)
            {
                beam.echoes[echo].distance =
                    empty ? 0
                          : static_cast<uint16_t>(
                                300 + (hash % 25000) + echo * 1500);
                beam.echoes[echo].rssi =
                    empty ? 0 : static_cast<uint16_t>((hash >> 7) % 60000);
            }
            beam.properties = (!empty && (hash % 31) == 0) ? 1 : 0;
            // in this order: the golden clouds were made with it
            beam.azimuth = encodeAzimuth(
                module.theta_start[line] +
                (module.theta_stop[line] - module.theta_start[line]) *
                    static_cast<float>(beam_index) /
                    static_cast<float>(module.num_beams - 1));
        }
    }
}

}  // namespace detail

/* Build segment `segment` (0..11) of frame `frame`. Module order follows the
 * sensor: 1 deg module of unit 0, 1 deg module of unit 1, then the high-res
 * modules. */
inline ScanTelegram
    makeSegment(const SyntheticConfig& config, uint32_t segment, uint64_t frame)
{
    using detail::PI;
    const uint64_t frame_start_us = 1'000'000'000ULL + frame * 50'000ULL;
    const uint64_t segment_start_us = frame_start_us + segment * 4'167ULL;

    ScanTelegram telegram;
    telegram.telegram_counter = frame * 12 + segment + 1;
    telegram.transmit_time_us = segment_start_us + 7'000;
    telegram.version = config.version;

    struct ModuleGroup
    {
        uint32_t unit;
        bool high_resolution;
    };
    for (const ModuleGroup group : {
             ModuleGroup{0, false},
             ModuleGroup{1, false},
             ModuleGroup{0,  true},
             ModuleGroup{1,  true}
    })
    {
        std::vector<uint32_t> layer_ids;
        for (uint32_t layer = group.unit * 8; layer < group.unit * 8 + 8;
             layer++)
        {
            const bool high_resolution = (layer == 5 || layer == 13);
            if (high_resolution == group.high_resolution &&
                config.layers[layer])
            {
                layer_ids.push_back(layer);
            }
        }
        if (layer_ids.empty())
        {
            continue;
        }

        Module module;
        module.segment_counter = segment;
        module.frame_number = frame;
        module.sender_id = 23170008;
        module.num_echoes = config.num_echoes;
        module.num_beams = group.high_resolution ? 240 : 30;
        module.distance_scale = config.distance_scale;
        module.echo_content = config.echo_content;
        module.beam_content = config.beam_content;
        module.num_lines = static_cast<uint32_t>(layer_ids.size());

        // unit 1 looks the other way; each layer has a small azimuth phase
        // offset
        const float unit_theta =
            (group.unit ? 0.f : PI) - PI / 6.f * 0.95f + segment * (PI / 6.f);
        const float beam_step =
            (PI / 6.f) / static_cast<float>(module.num_beams);
        for (uint32_t line = 0; line < module.num_lines; line++)
        {
            const uint32_t layer = layer_ids[line];
            const float first_theta =
                unit_theta + 0.01f * static_cast<float>(layer % 8);
            module.phi.push_back(NOMINAL_PHI[layer]);
            module.theta_start.push_back(first_theta);
            module.theta_stop.push_back(
                first_theta +
                beam_step * static_cast<float>(module.num_beams - 1));
            module.time_start_us.push_back(segment_start_us + line * 3);
            module.time_stop_us.push_back(segment_start_us + 4'100 + line * 3);
        }
        detail::fillBeams(config, layer_ids, segment, frame, module);
        telegram.modules.push_back(std::move(module));
    }
    return telegram;
}

inline ImuTelegram makeImu(uint64_t time_us)
{
    ImuTelegram telegram;
    const float phase = static_cast<float>(time_us % 1000) * 1e-3f;
    telegram.acceleration = {0.1f * phase, -0.2f, 9.81f};
    telegram.angular_velocity = {0.01f, 0.02f * phase, -0.03f};
    telegram.orientation = {0.999f, 0.01f, -0.02f, 0.03f * phase};
    telegram.time_us = time_us;
    return telegram;
}

}  // namespace ms_test
