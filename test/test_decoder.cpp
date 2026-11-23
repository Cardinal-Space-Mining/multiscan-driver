/*  The new Compact decoder (src/compact/) against the oracle and the spec:
 *    - published clouds are byte-identical to the old driver's (oracle) on
 *      every capture and every synthetic configuration
 *    - every segment matches the executable spec exactly
 *    - field subsets are projections of the full layout, with the offsets the
 *      old compile-time configurations produced
 *    - malformed input is rejected without reading out of bounds (run this
 *      binary under ASan/UBSan to check the latter; see the fuzz test)
 */
#include <gtest/gtest.h>

#include <cstring>
#include <random>

#include "compact/decoder.hpp"
#include "compact/point_layout.hpp"
#include "legacy_slot_assembler.hpp"
#include "oracle/legacy_pipeline.hpp"
#include "support/compact_packet.hpp"
#include "support/expected_points.hpp"
#include "support/mscap.hpp"

#include "point_fields.hpp"

using namespace ms_test;
using multiscan::Field;
using multiscan::PointLayout;
namespace compact = multiscan::compact;

namespace
{

/* The node's data path (MultiscanNode::handleScan/publishScan) minus ROS. */
class NewPipeline
{
public:
    explicit NewPipeline(uint32_t echo_selection = 1, PointLayout layout = PointLayout::all()) :
        decoder(layout, {.echo_selection = echo_selection})
    {
    }

    void feed(std::span<const uint8_t> d)
    {
        if (compact::telegramType(d) != compact::TelegramType::Scan) return;
        uint64_t seg = UINT64_MAX;
        if (d.size() >= 40) std::memcpy(&seg, d.data() + 32, 8);
        if (seg >= 12) return;
        const size_t step = decoder.layout().pointStep();
        compact::ScanInfo info;
        auto err = decoder.decode(d, assembler.slotBuffer(seg, 4096, step), 4096, info);
        if (err != compact::Error::None)
        {
            errors++;
            return;
        }
        if (assembler.commit(seg, info.points, 0))
        {
            clouds.emplace_back();
            assembler.take(clouds.back(), step);
        }
    }

    compact::ScanDecoder decoder;
    multiscan::LegacySlotAssembler assembler;
    std::vector<std::vector<uint8_t>> clouds;
    size_t errors = 0;
};

void expectSameClouds(const legacy::Pipeline& oracle, const NewPipeline& mine, const std::string& name)
{
    ASSERT_EQ(mine.clouds.size(), oracle.clouds.size()) << name;
    ASSERT_GT(mine.clouds.size(), 0u) << name;
    for (size_t c = 0; c < mine.clouds.size(); c++)
    {
        const auto& a = oracle.clouds[c].data;
        const auto& b = mine.clouds[c];
        ASSERT_EQ(b.size(), a.size()) << name << " cloud " << c;
        if (std::memcmp(a.data(), b.data(), a.size()) != 0)
        {
            size_t i = 0;
            while (a[i] == b[i]) i++;
            FAIL() << name << " cloud " << c << ": first differing byte " << i << " (point " << i / 52
                   << ", byte " << i % 52 << ")";
        }
    }
}

std::vector<uint8_t> withFixedCrc(std::vector<uint8_t> b)
{
    const uint32_t crc = static_cast<uint32_t>(::crc32(0L, b.data(), static_cast<uInt>(b.size() - 4)));
    std::memcpy(b.data() + b.size() - 4, &crc, 4);
    return b;
}

}  // namespace


// --- Equivalence with the old driver -------------------------------------------

TEST(Decoder, ByteIdenticalToOracleOnCaptures)
{
    struct Case { const char* file; uint32_t echo; };
    for (Case c : {Case{"live_1echo.mscap.gz", 1}, Case{"interlaced.mscap.gz", 1},
                   Case{"all_layers_3echo.mscap.gz", 1}, Case{"all_layers_3echo.mscap.gz", 2},
                   Case{"all_layers_3echo.mscap.gz", 3}})
    {
        legacy::Pipeline oracle(static_cast<int>(c.echo));
        NewPipeline mine(c.echo);
        for (const auto& d : readCapture(fixture(c.file)))
        {
            oracle.feed(d.data, d.recv_time_ns);
            mine.feed(d.data);
        }
        EXPECT_EQ(mine.errors, 0u);
        expectSameClouds(oracle, mine, std::string(c.file) + " echo " + std::to_string(c.echo));
    }
}

TEST(Decoder, ByteIdenticalToOracleOnSyntheticConfigurations)
{
    std::vector<std::pair<std::string, SyntheticConfig>> cfgs;
    cfgs.emplace_back("live", SyntheticConfig::live());
    cfgs.emplace_back("all layers", SyntheticConfig{});
    {
        SyntheticConfig c; c.echoes = 3; cfgs.emplace_back("3 echoes", c);
    }
    {
        SyntheticConfig c = SyntheticConfig::live(); c.content_echoes = ECHO_HAS_DISTANCE;
        cfgs.emplace_back("no rssi", c);
    }
    {
        SyntheticConfig c = SyntheticConfig::live(); c.content_echoes = ECHO_HAS_RSSI;
        cfgs.emplace_back("no distance", c);
    }
    {
        SyntheticConfig c = SyntheticConfig::live(); c.content_beams = BEAM_HAS_PROPERTIES;
        cfgs.emplace_back("no per-beam azimuth", c);
    }
    {
        SyntheticConfig c = SyntheticConfig::live(); c.content_beams = BEAM_HAS_AZIMUTH;
        cfgs.emplace_back("no beam properties", c);
    }
    {
        SyntheticConfig c = SyntheticConfig::live(); c.content_beams = 0;
        cfgs.emplace_back("no beam data", c);
    }
    {
        SyntheticConfig c; c.distance_scale = 2.5f; cfgs.emplace_back("distance scale 2.5", c);
    }
    {
        SyntheticConfig c;
        c.layers = {true, false, true, false, true, true, false, true, true, true, false, false, true, false, true, true};
        cfgs.emplace_back("sparse layer filter", c);
    }
    {
        SyntheticConfig c; c.layers.fill(false); c.layers[13] = true;
        cfgs.emplace_back("single high-res layer", c);
    }
    for (const auto& [name, cfg] : cfgs)
    {
        for (uint32_t echo : {1u, 2u, 3u, 0u})
        {
            legacy::Pipeline oracle(static_cast<int>(echo));
            NewPipeline mine(echo);
            for (uint64_t frame = 3; frame < 6; frame++)
            {
                for (uint32_t seg = 0; seg < 12; seg++)
                {
                    const auto bytes = serialize(makeSegment(cfg, seg, frame));
                    oracle.feed(bytes, 0);
                    mine.feed(bytes);
                }
            }
            expectSameClouds(oracle, mine, name + " echo " + std::to_string(echo));
        }
    }
}

TEST(Decoder, EverySegmentMatchesSpecExactly)
{
    compact::ScanDecoder dec(PointLayout::all());
    std::vector<uint8_t> out(4096 * 52);
    size_t n = 0;
    for (const char* file : {"live_1echo.mscap.gz", "all_layers_3echo.mscap.gz", "range_pm10.mscap.gz"})
    {
        for (const auto& d : readCapture(fixture(file)))
        {
            auto t = parseScan(d.data);
            if (!t) continue;
            compact::ScanInfo info;
            ASSERT_EQ(dec.decode(d.data, out.data(), 4096, info), compact::Error::None) << file;
            const auto exp = expectedPoints(*t, 1);
            ASSERT_EQ(info.points, exp.size());
            EXPECT_EQ(info.segment, t->modules[0].segment_counter);
            EXPECT_EQ(info.frame, t->modules[0].frame_number);
            EXPECT_EQ(info.telegram_counter, t->telegram_counter);
            EXPECT_EQ(info.transmit_time_us, t->transmit_time_us);
            for (size_t i = 0; i < exp.size(); i++)
            {
                const auto got = readAllFieldsPoint(out.data() + i * 52);
                ASSERT_EQ(std::memcmp(&got.x, &exp[i].x, 4 * 7), 0) << file << " point " << i;
                ASSERT_EQ(got.layer, exp[i].layer);
                ASSERT_EQ(got.echo, exp[i].echo);
                ASSERT_EQ(got.index, exp[i].index);
                ASSERT_EQ(got.time_us, exp[i].time_us);
                ASSERT_EQ(got.reflective, exp[i].reflective);
            }
            n++;
        }
    }
    EXPECT_GT(n, 200u);
}

TEST(Decoder, ShapeCacheFollowsReconfiguration)
{
    // alternate between sensor configurations, as after a layer filter change
    compact::ScanDecoder dec(PointLayout::all());
    std::vector<uint8_t> out(4096 * 52);
    SyntheticConfig a = SyntheticConfig::live(), b;
    b.echoes = 2;
    for (int i = 0; i < 6; i++)
    {
        const auto t = makeSegment((i % 2) ? b : a, static_cast<uint32_t>(i), 9);
        compact::ScanInfo info;
        ASSERT_EQ(dec.decode(serialize(t), out.data(), 4096, info), compact::Error::None);
        const auto exp = expectedPoints(t, 1);
        ASSERT_EQ(info.points, exp.size()) << i;
        for (size_t k = 0; k < exp.size(); k++)
        {
            const auto got = readAllFieldsPoint(out.data() + k * 52);
            ASSERT_EQ(got.layer, exp[k].layer) << i;
            ASSERT_EQ(got.index, exp[k].index) << i;
            ASSERT_EQ(got.range, exp[k].range) << i;
        }
    }
}

TEST(Decoder, ImuMatchesCapture)
{
    size_t n = 0;
    for (const auto& d : readCapture(fixture("live_1echo.mscap.gz")))
    {
        auto exp = parseImu(d.data);
        if (!exp) continue;
        compact::ImuSample s;
        ASSERT_EQ(compact::decodeImu(d.data, s), compact::Error::None);
        EXPECT_EQ(s.acceleration, exp->acceleration);
        EXPECT_EQ(s.angular_velocity, exp->angular_velocity);
        EXPECT_EQ(s.orientation, exp->orientation);
        EXPECT_EQ(s.time_us, exp->time_us);
        n++;
    }
    EXPECT_GT(n, 15u);
}


// --- Point layouts ----------------------------------------------------------------

TEST(PointLayout, AllFieldsMatchesDriverDefault)
{
    const auto l = PointLayout::all();
    EXPECT_EQ(l.pointStep(), AllFieldsLayout::point_step);
    std::vector<std::string> names;
    for (const auto& f : l.fields()) names.push_back(f.name);
    EXPECT_EQ(names, (std::vector<std::string>{"x", "y", "z", "intensity", "range", "azimuth", "elevation",
                                               "layer", "echo", "index", "tl", "th", "reflective"}));
    EXPECT_EQ(l.offset(Field::Time), static_cast<int32_t>(AllFieldsLayout::tl));
    EXPECT_EQ(l.offset(Field::Reflective), static_cast<int32_t>(AllFieldsLayout::reflective));
}

TEST(PointLayout, SubsetsMatchOldCompileTimeConfigurations)
{
    std::string err;
    // MS_POINT_FIELD_ENABLE_XYZTR
    auto xyztr = PointLayout::fromNames({"x", "y", "z", "tl", "th", "reflective"}, err);
    ASSERT_TRUE(xyztr);
    EXPECT_EQ(xyztr->pointStep(), COMPUTE_NUM_POINT_FIELDS(MS_POINT_FIELD_ENABLE_XYZTR) * 4u);
    EXPECT_EQ(xyztr->offset(Field::Time), COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_POINT_FIELD_ENABLE_XYZTR) * 4);
    // MS_POINT_FIELD_ENABLE_XYZPTR
    auto xyzptr = PointLayout::fromNames(
        {"reflective", "x", "y", "z", "intensity", "range", "azimuth", "elevation", "tl", "th"}, err);
    ASSERT_TRUE(xyzptr);
    EXPECT_EQ(xyzptr->pointStep(), COMPUTE_NUM_POINT_FIELDS(MS_POINT_FIELD_ENABLE_XYZPTR) * 4u);
    EXPECT_EQ(xyzptr->offset(Field::Time), COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_POINT_FIELD_ENABLE_XYZPTR) * 4);
    EXPECT_EQ(xyzptr->offset(Field::Reflective), xyzptr->offset(Field::Time) + 8);
}

TEST(PointLayout, RejectsBadFieldLists)
{
    std::string err;
    EXPECT_FALSE(PointLayout::fromNames({"x", "y"}, err));
    EXPECT_FALSE(PointLayout::fromNames({"x", "y", "z", "colour"}, err));
    EXPECT_NE(err.find("colour"), std::string::npos);
}

TEST(PointLayout, SubsetIsProjectionOfFullLayout)
{
    std::string err;
    const auto sub = *PointLayout::fromNames({"x", "y", "z", "layer", "index", "range", "reflective"}, err);
    const auto full = PointLayout::all();
    compact::ScanDecoder dfull(full), dsub(sub);
    const auto bytes = serialize(makeSegment(SyntheticConfig{}, 4, 2));
    std::vector<uint8_t> a(4096 * full.pointStep()), b(4096 * sub.pointStep());
    compact::ScanInfo ia, ib;
    ASSERT_EQ(dfull.decode(bytes, a.data(), 4096, ia), compact::Error::None);
    ASSERT_EQ(dsub.decode(bytes, b.data(), 4096, ib), compact::Error::None);
    ASSERT_EQ(ia.points, ib.points);
    for (size_t i = 0; i < ia.points; i++)
    {
        for (Field f : {Field::X, Field::Y, Field::Z, Field::Layer, Field::Index, Field::Range, Field::Reflective})
        {
            ASSERT_EQ(std::memcmp(a.data() + i * full.pointStep() + full.offset(f),
                                  b.data() + i * sub.pointStep() + sub.offset(f), 4), 0);
        }
    }
}


// --- Malformed input ----------------------------------------------------------------

TEST(Decoder, RejectsMalformedTelegrams)
{
    compact::ScanDecoder dec(PointLayout::all());
    std::vector<uint8_t> out(4096 * 52);
    compact::ScanInfo info;
    const auto good = serialize(makeSegment(SyntheticConfig::live(), 2, 1));
    auto decode = [&](const std::vector<uint8_t>& b) { return dec.decode(b, out.data(), 4096, info); };

    EXPECT_EQ(decode(good), compact::Error::None);
    EXPECT_EQ(decode({good.begin(), good.begin() + 20}), compact::Error::TooShort);
    {
        auto b = good; b[0] = 0; EXPECT_EQ(decode(b), compact::Error::BadStartSequence);
    }
    {
        auto b = good; b[700] ^= 0x10; EXPECT_EQ(decode(b), compact::Error::BadCrc);
    }
    {
        auto b = good; b[4] = 7; EXPECT_EQ(decode(withFixedCrc(b)), compact::Error::UnknownTelegramType);
    }
    {
        SyntheticConfig v3 = SyntheticConfig::live(); v3.version = 3;
        EXPECT_EQ(decode(serialize(makeSegment(v3, 2, 1))), compact::Error::UnsupportedVersion);
    }
    {
        auto b = good; b[32 + 20] = 17;  // 17 lines in module 0
        EXPECT_EQ(decode(withFixedCrc(b)), compact::Error::BadModuleHeader);
    }
    {
        auto b = good; b[32 + 24] = 31;  // 31 beams: module size no longer matches
        EXPECT_EQ(decode(withFixedCrc(b)), compact::Error::BadModuleSize);
    }
    {
        auto b = good; b[28] += 1;  // first module size off by one
        const auto e = decode(withFixedCrc(b));
        EXPECT_TRUE(e == compact::Error::BadModuleSize || e == compact::Error::BadModuleChain) << toString(e);
    }
    {
        auto b = good; b.insert(b.end() - 4, 0);  // trailing byte before the CRC
        EXPECT_EQ(decode(withFixedCrc(b)), compact::Error::BadModuleChain);
    }
    {
        EXPECT_EQ(dec.decode(good, out.data(), 100, info), compact::Error::BufferTooSmall);
        EXPECT_EQ(info.points, 420u);
    }
    {
        compact::ImuSample s;
        auto imu = serialize(makeImu(5));
        EXPECT_EQ(compact::decodeImu(imu, s), compact::Error::None);
        imu[20] ^= 1;
        EXPECT_EQ(compact::decodeImu(imu, s), compact::Error::BadCrc);
        EXPECT_EQ(compact::decodeImu(good, s), compact::Error::UnknownTelegramType);
    }
}

TEST(Decoder, FuzzedHeadersNeverOverrun)
{
    // Mutate header/metadata bytes and re-seal the CRC so the structural
    // checks are what stands between the input and the output buffer.
    compact::ScanDecoder dec(PointLayout::all());
    const std::vector<std::vector<uint8_t>> seeds = {
        serialize(makeSegment(SyntheticConfig::live(), 1, 1)),
        serialize(makeSegment(SyntheticConfig{}, 5, 1)),
    };
    std::mt19937 rng(1234);
    constexpr size_t CAP = 1024;
    std::vector<uint8_t> out((CAP + 1) * 52, 0xAB);
    size_t accepted = 0;
    for (int iter = 0; iter < 20000; iter++)
    {
        auto b = seeds[iter % 2];
        const int n_mut = 1 + static_cast<int>(rng() % 4);
        for (int k = 0; k < n_mut; k++)
        {
            const size_t limit = std::min<size_t>(b.size() - 4, 32 + 512);  // header + first metadata
            b[rng() % limit] = static_cast<uint8_t>(rng());
        }
        if (rng() % 8 == 0) b.resize(rng() % b.size() + 1);
        if (b.size() >= 8) b = withFixedCrc(b);
        compact::ScanInfo info;
        if (dec.decode(b, out.data(), CAP, info) == compact::Error::None)
        {
            accepted++;
            ASSERT_LE(info.points, CAP);
        }
        // guard point after the capacity must never be written
        for (size_t i = CAP * 52; i < out.size(); i++) ASSERT_EQ(out[i], 0xAB);
    }
    EXPECT_GT(accepted, 0u);  // mutations of data bytes in metadata (times, angles) are legal
}
