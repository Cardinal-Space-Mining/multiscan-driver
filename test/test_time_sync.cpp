/*  ClockEstimator against simulated telegram streams: 240 telegrams/s,
 *  transport delay = 30 us + exponential jitter (mean 100 us, rare 3 ms
 *  spikes), the spread seen on the direct link to the sensor.
 */
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>

#include "time_sync.hpp"

using multiscan::ClockEstimator;

namespace
{

constexpr int64_t MIN_DELAY_NS = 30'000;

struct Stream
{
    /* sensor_us(host) = (host - start) * (1 - drift) / 1000 + sensor0_us */
    uint64_t host_ns = 1'795'450'000'000'000'000ULL;  // ~2026
    uint64_t sensor0_us = 422'717'320;                // ~7 min after boot
    double drift = 0.;
    uint64_t start_ns = host_ns;
    std::mt19937_64 rng{42};
    std::exponential_distribution<double> jitter{1. / 100'000.};
    std::uniform_real_distribution<double> u{0., 1.};

    uint64_t sensorAt(uint64_t host) const
    {
        const double dt = static_cast<double>(host - start_ns);
        return sensor0_us + static_cast<uint64_t>(dt * (1. - drift) / 1000.);
    }
    /* True host - sensor offset at host time `host` (what the estimate
     * should equal, plus the minimum delay). */
    int64_t trueOffset(uint64_t host) const
    {
        return static_cast<int64_t>(host) -
               static_cast<int64_t>(sensorAt(host) * 1000);
    }

    /* Advance one telegram period and deliver it. */
    void step(ClockEstimator& e)
    {
        host_ns += 4'166'667;
        const uint64_t sent = host_ns;
        double delay = MIN_DELAY_NS + jitter(rng);
        if (u(rng) < 0.002)
        {
            delay += 3'000'000;
        }
        e.add(sensorAt(sent), sent + static_cast<uint64_t>(delay));
    }
    void run(ClockEstimator& e, double seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds * 240); i++)
        {
            step(e);
        }
    }
    /* Estimate error, net of the constant minimum delay. */
    int64_t error(const ClockEstimator& e) const
    {
        return e.offsetNs(host_ns) - trueOffset(host_ns) - MIN_DELAY_NS;
    }
};

}  // namespace


TEST(ClockEstimator, InvalidUntilTheFirstSample)
{
    ClockEstimator e;
    EXPECT_FALSE(e.valid());
    EXPECT_EQ(e.offsetNs(123), 0);
    Stream s;
    s.step(e);
    EXPECT_TRUE(e.valid());
    EXPECT_LT(std::abs(s.error(e)), 2'000'000);  // one sample: within its delay
}

TEST(ClockEstimator, ConstantOffsetIsTheLowerEnvelope)
{
    ClockEstimator e;
    Stream s;
    s.run(e, 1.5);
    EXPECT_LT(std::abs(s.error(e)), 10'000);  // after one block
    s.run(e, 30);
    EXPECT_LT(std::abs(s.error(e)), 5'000);
    EXPECT_LT(std::abs(e.drift()), 0.5e-6);
    EXPECT_EQ(e.stats().restarts, 0u);
    EXPECT_EQ(e.stats().steps, 0u);
    // the jitter shows up as the block's max delay, not in the estimate
    EXPECT_GT(e.stats().block_max_delay_ns, 300'000);
}

TEST(ClockEstimator, FollowsDrift)
{
    // the multiScan136 without time sync: ~9 ppm against the host
    for (double drift : {9e-6, -9e-6, 100e-6})
    {
        ClockEstimator e;
        Stream s;
        s.drift = drift;
        s.run(e, 120);
        EXPECT_NEAR(e.drift(), drift, 0.3e-6) << drift;
        EXPECT_LT(std::abs(s.error(e)), 10'000) << drift;
        // and in between samples (extrapolated half a block ahead)
        EXPECT_LT(
            std::abs(
                e.offsetNs(s.host_ns + 500'000'000) -
                s.trueOffset(s.host_ns + 500'000'000) - MIN_DELAY_NS),
            15'000)
            << drift;
    }
}

TEST(ClockEstimator, SpacingOfConvertedTimesFollowsTheSensor)
{
    // stamps of telegrams 1/240 s apart must be 1/240 s apart, not jittered
    ClockEstimator e;
    Stream s;
    s.drift = 9e-6;
    s.run(e, 10);
    const uint64_t a_us = s.sensorAt(s.host_ns);
    const uint64_t b_us = a_us + 4'167;
    const uint64_t a = e.toHostNs(a_us, s.host_ns);
    const uint64_t b = e.toHostNs(b_us, s.host_ns + 4'166'667);
    EXPECT_NEAR(static_cast<double>(b - a), 4'167'000., 100.);
}

TEST(ClockEstimator, SensorRebootRestartsTheEstimate)
{
    ClockEstimator e;
    Stream s;
    s.run(e, 10);
    // the sensor clock restarts at zero: d jumps up by minutes
    s.start_ns = s.host_ns;
    s.sensor0_us = 0;
    s.run(e, 0.1);
    EXPECT_EQ(e.stats().restarts, 1u);
    s.run(e, 2);
    EXPECT_LT(std::abs(s.error(e)), 10'000);
}

TEST(ClockEstimator, ClockSetForwardRestartsAtOnce)
{
    // e.g. the sensor gets NTP time: sensor clock jumps ahead by years, a
    // sample then "arrives before it was sent"
    ClockEstimator e;
    Stream s;
    s.run(e, 5);
    s.sensor0_us += 1'795'000'000'000'000ULL;
    s.step(e);
    EXPECT_EQ(e.stats().restarts, 1u);
    s.run(e, 2);
    EXPECT_LT(std::abs(s.error(e)), 10'000);
}

TEST(ClockEstimator, SmallStepIsRefittedAfterAFewBlocks)
{
    ClockEstimator e;
    Stream s;
    s.run(e, 20);
    // the host clock is stepped back 20 ms (below the restart threshold)
    s.sensor0_us += 20'000;
    s.run(e, 5);
    EXPECT_EQ(e.stats().restarts, 0u);
    EXPECT_EQ(e.stats().steps, 1u);
    s.run(e, 2);
    EXPECT_LT(std::abs(s.error(e)), 10'000);
}

TEST(ClockEstimator, StrayLateDatagramsAreIgnored)
{
    ClockEstimator e;
    Stream s;
    s.run(e, 5);
    const int64_t before = s.error(e);
    // a few datagrams held up 300 ms (e.g. the host was suspended)
    for (int i = 0; i < 3; i++)
    {
        s.host_ns += 4'166'667;
        e.add(s.sensorAt(s.host_ns), s.host_ns + 300'000'000);
    }
    s.run(e, 0.5);
    EXPECT_EQ(e.stats().restarts, 0u);
    EXPECT_LT(std::abs(s.error(e) - before), 10'000);
}
