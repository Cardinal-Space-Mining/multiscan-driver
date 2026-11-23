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

/*  Maps the sensor clock to the host clock.
 *
 *  Every scan telegram carries the sensor time at which it was sent; the
 *  kernel stamps the time it arrived. Their difference d = host - sensor is
 *  the clock offset plus the transport delay, which is never negative and
 *  usually close to its minimum (on a direct link the spread is ~0.1 ms).
 *  So the lower envelope of d is the offset (plus the constant minimum
 *  delay):
 *
 *  - the minimum of d is taken over blocks of block_ns host time,
 *  - a line is fitted through the last window_blocks minima, which follows
 *    the drift between the clocks (the multiScan136 runs ~9 ppm slow
 *    without time sync, ~0.5 ms per minute),
 *  - a clock jump (sensor reboot: its clock restarts at 0; an NTP step on
 *    either side) is detected and the estimate restarts.
 *
 *  The estimate is a smooth function of host time, so stamps derived from
 *  sensor times keep the sensor's exact spacing instead of the receive
 *  jitter.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>


namespace multiscan
{

class ClockEstimator
{
public:
    struct Config
    {
        /* Host time over which the minimum of d is taken. */
        uint64_t block_ns = 1'000'000'000;
        /* Block minima the drift line is fitted through. */
        size_t window_blocks = 60;
        /* A block minimum this far from the line... */
        int64_t step_ns = 1'000'000;
        /* ...this many blocks in a row is a clock step: refit from them. */
        uint32_t step_blocks = 3;
        /* A sample this far below the line is impossible (negative delay):
         * the clock jumped, restart at once. This far above it for
         * jump_samples samples in a row also restarts. */
        int64_t jump_ns = 100'000'000;
        uint32_t jump_samples = 5;
        /* Larger fitted drifts (fractional) are treated as noise. */
        double max_drift = 500e-6;
    };

    struct Stats
    {
        uint64_t samples = 0;
        uint64_t restarts = 0;  // clock jumps
        uint64_t steps = 0;     // smaller steps that refitted the line
        /* Delay of the last completed block above the estimate:
         * its minimum (~0 when the fit is good) and maximum (jitter). */
        int64_t block_min_delay_ns = 0;
        int64_t block_max_delay_ns = 0;
    };

    ClockEstimator();
    explicit ClockEstimator(Config config);

    /* One telegram: the sensor time it was sent at and when it arrived. */
    void add(uint64_t sensor_us, uint64_t host_ns);

    /* True once a sample was added. */
    bool valid() const { return have_estimate; }

    /* Estimated host - sensor (ns) around host time host_ns. */
    int64_t offsetNs(uint64_t host_ns) const;

    /* Sensor time (us) -> host time (ns), using the offset around host_ns. */
    uint64_t toHostNs(uint64_t sensor_us, uint64_t host_ns) const
    {
        return static_cast<uint64_t>(
            static_cast<int64_t>(sensor_us * 1000) + offsetNs(host_ns));
    }

    /* Fitted drift of the host clock against the sensor clock (fraction:
     * 1e-6 = the host clock gains 1 us per second). */
    double drift() const { return slope; }

    const Stats& stats() const { return counters; }

    /* Forget everything (counters are kept). */
    void reset();

private:
    struct Point
    {
        uint64_t host_ns;
        int64_t d;  // block minimum
    };

    void closeBlock();
    void fit();
    int64_t predict(uint64_t host_ns) const;

    Config cfg;
    Stats counters;

    // current block
    bool block_open = false;
    uint64_t block_start_ns = 0;
    Point block_min{};
    int64_t block_max_delay = 0;

    std::deque<Point> minima;
    uint32_t step_streak = 0;
    int step_sign = 0;
    uint32_t jump_streak = 0;

    // estimate: d(t) = base_ns + base_frac + slope * (t - ref_ns)
    bool have_estimate = false;
    bool have_fit = false;
    uint64_t ref_ns = 0;
    int64_t base_ns = 0;
    double base_frac = 0.;
    double slope = 0.;
};

}  // namespace multiscan
