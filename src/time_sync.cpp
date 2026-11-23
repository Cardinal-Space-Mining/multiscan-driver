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

#include "time_sync.hpp"

#include <algorithm>
#include <cmath>


namespace multiscan
{

ClockEstimator::ClockEstimator() : ClockEstimator(Config{}) {}

ClockEstimator::ClockEstimator(Config config) : cfg{config} {}

void ClockEstimator::reset()
{
    block_open = false;
    minima.clear();
    step_streak = 0;
    step_sign = 0;
    jump_streak = 0;
    have_estimate = false;
    have_fit = false;
    slope = 0.;
}

int64_t ClockEstimator::predict(uint64_t host_ns) const
{
    const double dt =
        static_cast<double>(static_cast<int64_t>(host_ns - ref_ns));
    return base_ns + static_cast<int64_t>(std::llround(base_frac + slope * dt));
}

int64_t ClockEstimator::offsetNs(uint64_t host_ns) const
{
    if (!have_estimate)
    {
        return 0;
    }
    if (!have_fit)
    {
        return block_min.d;  // first block: its running minimum
    }
    return predict(host_ns);
}

void ClockEstimator::add(uint64_t sensor_us, uint64_t host_ns)
{
    const int64_t d =
        static_cast<int64_t>(host_ns) - static_cast<int64_t>(sensor_us * 1000);
    counters.samples++;

    if (have_estimate)
    {
        const int64_t delay = d - offsetNs(host_ns);
        if (delay < -cfg.jump_ns)
        {
            reset();  // arrived before it was sent: the clocks jumped
            counters.restarts++;
        }
        else if (delay > cfg.jump_ns)
        {
            if (++jump_streak >= cfg.jump_samples)
            {
                reset();
                counters.restarts++;
            }
            else
            {
                return;  // a stray late datagram, or the start of a jump
            }
        }
        else
        {
            jump_streak = 0;
        }
    }

    if (block_open && host_ns - block_start_ns >= cfg.block_ns)
    {
        closeBlock();
    }
    if (!block_open)
    {
        block_open = true;
        block_start_ns = host_ns;
        block_min = {host_ns, d};
        block_max_delay = 0;
    }
    else if (d < block_min.d)
    {
        block_min = {host_ns, d};
    }
    if (have_fit)
    {
        block_max_delay = std::max(block_max_delay, d - predict(host_ns));
    }
    have_estimate = true;
}

void ClockEstimator::closeBlock()
{
    block_open = false;
    if (have_fit)
    {
        counters.block_min_delay_ns = block_min.d - predict(block_min.host_ns);
        counters.block_max_delay_ns = block_max_delay;

        // a step: several blocks in a row off the line in the same direction
        const int64_t r = counters.block_min_delay_ns;
        const int sign = r > cfg.step_ns ? 1 : (r < -cfg.step_ns ? -1 : 0);
        if (sign != 0 && sign == step_sign)
        {
            step_streak++;
        }
        else
        {
            step_streak = sign != 0 ? 1 : 0;
        }
        step_sign = sign;
        if (step_streak >= cfg.step_blocks)
        {
            // keep only the blocks after the step
            const size_t keep = cfg.step_blocks - 1;
            while (minima.size() > keep)
            {
                minima.pop_front();
            }
            step_streak = 0;
            step_sign = 0;
            counters.steps++;
        }
    }
    minima.push_back(block_min);
    while (minima.size() > cfg.window_blocks)
    {
        minima.pop_front();
    }
    fit();
}

void ClockEstimator::fit()
{
    // least squares through the block minima, relative to the newest one
    // (keeps the doubles small)
    const Point& last = minima.back();
    ref_ns = last.host_ns;
    const size_t n = minima.size();
    double st = 0., sd = 0.;
    for (const auto& p : minima)
    {
        st += static_cast<double>(static_cast<int64_t>(p.host_ns - ref_ns));
        sd += static_cast<double>(p.d - last.d);
    }
    const double mt = st / n, md = sd / n;
    double stt = 0., std_ = 0.;
    for (const auto& p : minima)
    {
        const double t =
            static_cast<double>(static_cast<int64_t>(p.host_ns - ref_ns)) - mt;
        stt += t * t;
        std_ += t * (static_cast<double>(p.d - last.d) - md);
    }
    slope = (n >= 2 && stt > 0.) ? std_ / stt : 0.;
    if (std::abs(slope) > cfg.max_drift)
    {
        slope = 0.;
    }
    // line value at ref_ns (t = 0); the offset itself is ~1e18 ns, too
    // large for a double to hold to the nanosecond
    base_ns = last.d;
    base_frac = md - slope * mt;
    have_fit = true;
}

}  // namespace multiscan
