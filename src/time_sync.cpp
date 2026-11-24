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

ClockEstimator::ClockEstimator(Config config) : config{config} {}

void ClockEstimator::reset()
{
    this->block_open = false;
    this->block_minima.clear();
    this->step_streak = 0;
    this->step_sign = 0;
    this->jump_streak = 0;
    this->has_estimate = false;
    this->has_fit = false;
    this->fit_slope = 0.;
}

int64_t ClockEstimator::offsetNs(uint64_t host_ns) const
{
    if (!this->has_estimate)
    {
        return 0;
    }
    if (!this->has_fit)
    {
        // first block: its running minimum
        return this->block_minimum.difference_ns;
    }
    return this->predict(host_ns);
}

void ClockEstimator::add(uint64_t sensor_us, uint64_t host_ns)
{
    const int64_t difference_ns =
        static_cast<int64_t>(host_ns) - static_cast<int64_t>(sensor_us * 1000);
    this->statistics.samples++;
    if (this->acceptSample(difference_ns, host_ns))
    {
        this->addToBlock(difference_ns, host_ns);
    }
}

/* Detects clock jumps against the estimate. Returns false for a sample that
 * is ignored (a stray late datagram, or the start of a jump). */
bool ClockEstimator::acceptSample(int64_t difference_ns, uint64_t host_ns)
{
    if (!this->has_estimate)
    {
        return true;
    }
    const int64_t delay_ns = difference_ns - this->offsetNs(host_ns);
    if (delay_ns < -this->config.jump_ns)
    {
        // arrived before it was sent: the clocks jumped
        this->reset();
        this->statistics.restarts++;
    }
    else if (delay_ns > this->config.jump_ns)
    {
        if (++this->jump_streak < this->config.jump_samples)
        {
            return false;
        }
        this->reset();
        this->statistics.restarts++;
    }
    else
    {
        this->jump_streak = 0;
    }
    return true;
}

void ClockEstimator::addToBlock(int64_t difference_ns, uint64_t host_ns)
{
    if (this->block_open &&
        host_ns - this->block_start_ns >= this->config.block_ns)
    {
        this->closeBlock();
    }
    if (!this->block_open)
    {
        this->block_open = true;
        this->block_start_ns = host_ns;
        this->block_minimum = {host_ns, difference_ns};
        this->block_max_delay_ns = 0;
    }
    else if (difference_ns < this->block_minimum.difference_ns)
    {
        this->block_minimum = {host_ns, difference_ns};
    }
    if (this->has_fit)
    {
        this->block_max_delay_ns = std::max(
            this->block_max_delay_ns,
            difference_ns - this->predict(host_ns));
    }
    this->has_estimate = true;
}

void ClockEstimator::closeBlock()
{
    this->block_open = false;
    if (this->has_fit)
    {
        this->statistics.block_min_delay_ns =
            this->block_minimum.difference_ns -
            this->predict(this->block_minimum.host_ns);
        this->statistics.block_max_delay_ns = this->block_max_delay_ns;
        this->detectStep();
    }
    this->block_minima.push_back(this->block_minimum);
    while (this->block_minima.size() > this->config.window_blocks)
    {
        this->block_minima.pop_front();
    }
    this->fit();
}

/* A step: several blocks in a row off the line in the same direction. Only
 * the blocks after it are kept. */
void ClockEstimator::detectStep()
{
    const int64_t residual_ns = this->statistics.block_min_delay_ns;
    const int sign = residual_ns > this->config.step_ns
                         ? 1
                         : (residual_ns < -this->config.step_ns ? -1 : 0);
    if (sign != 0 && sign == this->step_sign)
    {
        this->step_streak++;
    }
    else
    {
        this->step_streak = sign != 0 ? 1 : 0;
    }
    this->step_sign = sign;

    if (this->step_streak >= this->config.step_blocks)
    {
        const size_t keep = this->config.step_blocks - 1;
        while (this->block_minima.size() > keep)
        {
            this->block_minima.pop_front();
        }
        this->step_streak = 0;
        this->step_sign = 0;
        this->statistics.steps++;
    }
}

/* Least squares through the block minima, relative to the newest one
 * (keeps the doubles small). */
void ClockEstimator::fit()
{
    const BlockMinimum& newest = this->block_minima.back();
    this->fit_reference_ns = newest.host_ns;
    const auto timeOf = [this](const BlockMinimum& minimum)
    {
        return static_cast<double>(
            static_cast<int64_t>(minimum.host_ns - this->fit_reference_ns));
    };
    const auto differenceOf = [&newest](const BlockMinimum& minimum)
    {
        return static_cast<double>(
            minimum.difference_ns - newest.difference_ns);
    };

    const size_t num_minima = this->block_minima.size();
    double sum_time = 0.;
    double sum_difference = 0.;
    for (const BlockMinimum& minimum : this->block_minima)
    {
        sum_time += timeOf(minimum);
        sum_difference += differenceOf(minimum);
    }
    const double mean_time = sum_time / num_minima;
    const double mean_difference = sum_difference / num_minima;

    double sum_time_squared = 0.;
    double sum_time_difference = 0.;
    for (const BlockMinimum& minimum : this->block_minima)
    {
        const double time = timeOf(minimum) - mean_time;
        sum_time_squared += time * time;
        sum_time_difference += time * (differenceOf(minimum) - mean_difference);
    }
    this->fit_slope = (num_minima >= 2 && sum_time_squared > 0.)
                          ? sum_time_difference / sum_time_squared
                          : 0.;
    if (std::abs(this->fit_slope) > this->config.max_drift)
    {
        this->fit_slope = 0.;
    }
    // line value at fit_reference_ns (t = 0); the offset itself is ~1e18 ns,
    // too large for a double to hold to the nanosecond
    this->fit_base_ns = newest.difference_ns;
    this->fit_base_fraction_ns = mean_difference - this->fit_slope * mean_time;
    this->has_fit = true;
}

int64_t ClockEstimator::predict(uint64_t host_ns) const
{
    const double elapsed_ns = static_cast<double>(
        static_cast<int64_t>(host_ns - this->fit_reference_ns));
    return this->fit_base_ns +
           static_cast<int64_t>(std::llround(
               this->fit_base_fraction_ns + this->fit_slope * elapsed_ns));
}

}  // namespace multiscan
