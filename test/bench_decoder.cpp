/*  Timing of the decoder and frame assembler on the captured fixtures. Not a
 *  test; run it on the target:
 *
 *    build/multiscan_driver/bench_decoder [iterations]
 *
 *  Reference (Ryzen, 12 cores, before the vendored parser was removed), per
 *  segment: old driver 52 us (live, 3456 B) / 180 us (all layers, 3 echoes,
 *  14160 B); this code 8.2 / 19.7 us including assembly, 6.9 / 16.4 us decode.
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "support/mscap.hpp"

namespace compact = multiscan::compact;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv)
{
    const int iters = argc > 1 ? std::atoi(argv[1]) : 20;

    for (const char* file :
         {"live_1echo.mscap.gz", "all_layers_3echo.mscap.gz"})
    {
        std::vector<ms_test::CapturedDatagram> scans;
        for (auto& d : ms_test::readCapture(ms_test::fixture(file)))
        {
            if (compact::telegramType(d.data) == compact::TelegramType::Scan)
            {
                scans.push_back(std::move(d));
            }
        }
        const double n = static_cast<double>(scans.size()) * iters;

        compact::ScanDecoder dec(multiscan::PointLayout::all());
        std::vector<multiscan::FrameAssembler::Frame> frames;
        size_t clouds_new = 0;
        auto t0 = Clock::now();
        for (int i = 0; i < iters; i++)
        {
            // a fresh assembler per pass: the captures' frame numbers repeat
            multiscan::FrameAssembler asmb(
                dec,
                {},
                [](const compact::ScanInfo&, uint64_t) -> int64_t
                { return 0; });
            for (const auto& d : scans)
            {
                if (asmb.add(d.data, d.receive_time_ns, frames) !=
                    compact::Error::None)
                {
                    return 1;
                }
                clouds_new += frames.size();
                frames.clear();
            }
        }
        const double us_new =
            std::chrono::duration<double, std::micro>(Clock::now() - t0)
                .count() /
            n;

        // decoder alone, into one reused buffer: what the assembler adds is the difference
        std::vector<uint8_t> scratch(4096 * 52);
        t0 = Clock::now();
        for (int i = 0; i < iters; i++)
        {
            for (const auto& d : scans)
            {
                compact::ScanInfo info;
                if (dec.decode(d.data, scratch.data(), 4096, info) !=
                    compact::Error::None)
                {
                    return 1;
                }
            }
        }
        const double us_dec =
            std::chrono::duration<double, std::micro>(Clock::now() - t0)
                .count() /
            n;
        std::printf("%-26s decoder alone %6.2f us/segment\n", "", us_dec);
        std::printf(
            "%-26s %5zu B/segment: %6.2f us/segment, %.3f ms per 20 Hz frame "
            "(%zu clouds)\n",
            file,
            scans[0].data.size(),
            us_new,
            us_new * 12 / 1000,
            clouds_new);
    }
}
