/*  Timing of the old data path (oracle: vendored parser + driver packing) vs the
 *  new decoder on the captured fixtures. Not a test; run it on the target:
 *
 *    build/multiscan_driver/bench_decoder [iterations]
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "compact/decoder.hpp"
#include "frame_assembler.hpp"
#include "oracle/legacy_pipeline.hpp"
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

        auto t0 = Clock::now();
        size_t clouds_old = 0;
        for (int i = 0; i < iters; i++)
        {
            ms_test::legacy::Pipeline oracle;
            for (const auto& d : scans)
            {
                oracle.feed(d.data, d.recv_time_ns);
            }
            clouds_old += oracle.clouds.size();
        }
        const double us_old =
            std::chrono::duration<double, std::micro>(Clock::now() - t0)
                .count() /
            n;

        compact::ScanDecoder dec(multiscan::PointLayout::all());
        std::vector<multiscan::FrameAssembler::Frame> frames;
        size_t clouds_new = 0;
        t0 = Clock::now();
        for (int i = 0; i < iters; i++)
        {
            // a fresh assembler per pass: the captures' frame numbers repeat
            multiscan::FrameAssembler asmb(
                dec,
                {},
                [](uint64_t us, uint64_t) { return us * 1000; });
            for (const auto& d : scans)
            {
                if (asmb.add(d.data, d.recv_time_ns, frames) !=
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
            "%-26s %5zu B/segment: old %7.2f us/segment, new %6.2f us/segment (%.1fx), "
            "%.3f -> %.3f ms per 20 Hz frame (%zu/%zu clouds)\n",
            file,
            scans[0].data.size(),
            us_old,
            us_new,
            us_old / us_new,
            us_old * 12 / 1000,
            us_new * 12 / 1000,
            clouds_old,
            clouds_new);
    }
}
