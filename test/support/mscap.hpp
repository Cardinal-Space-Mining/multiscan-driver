/*  Reader for raw multiScan UDP captures ("MSCAP001", optionally gzipped).
 *
 *  File layout (little endian):
 *    header : 8 bytes "MSCAP001"
 *    record : u64 receive_time_ns (CLOCK_REALTIME, kernel receive timestamp)
 *             u16 destination UDP port
 *             u32 datagram length
 *             u8  datagram[length]
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <zlib.h>


namespace ms_test
{

struct CapturedDatagram
{
    uint64_t receive_time_ns = 0;
    uint16_t port = 0;
    std::vector<uint8_t> data;
};

inline std::vector<CapturedDatagram> readCapture(const std::string& path)
{
    gzFile f = gzopen(path.c_str(), "rb");  // also reads uncompressed files
    if (!f)
    {
        throw std::runtime_error("cannot open capture " + path);
    }
    std::vector<uint8_t> bytes;
    uint8_t chunk[1 << 16];
    int n;
    while ((n = gzread(f, chunk, sizeof(chunk))) > 0)
    {
        bytes.insert(bytes.end(), chunk, chunk + n);
    }
    gzclose(f);
    if (bytes.size() < 8 || std::memcmp(bytes.data(), "MSCAP001", 8) != 0)
    {
        throw std::runtime_error("not an MSCAP001 capture: " + path);
    }
    std::vector<CapturedDatagram> out;
    size_t off = 8;
    while (off + 14 <= bytes.size())
    {
        CapturedDatagram d;
        uint32_t len;
        std::memcpy(&d.receive_time_ns, &bytes[off], 8);
        std::memcpy(&d.port, &bytes[off + 8], 2);
        std::memcpy(&len, &bytes[off + 10], 4);
        off += 14;
        if (off + len > bytes.size())
        {
            throw std::runtime_error("truncated capture " + path);
        }
        d.data.assign(bytes.begin() + off, bytes.begin() + off + len);
        off += len;
        out.push_back(std::move(d));
    }
    return out;
}

#ifdef MS_TEST_DATA_DIR
inline std::string fixture(const std::string& name)
{
    return std::string(MS_TEST_DATA_DIR) + "/" + name;
}
#endif

}  // namespace ms_test
