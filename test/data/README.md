# Test fixtures

Raw UDP captures from the team's multiScan136 (serial 23170008, firmware 2.3.0),
recorded on 2026-11-22 with `tools/mscap_capture.py` and trimmed to whole frames
with `tools/mscap_trim.py`. Format: see `test/support/mscap.hpp`. Scan data
arrived on port 2115 and IMU data on port 2116.

| File | Sensor settings | Contents |
|---|---|---|
| `live_1echo.mscap.gz` | layer filter on (layers 6, 14 off), echo filter last (`FREchoFilter 2`) — the team default | 10 frames + IMU |
| `all_layers_3echo.mscap.gz` | layer filter off, all echoes (`FREchoFilter 1`) | 4 frames + IMU |
| `range_pm10.mscap.gz` | as live, `LMPoutputRange 1 +1250 -100000 +100000` (-10..+10 deg) | 10 frames, 6 segments each, scan port only |
| `interlaced.mscap.gz` | as live, `sMN mCLsetscancfglist 1` | 9 frames + IMU |

All captures use the Compact format (`ScanDataFormat 2`) with
`ScanDataPreformatting 1`, sent over UDP.
