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

Other files:

| File | Contents |
|---|---|
| `sopas_transcript_fw2.3.txt` | CoLa A requests and replies recorded from the same sensor (settings, status, time sync); the fake sensor in `test/support/fake_sensor.hpp` starts from its values |
| `ghost_full_30trees.model.txt`, `ghost_xyz_30trees.model.txt` | The first 30 of 600 trees of the ghost-ml realtime models (trained 2026-11-18), with two made-up early exit checkpoints, for `test_ghost_filter`. They exercise the code, not the classification: use the real models to filter |
| `golden_clouds.txt` | Size and CRC-32 of every cloud the old sick_scan_xd-based driver published for the captures and the synthetic configurations in `test_decoder.cpp`, recorded before that code was removed. Regenerate only for intentional output changes: `MS_UPDATE_GOLDEN=1 build/multiscan_driver/test_decoder` |
