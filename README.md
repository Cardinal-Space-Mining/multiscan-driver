# multiscan_driver

ROS 2 driver for the SICK multiScan136 lidar (tested with firmware 2.3.0). On
every connect it configures the sensor over SOPAS (CoLa A), reads the
configuration back to verify it, then receives the Compact UDP stream and
publishes one point cloud per revolution (20 Hz) plus the built-in IMU
(100 Hz).

Nothing is ever saved on the sensor. A power cycle restores its stored
configuration, which is why the driver applies its settings on every connect.

## Running

```
ros2 run multiscan_driver multiscan_driver --ros-args \
    --params-file config/params.yaml -p lidar_hostname:=10.11.10.3
```

On the robot the `lance` package starts it from
`lance/config/actions/multiscan_driver.json`.

The host needs:

- TCP to the sensor's port 2111 (SOPAS, CoLa A; 2112 is CoLa B, which the
  driver doesn't use).
- UDP 2115 (scans) and 2116 (IMU) open in the firewall. Set `imu_udp_port`
  equal to `lidar_udp_port` to use a single port.
- A large enough socket buffer. The driver asks for 4 MiB
  (`udp_receive_buffer_bytes`) and warns if `net.core.rmem_max` caps it.

The sensor is told to stream to `driver_hostname`. When that is empty, the
driver uses the local address of its connection to the sensor.

## Topics

| Topic | Type | |
|---|---|---|
| `lidar_scan` | `sensor_msgs/PointCloud2` | One revolution per message. Stamp: start of its first segment. |
| `lidar_imu` | `sensor_msgs/Imu` | 100 Hz. Frame `imu.frame_id`, or `lidar_frame` if empty. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz; see [Diagnostics](#diagnostics). |
| `/tf_static` | | `lidar_frame` → `imu.frame_id`, only when `imu.frame_id` is set. |
| `multiscan_driver/process_stats` | `csm_metrics/ProcessStats` | CPU and memory of the node. |

### Point fields

`output.point_fields` picks any subset that contains x, y and z. Fields are
always packed in this order:

| Field | Type | |
|---|---|---|
| `x` `y` `z` | float32 | m, sensor frame |
| `intensity` | float32 | RSSI of the published echo |
| `range` | float32 | m (0 = no return) |
| `azimuth` `elevation` | float32 | rad, as calibrated and sent by the sensor |
| `layer` | uint32 | 0..15 by elevation; 5 and 13 are the 0.125° high-resolution layers |
| `echo` | uint32 | 0..2 |
| `index` | uint32 | beam index within the layer over the whole revolution |
| `tl` `th` | uint32 | low and high halves of the point's time: µs, host clock |
| `reflective` | float32 | 1 if the sensor flagged a reflector |

Clouds are dense, `height` 1. Beams without a return have range 0, and so do
beams outside a narrowed output range when the sensor zero-fills them. The
published echo is `echo_selection` (1-based). If the sensor sends fewer echoes,
the first one it sends is used.

A frame is published as soon as its last expected segment arrives. With
`output.publish_partial_frames` (the default), a frame that is still missing
segments when the next one starts is published as it is. Segments the sensor
never sends, because of a narrowed `sensor.output_range_deg`, are learned and
don't count as missing.

## Parameters

[`config/params.yaml`](config/params.yaml) lists every parameter with its
default and units. In short:

- **Connection:** `lidar_hostname` (required), `driver_hostname`, ports,
  `sopas_password` (the hash of the Authorized Client password; the factory
  default is `F4724744`), timeouts.
- **`sensor.*`:** echo filter, layers, output range, interlaced mode, range
  and cubic filters, particle, fog, moving-average and interval filters,
  contamination monitoring, clock synchronisation. These are applied on every
  connect and read back. A rejected or different value is logged and shown in
  `/diagnostics`; it doesn't stop the stream. Set `sensor.apply: false` to
  only compare the sensor's settings with these. Invalid values switch apply
  off.
- **`time.source`:** see [Timestamps](#timestamps).
- **`imu.*`:** frame, static TF, covariances.
- **`diagnostics.status_period`:** how often the device state, temperature and
  contamination are read.

## Timestamps

The sensor's clock counts from power-up and drifts by a few ppm against the
host. Every scan telegram carries its send time, and the kernel stamps when it
arrived. `time.source: estimated` (the default) fits the host − sensor offset
and drift to the lower edge of those pairs (`src/time_sync.hpp`) and maps all
sensor times through it. The stamps then keep the sensor's exact spacing:
scans come 50.000 ms apart with ~10 µs spread, where receive times spread
~120 µs. A sensor reboot or clock step is detected and the fit restarts.

The remaining error is a constant: the shortest network delay plus however
long before sending the sensor stamps a telegram. It couldn't be measured
without a synchronised sensor clock, but on a direct link it should be well
under a millisecond.

`time.source: receive` uses each telegram's arrival time (jittery).
`time.source: sensor` uses the sensor clock as is, which only makes sense if
the sensor is synchronised. On firmware 2.3:

- NTP (`sensor.time_sync.mode: ntp`, with an NTP server on the host) locks
  within a second. The clock then stays 1–4 ms off and is corrected in steps
  of up to ~2 ms.
- PTP (`ptp`) is accepted, but against a linuxptp master the sensor sent no
  PTP messages and never corrected its time.

Keep `sensor.time_sync.mode: off` with `time.source: estimated`.

## Diagnostics

`/diagnostics` carries six statuses, prefixed `multiscan_driver:`:

| Status | Contents |
|---|---|
| Connection | streaming / connected without data / error; device type, serial, firmware, reconnects |
| Sensor | device state (`DevSta`: WARN unless normal, ERROR for recoverable or fatal error), internal temperature |
| Contamination | warning / error of the optics cover monitoring, state of each 30° sector |
| Stream | frames/s, IMU samples/s, lost telegrams, partial, discarded and missed frames, decode errors |
| Time | source, host − sensor offset, drift, clock restarts |
| Configuration | settings verified, and each setting that wasn't applied or reads back differently |

Contamination is only evaluated when `sensor.contamination.strategy` isn't
`inactive`. Changes in device state and contamination are also logged.

## Tools

- `tools/ms_geometry.py` (installed: `ros2 run multiscan_driver ms_geometry.py`)
  measures every layer's elevation, the azimuth of index 0 and the azimuth step
  from the running driver. These are the sensor's calibrated angles. It prints
  them as `ELEVATION_LUT` / `AZIMUTH_OFFSET_LUT` for
  `include/multiscan_driver/multiscan_spec.hpp` and shows the difference from
  the values there. Run it once per sensor unit.
- `tools/mscap_capture.py OUT SECONDS PORT...` records raw UDP datagrams with
  kernel timestamps. `tools/mscap_trim.py` cuts a capture to whole frames.
  The test fixtures in `test/data` were made with these.

## Development

```
src/compact/        Compact format decoder and the published point layout
src/frame_assembler segments -> frames by frame number; time offset applied here
src/net/            batched UDP receive (recvmmsg, kernel receive timestamps)
src/sopas/          CoLa A client, start/stop/status sequence, sensor settings
src/time_sync       sensor clock -> host clock estimator
src/multiscan_driver.cpp   the ROS node
include/multiscan_driver/multiscan_spec.hpp   dense-buffer geometry used by net_adapter's codec
```

`colcon test --packages-select multiscan_driver` runs everything without a
sensor. CI runs the same on Jazzy and Lyrical (`.github/`).

- `test_decoder` checks the decoder against an independent reader of the
  format (`test/support/compact_packet.hpp`) on captures and synthetic
  configurations. It also checks that the published clouds are byte-identical
  to what the old sick_scan_xd-based driver published
  (`test/data/golden_clouds.txt`). After an intentional output change,
  regenerate that file with `MS_UPDATE_GOLDEN=1 build/multiscan_driver/test_decoder`.
- `test_sopas` runs the start sequence against a simulated sensor
  (`test/support/fake_sensor.hpp`). The simulation behaves like the
  recording of the real device in `test/data/sopas_transcript_fw2.3.txt`.
- `test_frame_assembler`, `test_udp_receiver` and `test_time_sync` cover
  framing, the socket layer and the clock estimator.
- `bench_decoder` (not a test) times decoding on the captures. A full
  revolution takes ~0.1 ms with the default layers.

The decoder reads untrusted network input, so run the tests under ASan and
UBSan after changing it:

```
colcon build --packages-select multiscan_driver --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-sanitize-recover=undefined"
```

### Sensor behaviour worth knowing (firmware 2.3)

- Port 2111 speaks only CoLa A and 2112 only CoLa B. Writes need a login
  (`sMN SetAccessMode 3 <hash>`, else `sFA A`). `sMN Run` applies changes and
  logs out.
- While a client is logged in, `DevSta` reads 2 (service mode). The driver
  logs out once streaming starts, so the state it reports is the real one.
- Replies print numbers in hex, negative values as 32-bit two's complement.
  Writes accept `+dec`, `-dec` or hex.
- `LFPcubicareafilter` and `LFPradialDistanceRangeFilter` take mm (the
  manual says 1/10 mm for the cubic filter). `LFPintervalFilter` rejects
  factor 1 (`sFA 30`). The `temperatureAlarm*` variables in the manual don't
  exist (`sFA 3`). Interlaced mode (`sMN mCLsetscancfglist`) can't be read
  back.
- With a narrowed output range, segments entirely outside it are not sent at
  all. Segments partly inside are sent with zeroed points.
- Interlaced mode shifts every layer's azimuth by 0.125° per frame over 8
  frames.
- The stream timestamps are UTC regardless of the time zone setting.
