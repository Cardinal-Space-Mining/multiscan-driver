# AGENTS.md

Guidance for AI coding agents (and humans) working on `multiscan_driver`, the
ROS 2 driver for the SICK multiScan136. The [README](README.md) explains what
the driver does and how to configure it. This file covers how to work in the
code.

This repository is usually a submodule of the LANCE robot workspace
(`<ws>/src/multiscan-driver`). Run commands from the workspace root. The
workspace's own `AGENTS.md` covers that side.

## Building and testing

```bash
source /opt/ros/lyrical/setup.bash        # or jazzy: both must build
colcon build --symlink-install --packages-select multiscan_driver \
    --cmake-args -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
colcon test --packages-select multiscan_driver
colcon test-result --test-result-base build/multiscan_driver --verbose
```

- The tests need no sensor. They run on recorded captures (`test/data`) and
  a simulated SOPAS sensor (`test/support/fake_sensor.hpp`). Keep them
  passing, and keep the build free of warnings (`-Wall -Wextra -Wpedantic`).
- The decoder reads untrusted network input, and the ghost filter has its
  own thread. After changing either, run the tests under ASan/UBSan
  (`-fsanitize=address,undefined`) and TSan (`-fsanitize=thread`) in a
  separate build base.
- `test_decoder` holds the published clouds byte-identical to
  `test/data/golden_clouds.txt`. A refactor must not change them. Only after
  an intentional output change, regenerate with
  `MS_UPDATE_GOLDEN=1 build/multiscan_driver/test_decoder`.
- `bench_decoder` and `bench_ghost_filter` (not tests) time the decoder and
  the ghost filter.

## Working with the sensor

- **Never persist sensor settings.** Don't send `mEEwriteall`, and avoid
  `mSCreboot`. The driver applies its settings on every connect, and a power
  cycle restores the sensor's stored configuration. Changing settings for a
  test is fine.
- The team sensor is at 10.11.10.3, and the host is 10.11.10.1. For live
  tests, keep ROS traffic local:
  `ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- Say clearly what could only be verified on hardware.

## The ghost filter and ghost-ml-ws

- The classifier, `include/multiscan_driver/artifact_filter/` (header only,
  standard library only), is owned here and installed with the package.
  ghost-ml-ws (the labelling and training tools) builds against it. The
  dependency runs one way: ghost-ml-ws -> driver.
- It mirrors ghost-ml-ws's Python feature code (`artifact_features.py`,
  `scan_grid.py`) exactly. After changing it, run ghost-ml-ws's
  `artifact_filter_check` against a check file. Its results must stay
  identical, not just close.
- The default models in `models/` are published from ghost-ml-ws
  (`annotator_app/publish_model.py`). Don't edit them by hand.
- `test/data/ghost_*_30trees.model.txt` are cut-down models for the tests.
  They exercise the code, not the classification.

## Code style

The reference is the LANCE package (`src/lance/src` of the robot
workspace): code should read like it. These rules apply to all C++ here
and in ghost-ml-ws.

### Formatting

- Format every new or changed C++ file with `clang-format -i` and the
  repository's `.clang-format`, a copy of the LANCE workspace's:
  Google-based, 4-space indent, 80 columns, Allman braces.
- Always use braces, even for one-line `if` and loop bodies.
- Every C++ file starts with the Cardinal Space Mining copyright banner
  (copy it from any file here). A block comment after it says what the file
  is for and anything a reader must know first.
- Use `#pragma once`. In a `.cpp`, include its own header first. Then group
  includes, separated by blank lines: standard library, third-party (ROS,
  zlib, ...), then this project's headers.

### Names

- **Spell names out.** Write `num_points`, `receive_time_ns`, `datagram`,
  `offset`, `probability` and `config`, not `n`, `recv_ns`, `dgram`, `off`,
  `prob` and `cfg`. Readability is worth the column space.
- Exceptions:
  - `i`, `j` and `k` for plain loop indices;
  - `x`, `y` and `z` for coordinates;
  - names from a specification, explained where they appear (e.g. `phi` and
    `theta` from the operating instructions);
  - `a` and `b` in a sort comparator.
- Put the unit at the end of the name: `_ns`, `_us`, `_ms`, `_s`, `_m`,
  `_mm`, `_deg`, `_rad`, `_hz`, `_bytes`. Use `num_` for counts and `has_`
  or `is_` for booleans.
- Casing:
  - types: `PascalCase`;
  - functions and methods: `camelCase` (`addSegment`, `cellAt`);
  - variables and members: `snake_case`;
  - constants: `UPPER_SNAKE_CASE`;
  - enums: `enum class` with `PascalCase` values.
- Name a lambda like a function (`readList`, `isFloat32`).

### Functions

- Keep functions short and about one thing. Break long ones into named steps
  (e.g. `FrameAssembler::add`: decode, move to the host clock, count lost
  telegrams, place the segment, accept it). A function that reads as a list
  of steps is the goal.
- If a function stays long, label its parts with `// --- Step ---` comments.
- Structure control flow plainly: early returns instead of deep nesting, and
  no `goto`.

### Classes and headers

- Group a class with repeated access labels, in this order:
  - `public:` types;
  - `public:` constructors;
  - `public:` methods (more than one block if they group naturally);
  - `protected:` types, then `protected:` methods, then `protected:` members.
- Use `protected` rather than `private` (the LANCE convention).
- Use a `struct` with public members for plain data. Give it methods only if
  they are small helpers.
- Initialise members in the declaration with braces: `uint32_t
  num_points{0};`.
- Always access members through `this->`.
- Group related members with `// --- Group ---` comments, and say which
  thread owns or which mutex guards shared state.
- Put two blank lines between top-level classes or structs, and one between
  closely related small structs.
- Put `/* ... */` doc comments above declarations, saying what something
  means, its units and its edge cases. Use `//` for short inline notes.

### Source files

- Mark the sections of a `.cpp` with `// --- Section ---` banners padded to
  80 columns, with three blank lines before each. Keep helpers in an
  anonymous namespace at the top.
- Log messages from the node start with `LOG_PREFIX` (`"[MULTISCAN DRIVER]:
  "`).

### Comments

- Explain why, not what: the sensor behaviour, the manual section, or the
  reason for an odd choice. Keep sentences short and plain.

### Refactoring numeric code

- Renaming and splitting must not change results. Keep the exact order of
  floating-point operations: `(a * b) / c` and `a * (b / c)` round
  differently. That alone has broken the golden-cloud test.
- To check a refactor, compare outputs byte for byte against the previous
  version on the captures, not just with the tests.

### Files that keep their own style

`include/multiscan_driver/multiscan_spec.hpp` is a public header that
net_adapter's codec uses. Don't rename its API.

## Commits

Commit subjects are short, lower case and imperative ("add bandwidth
monitoring"). Commit inside this repository, push, then update the submodule
pointer in the parent workspace. The user commits; don't commit unless
asked.
