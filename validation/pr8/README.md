# PR #8 customer-package verification — 2026-11-23

**Decision: keep draft; not a fully verified combined ROS/STM32 production release.** No merge, history rewrite or Release replacement was performed. The scoped ROS build and virtual runtime checks pass. Keil target builds and physical-robot checks remain unverified. Existing compiler/static-analysis warnings are recorded, not suppressed.

## Distribution boundary

New customer archives contain only the explicit roots in `customer-assets.json`: seven firmware archives, one ROS driver archive, and 26 model archives. The normal driver does not read a legacy map, substitute an old device path or infer a model. The default model string is empty; explicitly select the delivered configuration. Existing numeric transforms for the six retained LucSys profiles are unchanged. Other historical profiles are retained only in the optional overlay; they are not guessed into LucSys mappings.

`compatibility/ros2-legacy-overlay` is a separate opt-in migration overlay with its own old launch tests. It is not installed by customer builds or included in customer archives. Repository audit files intentionally retain evidence of old names. **GitHub's automatic whole-repository ZIP and a Git clone include these materials and history; they are not the clean customer artifact.** Do not present the repository as having no historical supplier traces.

Third-party ST, ARM/CMSIS, FreeRTOS, serial and URDF exporter notices remain. The LSR180 NOTICE's malformed exporter URL was corrected without removing author/version attribution. The LSR280 host regression script now resolves `gcc.exe` from PATH or an explicit argument instead of a personal drive path. Its Windows execution remains unverified.

## Results and limits

| Check | Result | Limit |
|---|---|---|
| Fresh Ubuntu 22.04 / ROS Humble ARM64 dependencies | PASS via public rosdep/apt | Network and public dependencies required; not Windows/x86 validation |
| ROS source build | PASS, 29 packages | Includes all 26 model packages, not unbundled optional sensors |
| Build and run directly from final ZIPs | PASS, 29 packages; base/full runtime matches | Empty build/install workspace; same public dependency container |
| Registered colcon tests | 246 reported, 0 errors/failures, 53 skipped | ament skips cppcheck 2.7 for upstream performance issues |
| Direct cppcheck | Completed, two warnings | Existing third-party alloca calls; default preprocessor configuration limit, not exhaustive |
| Compiler diagnostics | Build succeeds with warnings | Three third-party serial warnings and two existing low-byte conversion warnings; no claim of warning-free code |
| Model launch resolution | PASS, 37 configurations | XML/installed mesh paths and launch actions, not GUI rendering or physics |
| Base and EKF/model launch | PASS | Actual processes with pseudo-terminal, LSR550A Mecanum model; no robot attached |
| Commands and feedback | PASS | Four positive/negative/multi-axis/zero commands, XOR frames, odometry/IMU/voltage, silent peer |
| Old-deployment overlay | 6 tests pass | Unit/launch contract regression, not legacy physical devices |
| Customer archive scan | 34 ZIPs; zero known identity/private-path matches | Text and filenames incl. UTF-8/GB18030/UTF-16; binary STL headers, not numerical geometry |
| Firmware unchanged | 1,992 source files byte-identical to d5d5927 | Identity cleanup from main was validated separately; this check is not compilation |
| Keil references | 9 projects, 21 targets, 1,542 source references resolve | Actual compiler/device-pack resolution still requires Windows |
| STM32 compilation | **NOT RUN** | All listed targets unverified; see build instructions below |
| Physical robot/safety/optional sensors | **NOT RUN** | No delivered hardware/firmware matching, stopping-distance or safety validation |

The historical inverse-square-root portability correction in d5d5927 remains: defined 32-bit memcpy instead of float/long pointer aliasing, preserving its constant and Newton step. The registered sanitizer test covers 10,001 positive inputs. This is an explicit correctness change, not merely renaming. This follow-up changes no protocol/control C++ beyond removing the unused implicit model default.

`runtime-main.json` was obtained by rebuilding public main c3de982 and running its node directly with explicit serial port and a neutral node-name remap. The candidate was launched through its normal base/full launch entry. Expected wire frames and measured feedback match exactly. `baseline-*.json` records the immediately preceding d5d5927 candidate; its base/full results also match. Timing and physical dynamics are not inferred from these virtual tests.

## Reproduction

Use the candidate checkout, Python 3, Docker, and public internet access. Build archives outside the repository:

```bash
python3 validation/pr8/package_customer.py /tmp/lucsys-customer-packages
python3 validation/pr8/scan_package.py /tmp/lucsys-customer-packages /tmp/package-scan.json
```

Extract the driver ZIP and desired model ZIPs into an empty workspace `src`. The ROS test used `ros:humble` image ID `sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855` with no old build/install cache or private repository mount. Inside the container:

```bash
source /opt/ros/humble/setup.bash
apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --executor sequential --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
python3 /path/to/validation/pr8/runtime_regression.py --output /tmp/base.json
python3 /path/to/validation/pr8/runtime_regression.py --full --output /tmp/full.json
```

The full test requires the included LSR550A Mecanum model package. The overlay contract test is separate: `python3 -m pytest /path/to/compatibility/ros2-legacy-overlay/lucsys_chassis_driver/test/test_launch_contracts.py`. Logs and local candidate ZIPs are retained locally; manifests and machine-readable summaries are committed here. `check_model_launches.py` uses the recorded `/audit/ws/install` container layout and validates all 37 installed model variants.

For native target compilation use [Windows/Keil instructions](../../firmware/build/README.md), [exact project inventory](../../firmware/build/keil-projects.json), and [sequential build script](../../firmware/build/build-all.ps1). All projects specify ARM Compiler 5.06 update 7 build 960 and STM32F4xx_DFP 2.9.0 or 3.1.1. The script is provided for reproduction and has **not** been executed on Windows. Keil commands follow the [official documentation](https://www.keil.com/support/man/docs/uv4/uv4_commandline.asp).

## Public exposure remains

The before-update branch audit is in `public-branch-exposure.json`. Nine public branch heads were checked; every one has matching files, including main (25) and previous PR head (3). The new PR keeps old names only in its independent overlay/audit evidence, but its diff, earlier commits and original base remain publicly viewable. `history-exposure.txt` records matching change commits, including c06975b, cf235fb, c3de982 and f163e6d. No Git history was rewritten.

Four existing source Release sets were freshly downloaded and scanned: `source-2026-11-23-en` (57 matching entries), `source-2026-11-23-lucsys-models` (41), `source-2026-11-23` (45), and `lsr550a-source-2026-11-23` (16). See `release-exposure.json`. These remain downloadable. The older model Release was inventoried; its current source NOTICE contains the malformed original path in LSR180, but its 26 remote ZIP bodies were not redownloaded in this run. No claim is made that historical model assets, external caches or clones have been erased. The corrected 26 local customer model ZIPs were scanned.

## Release gate

The customer archive identity-cleanup criterion passes for the known identifier set. Scoped ROS build and virtual runtime criteria pass with disclosed warnings/skips. The combined “ROS 2 and STM32 all run normally” claim does not pass: native Keil builds, exact delivered configuration matching and hardware tests remain outstanding. Keep PR #8 draft and existing Releases unchanged.
