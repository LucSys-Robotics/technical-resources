# LucSys Robotics — LSR280 host kinematics tests

This PC-side test compiles the actual `LUCSYS_BSP/4ws_v2.c` source from this snapshot, without copying or rewriting its kinematics. It tests that source revision; it does not establish which firmware is installed on a delivered robot.

Run it after changing kinematics, before vehicle testing.

## Usage

From the firmware project root:

```bash
cd tools/4ws_sim
make run        # 200,000 samples: routine check
make quick      # 10,000 samples: quick development check
make full       # 2,000,000 samples: extended comparison
```

Specify a sample count and random seed directly for reproducibility:

```bash
./4ws_sim 500000 42
```

Exit code 0 means all checks passed; 1 means at least one check failed. The command can be used in CI or a commit hook.

## Five documented invariants

| Test | Check | Interpretation of a failure |
| --- | --- | --- |
| T1 Round trip | Inverse kinematics → assume wheels reach their targets → forward kinematics; recover all three velocity axes | A kinematic inconsistency |
| T2 Angle range | Wheels A/B: [−50°, 220°]; wheels C/D: [−220°, 50°] | A target outside the limits represented by this test |
| T3 Projection consistency | `WheelSpeedProjection` and `InverseKinematics` must produce matching wheel speeds | Steering-angle and wheel-speed signs may disagree, causing an abrupt wheel reversal |
| T4 Vector continuity | With continuous input, `speed × (cos a, sin a)` must not jump | An unnecessary steering-target discontinuity, potentially observed as jitter |
| T5 Hold at zero speed | With all three velocity inputs zero, retain the previous angles and set only wheel speed to zero | Unwanted steering movement while stopped |

### Why T3 and T4 matter

The original review identifies T3 as item 7: the firmware takes steering angles from `four_ws_InverseKinematics` and wheel speeds from `four_ws_WheelSpeedProjection`. Both must agree for the same input. The host test exercises reversal boundaries without requiring a vehicle for every check.

T4 checks the velocity vector, not just the angle. A 180° target-angle reversal with the wheel-speed sign reversed is an equivalent representation and can shorten steering travel. The vector remains unchanged, so this valid reversal must not be flagged as jitter.

T4 scans five transitions: straight motion ↔ rotation, straight motion ↔ lateral motion, low speed with rotation, reverse ↔ forward, and diagonal sweeps. The original development note reports the most pronounced vehicle jitter in the low-speed/rotation case; this is not a new measurement.

## Thresholds

Defined near the top of `main.c`:

```c
#define EPS_ROUNDTRIP   1e-3f    // Round-trip error: m/s and rad/s
#define EPS_PROJECTION  1e-4f    // Wheel-speed difference between the two methods
#define ANGLE_MARGIN    0.5f     // Angle-limit margin, degrees
#define EPS_CONTINUITY  0.05f    // Velocity-vector continuity, m/s
```

Track width, wheelbase and steering-radius parameters are also near the top of `main.c`. Keep them consistent with the applicable LSR280 definition in `robot_select_init.h`. A different configuration requires matching parameters; do not infer them from another model.

## Build

The kinematics test uses `stdint.h` and `math.h`; no hardware stubs are required for this pure-kinematics path.

The original source uses GBK/GB18030 comments. The original build note reports that MinGW 8.1 without iconv fails with `-finput-charset=GBK` (`failure to convert GBK to UTF-8`), while its default input handling accepts those comment bytes. Check your compiler and editor encoding. The English edition translates comments without changing executable tokens.

The original scan checked for a GBK character whose final byte is `0x5C` at the end of a `//` line, which can accidentally splice the following line. Translation validation also preserves the original line-continuation structure.

### Windows / MinGW

Add your own MinGW `bin` directory to `PATH`. The executable is typically `mingw32-make`. Windows PowerShell 5.1 does not support `&&`; run commands on separate lines.

```powershell
Set-Location "tools/4ws_sim"
mingw32-make run
```

Or compile directly:

```bat
gcc -std=c99 -O2 -Wall -Wextra -I../../LUCSYS_BSP/Inc -o 4ws_sim.exe main.c ../../LUCSYS_BSP/4ws_v2.c -lm
4ws_sim.exe
```

### Linux

```bash
make run
```

## Limitations

This host test covers the pure kinematics in `4ws_v2.c`. It does not verify:

- Control logic in `RobotControl_task_v2.c`, including startup alignment, speed limiting, the deceleration hold gate and command-rate throttling.
- Actual steering-motor tracking under load.
- CAN communications or motor-driver behavior.

Use the scripts in the parent `tools/` directory (`datalog_recv.py`, `datalog_plot.py`, `datalog_compare.py`) to observe those behaviors during an appropriately controlled hardware test. See [the data-logging guide](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C63A/lsr280-20261013/tools/datalog_README.md). Host-test success is not vehicle or safety validation.
