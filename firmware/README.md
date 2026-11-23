# LucSys Robotics — STM32 source references

Complete source projects are provided for study and configuration-specific development. Select the controller, encoder variant and target before building. A source snapshot is not proof of the firmware installed in a delivered robot.

| Controller branch | Snapshot | Project and development guide | Scope |
|---|---|---|---|
| C50C multi-target | 2026-11-04 | [Source / build / PID locations](controllers/C50C/20261104/README.md) | LSR550 family reference; differential, 4WD, Ackermann, Mecanum and Omni targets; exact installed revision must match |
| C30D 2.0 GMR | 2026-09-21 | [Source / build / encoder locations](controllers/C30D/v2-gmr-20260921/README.md) | Board and encoder-specific reference associated with LSE550/LSV550; not the Hall variant |
| C63A S branch | 2026-01-27 | [Source / build](controllers/C63A/s-series-20260127/README.md) | S-series controller reference; inspect mode selection and delivered hardware |
| C63A T branch | 2026-10-13 | [Source / build](controllers/C63A/t-series-20261013/README.md) | Explicit T280 selector in source; not a universal replacement for S branch |

LSR550A V2/V2.1 and LSR200Auto installed-firmware mappings are not established by these downloads. Do not flash a different branch because the MCU or chassis looks similar. Historical versions, special Hall/open-loop and eight-drive projects are not bundled indiscriminately.

## Build and verification

All four projects select ARM Compiler 5.06 update 7 in Keil. Configured source files exist and local include references were checked. Source tokens, constants and protocol strings match the originals after the documented naming transformation; no PID gains, motor limits, emergency-stop logic or communication-loss policy was changed.

Keil MDK compilation and flashing were **not run** on macOS. The C63A T-branch host test was compiled with Clang and passed seven kinematics groups plus RC-capture and OLED/gamepad regressions. These tests do not compile the complete STM32 firmware and are not vehicle tests. The ROS 2 packages have a separate real Humble build and virtual-serial test.

Third-party ST, ARM and FreeRTOS copyright/license text remains in place. LucSys Robotics prepares and maintains this distribution. No blanket new open-source license is applied.

[ROS 2 driver / Quick Start](../ros2/README.md) · [Electrical revisions](../electrical/controllers/README.md)
