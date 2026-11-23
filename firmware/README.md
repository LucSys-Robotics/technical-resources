# LucSys Robotics — STM32 source references

[Find all development resources for your model](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/README.md)

Complete source projects are provided for study and configuration-specific development. Select the controller, encoder variant and target before building. A source snapshot is not proof of the firmware installed in a delivered robot.

| Controller branch | Snapshot | Project and development guide | Scope |
|---|---|---|---|
| C50C multi-target | 2026-11-04 | [Source / build / PID locations](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C50C/20261104/README.md) | LSR550 family reference; differential, 4WD, Ackermann, Mecanum and Omni targets; exact installed revision must match |
| C30D 2.0 GMR | 2026-09-21 | [Source / build / encoder locations](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C30D/v2-gmr-20260921/README.md) | Board and encoder-specific reference associated with LSE550/LSV550; not the Hall variant |
| C63A — LSR100 / LSR180 / LSR200 / LSR300 | 2026-01-27 | [Source / build](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C63A/lsr100-lsr180-lsr200-lsr300-20260127/README.md) | LSR100 / LSR180 / LSR200 / LSR300; select the matching mode and delivered hardware |
| C63A — LSR280 | 2026-10-13 | [Source / build](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C63A/lsr280-20261013/README.md) | LSR280-specific project; do not substitute it for another LucSys model |

[LSR550A V1 / V2 / V2.1 source and build guides](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/models/LSR550A/README.md) are available as versioned references. LSR200Auto shares the LSR200 reference following LucSys confirmation on 2026-11-23. The actual installed firmware for either model remains to be matched. Do not flash a different branch because the MCU or chassis looks similar. Historical versions, special Hall/open-loop and eight-drive projects are not bundled indiscriminately.

## Build and verification

The controller projects and LSR550A references select ARM Compiler 5.06 update 7 in Keil. Configured source files exist and local include references were checked. Source tokens, constants and protocol strings match the originals after the documented naming transformation; no PID gains, motor limits, emergency-stop logic or communication-loss policy was changed.



Third-party ST, ARM and FreeRTOS copyright/license text remains in place. LucSys Robotics prepares and maintains this distribution. No blanket new open-source license is applied.

[ROS 2 driver / Quick Start](https://github.com/LucSys-Robotics/technical-resources/blob/main/ros2/README.md) · [Electrical revisions](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/controllers/README.md)
