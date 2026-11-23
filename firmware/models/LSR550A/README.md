# LucSys Robotics — LSR550A STM32 version references

[LSR550A resources](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/LSR550A/README.md)

These are the LSR550A archives supplied by LucSys, cross-checked against their bundled version note, Keil projects and existing LSR550A documentation. They are separate from the LSR550 source project. Source availability is established; the firmware installed on a specific delivered robot is **not yet identified**.

| Version | Snapshot date | Source / compiler / build instructions | Download |
| --- | --- | --- | --- |
| V1 | 2025-12-04 | [Source and build instructions](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/models/LSR550A/v1-20251204/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550a_v1-20251204.zip) |
| V2 | 2026-10-12 | [Source and build instructions](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/models/LSR550A/v2-20261012/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550a_v2-20261012.zip) |
| V2.1 | 2026-10-12 | [Source and build instructions](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/models/LSR550A/v2.1-20261012/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550a_v2_1-20261012.zip) |

[ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt)

## Select the version before building

- V1: the supplied version note specifies a wired PS2 controller. There are separate 4WD, Mecanum and differential projects; only select a project whose hardware/configuration is confirmed.
- V2: the supplied version note specifies a multimode USB wireless controller. Its Keil project contains five chassis targets.
- V2.1: a separate archive with five chassis targets and hardware-version selection code. The short supplied version note does not define the precise V2-to-V2.1 hardware migration or prove interchangeability. Do not choose it solely because its version number is higher.
- Still needed for a particular robot: delivered firmware version/build identification, board revision, chassis/motor/encoder configuration and confirmation of the arm-interface revision. A matching hand controller alone is insufficient.

All projects specify STM32F407VE and ARM Compiler 5.06 update 7 (build 960).  These archives include chassis-side arm integration, not a claim that all separate arm-actuator firmware source is included. See each version's build guide and the [LSR550A development manual](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LSR550A_Development_Manual.pdf), PDF pages 28–29, for programming-interface context.
