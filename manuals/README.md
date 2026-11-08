# LucSys Robotics — Development Manuals

English technical references for customers developing and integrating LucSys mobile robot platforms.

[Back to technical resources](../README.md) · [ROS 2 model downloads](../README.md#ros-2-humble-platform-downloads)

## Choose a manual

| Model / topic | Manual | Version / document date | Pages | Download |
| --- | --- | --- | ---: | --- |
| Shared ROS 2 integration reference | [ROS 2 / STM32 Communication Guide](LucSys_ROS2_STM32_Communication_Guide.pdf) | V1.1 · 2025-12-08 | 8 | [PDF](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/manuals/LucSys_ROS2_STM32_Communication_Guide.pdf) |
| LSR100, LSR180*, LSR200 Series, LSR280, LSR300 Mini and LSR300 Pro | [LSR100 / LSR180 / LSR200 / LSR280 / LSR300 — User and Development Manual](LucSys_LSR100_LSR180_LSR200_LSR280_LSR300_User_and_Development_Manual.pdf) | V1.2 · 2026-10-13 | 18 | [PDF](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/manuals/LucSys_LSR100_LSR180_LSR200_LSR280_LSR300_User_and_Development_Manual.pdf) |
| LSR550 series | [LSR550 — STM32 Chassis Development Manual](LucSys_LSR550_STM32_Chassis_Development_Manual.pdf) | V1.0 · 2024-08-28 | 14 | [PDF](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/manuals/LucSys_LSR550_STM32_Chassis_Development_Manual.pdf) |
| LSE550 and LSV550 | [LSE550 / LSV550 — STM32 Chassis Development Manual](LucSys_LSE550_LSV550_STM32_Chassis_Development_Manual.pdf) | V1.1 · 2025-02-19 | 16 | [PDF](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/manuals/LucSys_LSE550_LSV550_STM32_Chassis_Development_Manual.pdf) |

*LSR180 is included in the supplied document grouping, but is not explicitly listed in the manual's preface. Confirm controller and firmware compatibility with LucSys before applying model-specific procedures.

## What each manual covers

- **[ROS 2 / STM32 Communication Guide](LucSys_ROS2_STM32_Communication_Guide.pdf):** USB/serial connectivity, device aliases, ROS 2 nodes, feedback and velocity commands.
- **[LSR100 / LSR180 / LSR200 / LSR280 / LSR300 — User and Development Manual](LucSys_LSR100_LSR180_LSR200_LSR280_LSR300_User_and_Development_Manual.pdf):** C63A chassis hardware, operation, motion control, communication and STM32 firmware.
- **[LSR550 — STM32 Chassis Development Manual](LucSys_LSR550_STM32_Chassis_Development_Manual.pdf):** Chassis hardware, drive kinematics, motor control, STM32 architecture and firmware programming.
- **[LSE550 / LSV550 — STM32 Chassis Development Manual](LucSys_LSE550_LSV550_STM32_Chassis_Development_Manual.pdf):** Educational robot chassis, drive configurations, control interfaces and STM32 firmware programming.

## Reading and downloading

Select a manual title to open the GitHub PDF preview, or select **PDF** to open the file directly and save a local copy. Start with the manual for your chassis, then refer to the ROS 2 / STM32 Communication Guide for host-to-controller integration.

Document dates above are the revision dates printed inside the PDFs. Match the controller revision, firmware, wiring and ordered configuration to your delivered robot before using model-specific instructions. Manuals describe development interfaces; the downloadable URDF packages are visualization models and do not include hardware drivers or STM32 firmware.

For configuration-specific support, contact [info@lucsysrobotics.com](mailto:info@lucsysrobotics.com).
