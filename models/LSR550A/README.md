# LSR550A Development Resources

[All robot models](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/README.md) · [Development FAQ](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md) · [Technical documentation](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/README.md)

## Product Information & Ordering

- [LSR550A Mobile Manipulation Robot Platform with Robotic Arm](https://shop.lucsysrobotics.com/products/mobile-robot-arm-platform-lsr550a)

Use the product page to choose the hardware package and ordering options. Match the source target and electrical revision to that configuration before building or connecting hardware.

## STM32 Firmware Source

Complete LSR550A V1, V2 and V2.1 chassis projects are available; select the version matching the delivered controller.

[Open the LSR550A STM32 version selector](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/models/LSR550A/README.md) for V1 (2025-12-04), V2 and V2.1 (2026-10-12), source ZIPs and version-specific Keil build instructions. These are LSR550A archives, not substitute LSR550 code. V1 is associated with wired PS2; V2 with multimode USB wireless control. The exact V2/V2.1 hardware-selection conditions still require confirmation.

## ROS 2 Driver

[Download the ROS 2 Humble driver ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_ros2_humble_20260903.zip) · [Setup and interfaces](https://github.com/LucSys-Robotics/technical-resources/blob/main/ros2/chassis-drivers/humble-20260903/README.md)

Includes the chassis driver, message definitions and serial dependency for Ubuntu 22.04 / ROS 2 Humble. Use the controller's matching serial protocol; chassis kinematics and encoder calibration are configured in STM32 firmware.

## URDF & Mesh Downloads

| Configuration | Model source and build / preview instructions | Download |
| --- | --- | --- |
| LSR550A-4WD | [lucsys_lsr550a_4wd_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550a_4wd_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550a_4wd_description.zip) |
| LSR550A-MEC | [lucsys_lsr550a_mec_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550a_mec_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550a_mec_description.zip) |

These packages provide robot geometry and display launch files. Install the separate ROS 2 driver above for the chassis interface.

## Manuals & Electrical Connections

- [Development manual](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LSR550A_Development_Manual.pdf)
- [C50C controller electrical reference](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/LSR550A/LSR550A_C50C_Controller_Schematic.pdf) (controller schematic; not a complete robotic-arm wiring diagram)
- [Controller schematics and selection](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/controllers/README.md)
- [ROS 2 / STM32 communication protocol](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_ROS2_STM32_Communication_Guide.pdf)

Use drawings for the actual controller and hardware revision. A controller schematic is not a complete as-built robot wiring diagram. Verify directions, encoder feedback and stop behavior during controlled commissioning; software zero velocity is not physical power isolation.

[SHA256 checksums for all source and model ZIPs](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt). Public packages are directly downloadable; see the [FAQ](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md) for source contents, setup and integration boundaries.
