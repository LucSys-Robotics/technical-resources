# LSR280 Development Resources

[All robot models](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/README.md) · [Development FAQ](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md) · [Technical documentation](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/README.md)

## Product Information & Ordering

- [LSR280 Four-Wheel Independent Steering ROS Research Robot Platform](https://shop.lucsysrobotics.com/products/lsr280-four-wheel-steering-ros-robot)

Use the product page to choose the hardware package and ordering options. Match the source target and electrical revision to that configuration before building or connecting hardware.

## STM32 Firmware Source

Available source: C63A / 2026-10-13.

| Resource | Link |
| --- | --- |
| STM32 source | [Browse project](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C63A/lsr280-20261013) |
| STM32 download | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr280_20261013.zip) |
| Compiler, target selection and flashing instructions | [Project guide](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C63A/lsr280-20261013/README.md#build-and-flash) |

This is the LSR280 reference. Do not substitute the LSR100 / LSR180 / LSR200 / LSR300 branch.

The selected project uses Keil MDK / ARM Compiler 5.06 update 7. Use the source project to build for the selected hardware.

## ROS 2 Driver

[Download the ROS 2 Humble driver ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_ros2_humble_20260903.zip) · [Setup and interfaces](https://github.com/LucSys-Robotics/technical-resources/blob/main/ros2/chassis-drivers/humble-20260903/README.md)

Includes the chassis driver, message definitions and serial dependency for Ubuntu 22.04 / ROS 2 Humble. Use the controller's matching serial protocol; chassis kinematics and encoder calibration are configured in STM32 firmware.

## URDF & Mesh Downloads

| Configuration | Model source and build / preview instructions | Download |
| --- | --- | --- |
| LSR280 | [lucsys_lsr280_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr280_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr280_description.zip) |

These packages provide robot geometry and display launch files. Install the separate ROS 2 driver above for the chassis interface.

## Manuals & Electrical Connections

- [Development manual](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_LSR100_LSR180_LSR200_LSR280_LSR300_User_and_Development_Manual.pdf)
- [Electrical connection and revision references](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LSR180-LSR200-LSR280-LSR300/README.md)
- [Controller schematics and selection](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/controllers/README.md)
- [ROS 2 / STM32 communication protocol](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_ROS2_STM32_Communication_Guide.pdf)

Use drawings for the actual controller and hardware revision. A controller schematic is not a complete as-built robot wiring diagram. Verify directions, encoder feedback and stop behavior during controlled commissioning; software zero velocity is not physical power isolation.

[SHA256 checksums for all source and model ZIPs](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt). Public packages are directly downloadable; see the [FAQ](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md) for source contents, setup and integration boundaries.
