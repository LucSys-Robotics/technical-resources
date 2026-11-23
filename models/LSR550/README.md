# LSR550 Development Resources

[All robot models](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/README.md) · [Development FAQ](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md) · [Technical documentation](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/README.md)

## Product Information & Ordering

- [LSR550 2WD Differential-Drive ROS Research Robot Platform](https://shop.lucsysrobotics.com/products/lsr550-2wd-ros-research-robot-platform)
- [LSR550 Omni-Wheel ROS Research Robot Platform](https://shop.lucsysrobotics.com/products/lsr550-omni-ros-research-robot-platform)
- [LSR550 4WD ROS Research Robot Platform](https://shop.lucsysrobotics.com/products/lsr550-4wd-ros-research-robot-platform)
- [LSR550 Ackermann ROS 2 UGV Research Platform](https://shop.lucsysrobotics.com/products/lsr550-ackermann-ros2-ugv)
- [LSR550 MEC Mecanum ROS 2 Research Robot Platform](https://shop.lucsysrobotics.com/products/ros-research-robot-r550mecanum-wheel)

Use the product page to choose the hardware package and ordering options. Match the source target and electrical revision to that configuration before building or connecting hardware.

## STM32 Firmware Source

Available source: C50C / 2026-11-04.

| Resource | Link |
| --- | --- |
| STM32 source | [Browse project](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C50C/20261104) |
| STM32 download | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_c50c_20261104.zip) |
| Compiler, target selection and flashing instructions | [Project guide](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/controllers/C50C/20261104/README.md#build-and-flash) |

Select the matching controller target, motor/encoder configuration and hardware revision. This family reference does not confirm every suspension/package combination and does not automatically apply to LSR550A.

The selected project uses Keil MDK / ARM Compiler 5.06 update 7. Use the source project to build for the selected hardware.

## ROS 2 Driver

[Download the ROS 2 Humble driver ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_ros2_humble_20260903.zip) · [Setup and interfaces](https://github.com/LucSys-Robotics/technical-resources/blob/main/ros2/chassis-drivers/humble-20260903/README.md)

Includes the chassis driver, message definitions and serial dependency for Ubuntu 22.04 / ROS 2 Humble. Use the controller's matching serial protocol; chassis kinematics and encoder calibration are configured in STM32 firmware.

## URDF & Mesh Downloads

| Configuration | Model source and build / preview instructions | Download |
| --- | --- | --- |
| LSR550-2WD | [lucsys_lsr550_2wd_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_2wd_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_2wd_description.zip) |
| LSR550-4WD-Premium Independent Suspension | [lucsys_lsr550_4wd_premium_independent_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_4wd_premium_independent_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_4wd_premium_independent_suspension_description.zip) |
| LSR550-4WD-Premium Oscillating Axle Suspension | [lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description.zip) |
| LSR550-AKM-Flagship Independent Suspension | [lucsys_lsr550_akm_flagship_independent_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_akm_flagship_independent_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_akm_flagship_independent_suspension_description.zip) |
| LSR550-AKM-Premium Oscillating Axle Suspension | [lucsys_lsr550_akm_premium_oscillating_axle_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_akm_premium_oscillating_axle_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_akm_premium_oscillating_axle_suspension_description.zip) |
| LSR550-AKM-Ultimate Oscillating Axle Suspension | [lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description.zip) |
| LSR550-MEC-Flagship Independent Suspension | [lucsys_lsr550_mec_flagship_independent_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_mec_flagship_independent_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_mec_flagship_independent_suspension_description.zip) |
| LSR550-MEC-Premium Independent Suspension | [lucsys_lsr550_mec_premium_independent_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_mec_premium_independent_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_mec_premium_independent_suspension_description.zip) |
| LSR550-MEC-Premium Oscillating Axle Suspension | [lucsys_lsr550_mec_premium_oscillating_axle_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_mec_premium_oscillating_axle_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_mec_premium_oscillating_axle_suspension_description.zip) |
| LSR550-MEC-Ultimate Oscillating Axle Suspension | [lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description.zip) |
| LSR550-Omni-Premium | [lucsys_lsr550_omni_premium_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_omni_premium_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_omni_premium_description.zip) |
| LSR550-Omni-Ultimate | [lucsys_lsr550_omni_ultimate_description](https://github.com/LucSys-Robotics/technical-resources/blob/main/platforms/lucsys_lsr550_omni_ultimate_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr550_omni_ultimate_description.zip) |

These packages provide robot geometry and display launch files. Install the separate ROS 2 driver above for the chassis interface.

## Manuals & Electrical Connections

- [Development manual](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_LSR550_STM32_Chassis_Development_Manual.pdf)
- [Electrical connection and revision references](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LSR550/README.md)
- [Controller schematics and selection](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/controllers/README.md)
- [ROS 2 / STM32 communication protocol](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_ROS2_STM32_Communication_Guide.pdf)

Use drawings for the actual controller and hardware revision. A controller schematic is not a complete as-built robot wiring diagram. Verify directions, encoder feedback and stop behavior during controlled commissioning; software zero velocity is not physical power isolation.

[SHA256 checksums for all source and model ZIPs](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt). Public packages are directly downloadable; see the [FAQ](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md) for source contents, setup and integration boundaries.
