# LucSys Robotics — LSR550A Development Resources

[All models](../README.md) · [Repository home](../../README.md)

## 1. ROS 2 chassis driver

**Shared reference; compatibility with the delivered LSR550A controller/firmware is UNCONFIRMED.** Check the serial protocol, controller revision and configuration before connecting hardware. A matching model name or connector does not establish compatibility.

| Resource | Link |
| --- | --- |
| Driver source (Humble, 2026-09-03) | [Browse source](../../ros2/chassis-drivers/humble-20260903/) |
| Driver download | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/source-2026-11-23-lucsys-models/lucsys_ros2_humble_20260903.zip) |
| Build and connection instructions | [Ubuntu 22.04 / ROS 2 Humble](../../ros2/chassis-drivers/humble-20260903/README.md#build) |

The driver ZIP includes the chassis driver, message definitions and serial dependency. It is separate from the model package below and is not a complete navigation, sensor or robotic-arm SDK. Build and virtual-serial tests do not verify vehicle behavior.

## 2. ROS 2 model packages

Choose the matching sales configuration. These packages contain URDF, meshes, display launch files and RViz resources; they do not include the hardware driver or STM32 firmware.

| Configuration | Model source and build / preview instructions | Download |
| --- | --- | --- |
| LSR550A-4WD | [lucsys_lsr550a_4wd_description](../../platforms/lucsys_lsr550a_4wd_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550a_4wd_description.zip) |
| LSR550A-MEC | [lucsys_lsr550a_mec_description](../../platforms/lucsys_lsr550a_mec_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550a_mec_description.zip) |

The linked package README contains its build command and supported preview configurations. Model geometry does not establish controller or firmware compatibility.

## 3. STM32 source and build instructions

**Versioned LSR550A source references are available. The actual delivered firmware version remains UNCONFIRMED.**

[Open the LSR550A STM32 version selector](../../firmware/models/LSR550A/README.md) for V1 (2025-12-04), V2 and V2.1 (2026-10-12), source ZIPs and version-specific Keil build instructions. These are LSR550A archives, not substitute LSR550 code. V1 is associated with wired PS2; V2 with multimode USB wireless control. The exact V2/V2.1 hardware-selection conditions still require confirmation.

## 4. Download verification

- [Driver and STM32 ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/source-2026-11-23-lucsys-models/SHA256SUMS.txt)
- [Model ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/SHA256SUMS.txt)

Extract each selected ROS package into a workspace `src/` directory and follow its linked build guide. Keep STM32 projects separate from the ROS workspace. Confirm the delivered configuration before hardware operation.
