# LucSys Robotics — LSR180 Development Resources

[All models](../README.md) · [Repository home](../../README.md)

## 1. ROS 2 chassis driver

**Shared reference; compatibility with the delivered LSR180 controller/firmware is UNCONFIRMED.** Check the serial protocol, controller revision and configuration before connecting hardware. A matching model name or connector does not establish compatibility.

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
| LSR180 | [lucsys_lsr180_description](../../platforms/lucsys_lsr180_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr180_description.zip) |

The linked package README contains its build command and supported preview configurations. Model geometry does not establish controller or firmware compatibility.

## 3. STM32 source and build instructions

**Model-associated reference: C63A / 2026-01-27. Delivered hardware/firmware match still required.**

| Resource | Link |
| --- | --- |
| STM32 source | [Browse project](../../firmware/controllers/C63A/lsr100-lsr180-lsr200-lsr300-20260127/) |
| STM32 download | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/source-2026-11-23-lucsys-models/lucsys_lsr100_lsr180_lsr200_lsr300_20260127.zip) |
| Compiler, target selection and flashing instructions | [Project guide](../../firmware/controllers/C63A/lsr100-lsr180-lsr200-lsr300-20260127/README.md#build-and-flash) |

LSR100, LSR180, LSR200 and LSR300 link to the same source archive. Select the matching mode and delivered hardware; shared storage does not make their parameters interchangeable. Do not use the LSR280 branch as a substitute.

The selected project uses Keil MDK / ARM Compiler 5.06 update 7. Native Keil compilation and vehicle testing were not performed for this distribution. No tested drop-in firmware binary is supplied.

## 4. Download verification

- [Driver and STM32 ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/source-2026-11-23-lucsys-models/SHA256SUMS.txt)
- [Model ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/SHA256SUMS.txt)

Extract each selected ROS package into a workspace `src/` directory and follow its linked build guide. Keep STM32 projects separate from the ROS workspace. Confirm the delivered configuration before hardware operation.
