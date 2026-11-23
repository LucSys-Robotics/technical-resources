# LucSys Robotics — LSR200Auto Development Resources

[All models](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/models/README.md) · [Repository home](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/README.md)

## 1. ROS 2 chassis driver

**Shared reference; compatibility with the delivered LSR200Auto controller/firmware is UNCONFIRMED.** Check the serial protocol, controller revision and configuration before connecting hardware. A matching model name or connector does not establish compatibility.

| Resource | Link |
| --- | --- |
| Driver source (Humble, 2026-09-03) | [Browse source](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/ros2/chassis-drivers/humble-20260903) |
| Driver download | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_ros2_humble_20260903.zip) |
| Build and connection instructions | [Ubuntu 22.04 / ROS 2 Humble](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/ros2/chassis-drivers/humble-20260903/README.md#build) |

The driver ZIP includes the chassis driver, message definitions and serial dependency. It is separate from the model package below and is not a complete navigation, sensor or robotic-arm SDK. Build and virtual-serial tests do not verify vehicle behavior.

## 2. ROS 2 model packages

Choose the matching sales configuration. These packages contain URDF, meshes, display launch files and RViz resources; they do not include the hardware driver or STM32 firmware.

| Configuration | Model source and build / preview instructions | Download |
| --- | --- | --- |
| LSR200Auto-4WD | [lucsys_lsr200auto_4wd_description](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/platforms/lucsys_lsr200auto_4wd_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr200auto_4wd_description.zip) |
| LSR200Auto Ackermann Lite | [lucsys_lsr200auto_akm_lite_description](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/platforms/lucsys_lsr200auto_akm_lite_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr200auto_akm_lite_description.zip) |
| LSR200Auto-AKM Pro | [lucsys_lsr200auto_akm_pro_description](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/platforms/lucsys_lsr200auto_akm_pro_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr200auto_akm_pro_description.zip) |
| LSR200Auto-AKM Ultra | [lucsys_lsr200auto_akm_ultra_description](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/platforms/lucsys_lsr200auto_akm_ultra_description/README.md) | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr200auto_akm_ultra_description.zip) |

The linked package README contains its build command and supported preview configurations. Model geometry does not establish controller or firmware compatibility.

## 3. STM32 source and build instructions

**Shared LSR200 / LSR200Auto technical reference, confirmed by LucSys on 2026-11-23.** The actual delivered firmware revision and configuration still need matching.

- [Shared STM32 source](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/firmware/controllers/C63A/lsr100-lsr180-lsr200-lsr300-20260127)
- [Keil compiler, target selection and build instructions](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/firmware/controllers/C63A/lsr100-lsr180-lsr200-lsr300-20260127/README.md#build-and-flash)
- [Download the existing shared source ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/lucsys_lsr100_lsr180_lsr200_lsr300_20260127.zip)

This links the same source used by the LSR200 entry without copying it. Confirm the delivered controller, drive mode, motor/encoder setup and firmware build before flashing. Shared documentation does not independently verify automatic charging, docking-station integration, safety behavior or compatibility of every sales configuration. Native Keil compilation and vehicle testing were not performed for this distribution.

## 4. Download verification

- [Driver and STM32 ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt)
- [Model ZIP SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt)

Extract each selected ROS package into a workspace `src/` directory and follow its linked build guide. Keep STM32 projects separate from the ROS workspace. Confirm the delivered configuration before hardware operation.

## Technical documentation

- [Development manual](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_LSR100_LSR180_LSR200_LSR280_LSR300_User_and_Development_Manual.pdf)
- [Electrical connection and revision references](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LSR180-LSR200-LSR280-LSR300/README.md)
- [Controller schematics and selection](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/controllers/README.md)
- [ROS 2 / STM32 communication protocol](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_ROS2_STM32_Communication_Guide.pdf)

Select the document revision matching the controller and actual configuration. Shared document access does not make all configurations interchangeable.
