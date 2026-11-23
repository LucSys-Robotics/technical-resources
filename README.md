# LucSys Robotics — Development Source

Development source packages for research, education and robotic system integration.

[Download Development Source](https://github.com/LucSys-Robotics/technical-resources/releases/tag/development-source-20261123) · [Choose your model](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/models/README.md) · [Technical documentation](https://github.com/LucSys-Robotics/technical-resources/blob/main/README.md)

| Resource | Contents | Setup |
| --- | --- | --- |
| STM32 firmware | Seven source archives with motor control, encoder processing, communication interfaces and Keil project files | [Controller and version selection](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/firmware/README.md) · [Windows / Keil build](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/firmware/build/README.md) |
| ROS 2 driver | ROS 2 Humble chassis driver, message definitions and serial dependency | [Build and connection](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/ros2/chassis-drivers/humble-20260903/README.md) |
| Robot models | 26 packages containing URDF, meshes and ROS 2 launch resources | [Model-specific resources](https://github.com/LucSys-Robotics/technical-resources/blob/development-source-20261123/models/README.md) |

ROS 2 builds and virtual serial communication checks passed on Ubuntu 22.04 / Humble ARM64. Select source and project targets for the actual controller revision, motors, encoders and firmware configuration. Commission hardware with appropriate mechanical restraint and emergency-stop access. Software zero velocity is not physical power isolation.

[SHA256 checksums](https://github.com/LucSys-Robotics/technical-resources/releases/download/development-source-20261123/SHA256SUMS.txt). Original third-party license and copyright notices remain applicable. LucSys Robotics prepares and maintains this distribution; no additional open-source license is granted.
