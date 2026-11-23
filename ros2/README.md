# LucSys Robotics — ROS 2 development resources

[Humble chassis-driver source and Quick Start](chassis-drivers/humble-20260903/README.md)

This release contains a real serial chassis driver, message definitions and its serial-library dependency. It complements the existing robot-description packages. It is not a complete navigation or sensor SDK.

## Development FAQ

**Is the full STM32 firmware source code available here?** STM32 firmware is not included in this ROS release. Select the exact controller and delivered firmware before using a firmware project; the ROS driver is not STM32 source.

**Can I modify the motor PID parameters?** PID/PI and encoder calibration belong to the matching controller firmware or motor driver. This ROS package exposes commands and feedback, not universal gains. Do not use another configuration's values.

**Is ROS 2 Humble driver source available?** Yes: [source](chassis-drivers/humble-20260903/) and [build instructions](chassis-drivers/humble-20260903/README.md#build). All three packages were built in Ubuntu 22.04 / Humble ARM64.

**Can I use my own Jetson or Raspberry Pi?** A host running the matching ROS distribution and dependencies can use the serial interface, subject to controller protocol and electrical compatibility. Physical Jetson/Raspberry Pi hardware was not tested in this release.

**How do I access encoder and odometry feedback?** The driver publishes `/odom` from chassis-velocity feedback and `/imu/data_raw`. It does not expose raw individual wheel encoder counts. Inspect the matching STM32 source/protocol for raw counts; do not infer them from odometry.

**Which controller and firmware versions are supported?** The source snapshot is dated 2026-09-03 and implements the frame formats described in its README. Exact installed controller/firmware compatibility must be matched separately. Shared source availability does not make LSR550, LSR550A, LSR200 and LSR200Auto interchangeable.

**How do I compile and flash firmware?** The Quick Start compiles ROS packages with colcon. It does not flash a controller. A matching STM32 project, its specified toolchain and board-specific programming settings are required; no prebuilt firmware image is supplied here.

**Where are the electrical diagrams?** Use the [controller revision library](../electrical/controllers/README.md), [LSR550 wiring references](../manuals/LSR550/README.md) and the [shared communication guide](../manuals/LucSys_ROS2_STM32_Communication_Guide.pdf). A controller schematic is not a complete robotic-arm wiring diagram.

## Model resource entry points

- [LSR550 configurations](../manuals/LSR550/README.md)
- [LSE550 / LSV550](../manuals/LSE550-LSV550/README.md)
- [LSR180 / LSR200 / LSR280 / LSR300](../manuals/LSR180-LSR200-LSR280-LSR300/README.md)
- [Other model manuals, including LSR100 and LSR550A](../manuals/README.md)

These links identify documentation families, not tested model-to-firmware mappings. Commands were exercised only with a synthetic serial peer; no vehicle, safety or disconnection certification is implied.
