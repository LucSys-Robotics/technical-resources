# LSR550 series development resources

## Downloads

- [English development manual](../LucSys_LSR550_STM32_Chassis_Development_Manual.pdf)
- [Shared ROS 2 / STM32 communication guide](../LucSys_ROS2_STM32_Communication_Guide.pdf)
- [2WD / 4WD wiring diagrams](../../electrical/LSR550/README.md) and [C50C controller references](../../electrical/controllers/README.md)
- [ROS 2 Humble URDF, meshes and launch documentation](../../platforms/lucsys_lsr550_2wd_description/); [package ZIP downloads](../../README.md#ros-2-humble-platform-downloads)

## Configuration and version limits

Choose the precise model and drive configuration in the platform index. A family name is not proof of a particular controller, encoder, motor, firmware or package. Electrical references apply only to matching board revisions and depicted configurations. Do not transfer electrical or safety parameters to a related model. In particular, LSR550 and LSR550A are separate, and LSR200 and LSR200Auto are separate selections.

## Common questions

**Is complete STM32 firmware source available in this release?** Selected complete controller projects are available in the [firmware library](../../firmware/README.md). See [the corresponding controller branch](../../firmware/controllers/C50C/20261104/). This is source availability, not confirmation of the firmware installed in every configuration. Native Keil build and hardware validation were not performed.

**Can I modify motor-control parameters?** The diagrams and model packages do not establish which firmware is installed. Use the matching firmware README to locate PI/PID and encoder configuration. Parameter changes require the exact hardware and validation; no cross-model values are supplied here.

**Are electrical schematics or motor/encoder wiring references available?** Yes, see the exact electrical references above and their revision limits. A controller schematic is not a full as-built robot wiring diagram.

**Is ROS 2 Humble supported?** Humble description packages and their launch instructions are available. They provide robot models and visualization; this release does not validate a complete navigation, sensor-driver or motor-control stack on every host computer.

**What is included in the development package?** This page lists the public resources currently available. It is not an order-specific delivery commitment. Hardware-driver archives, complete firmware and custom drawings are not implied by a model download.

**Where are CAD/URDF models?** Use the model links above. A controller board or robotic-arm component model is not a chassis top-plate engineering drawing.

**How do I connect my own computer or sensors?** Start with the shared communication guide and exact controller drawing. Match voltage, pinout, interface and firmware before connection; custom integration and safety validation remain project-specific.

Product selection and purchasing: [LucSys Robotics](https://www.lucsysrobotics.com).

## ROS 2 chassis-driver source

[Humble source and Quick Start](../../ros2/chassis-drivers/humble-20260903/README.md) · [Development FAQ](../../ros2/README.md). This is a serial hardware driver, not only a URDF package. Three packages built in Ubuntu 22.04 / Humble ARM64 and passed a virtual-serial command/feedback test. Select it by the actual protocol and firmware; no all-configuration compatibility or safety guarantee is implied.
