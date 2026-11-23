# LSR550 series development resources

## Downloads

- [English development manual](../LucSys_LSR550_STM32_Chassis_Development_Manual.pdf)
- [Shared ROS 2 / STM32 communication guide](../LucSys_ROS2_STM32_Communication_Guide.pdf)
- [2WD / 4WD wiring diagrams](../../electrical/LSR550/README.md) and [C50C controller references](../../electrical/controllers/README.md)
- [ROS 2 Humble URDF, meshes and launch documentation](../../platforms/lucsys_lsr550_2wd_description/); [package ZIP downloads](../../README.md#ros-2-humble-platform-downloads)

## Configuration and version limits

Choose the precise model and drive configuration in the platform index. A family name is not proof of a particular controller, encoder, motor, firmware or package. Electrical references apply only to matching board revisions and depicted configurations. Do not transfer electrical or safety parameters to a related model. In particular, LSR550 and LSR550A are separate, and LSR200 and LSR200Auto are separate selections.

## Common questions

**Is complete STM32 firmware source available in this release?** No complete STM32 source archive is included in this release. The public files listed above are the available downloads; do not assume that a URDF package contains firmware or hardware drivers.

**Can I modify motor-control parameters?** The diagrams and model packages do not establish which firmware is installed. Parameter changes require the matching source, hardware revision and validation on that configuration; no cross-model values are supplied here.

**Are electrical schematics or motor/encoder wiring references available?** Yes, see the exact electrical references above and their revision limits. A controller schematic is not a full as-built robot wiring diagram.

**Is ROS 2 Humble supported?** Humble description packages and their launch instructions are available. They provide robot models and visualization; this release does not validate a complete navigation, sensor-driver or motor-control stack on every host computer.

**What is included in the development package?** This page lists the public resources currently available. It is not an order-specific delivery commitment. Hardware-driver archives, complete firmware and custom drawings are not implied by a model download.

**Where are CAD/URDF models?** Use the model links above. A controller board or robotic-arm component model is not a chassis top-plate engineering drawing.

**How do I connect my own computer or sensors?** Start with the shared communication guide and exact controller drawing. Match voltage, pinout, interface and firmware before connection; custom integration and safety validation remain project-specific.

Product selection and purchasing: [LucSys Robotics](https://www.lucsysrobotics.com).
