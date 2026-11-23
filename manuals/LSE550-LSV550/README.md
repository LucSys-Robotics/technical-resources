# LSE550 / LSV550 development resources

[Complete model-specific development resources](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/README.md)

## Downloads

- [English development manual](../LucSys_LSE550_LSV550_STM32_Chassis_Development_Manual.pdf)
- [Shared ROS 2 / STM32 communication guide](../LucSys_ROS2_STM32_Communication_Guide.pdf)
- [C30D V2.2 / V2.3 electrical references](../../electrical/controllers/README.md) and a separate [C30D V2.1 controller STEP model](../../cad/controllers/README.md)
- [ROS 2 Humble URDF, meshes and launch documentation](../../platforms/lucsys_lse550_description/); [package ZIP downloads](../../platforms/README.md)

## Configuration and version limits

Choose the precise model and drive configuration in the platform index. A family name is not proof of a particular controller, encoder, motor, firmware or package. Electrical references apply only to matching board revisions and depicted configurations. Do not transfer electrical or safety parameters to a related model. In particular, LSR550 and LSR550A are separate, and LSR200 and LSR200Auto are separate selections.

## Development FAQ

[Source availability, setup, wiring and integration questions](https://github.com/LucSys-Robotics/technical-resources/blob/main/FAQ.md).

## ROS 2 chassis-driver source

[Humble source and Quick Start](../../ros2/chassis-drivers/humble-20260903/README.md) · [Development FAQ](../../ros2/README.md). This is a serial hardware driver, not only a URDF package. Three packages built in Ubuntu 22.04 / Humble ARM64 and passed a virtual-serial command/feedback test. Select it by the actual protocol and firmware; no all-configuration compatibility or safety guarantee is implied.
