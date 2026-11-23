# Controller electrical references

LucSys Robotics publishes these revision-specific circuit references for hardware integration. The circuit sheets retain their original connections, component values, pin labels and drawing revision fields. Commercial marks have been removed. Chinese annotations have been replaced directly with English on the circuit sheets; no separate translation key is needed. Original circuit page order is preserved. The circuit design is not represented as LucSys original authorship.

| Controller designation in the supplied file | Reference | Circuit sheets |
| --- | --- | ---: |
| C30D V2.2 | [PDF](C30D_V2_2_Schematic_Reference.pdf) | 3 |
| C30D V2.3 | [PDF](C30D_V2_3_Schematic_Reference.pdf) | 3 |
| C63A V2.1 | [PDF](C63A_V2_1_Schematic_Reference.pdf) | 4 |
| C63A V2.2 | [PDF](C63A_V2_2_Schematic_Reference.pdf) | 4 |
| C50C V4.0 | [PDF](C50C_V4_0_Schematic_Reference.pdf) | 4 |
| C50C V4.1 / V4.2 | [PDF](C50C_V4_1_V4_2_Schematic_Reference.pdf) | 4 |
| C50C V4.3 | [PDF](C50C_V4_3_Schematic_Reference.pdf) | 4 |

## Select the exact hardware

Use the controller's physical silkscreen, connector layout and matching delivered firmware to select a reference. File designations and embedded drawing `REV` fields are separate identifiers and have not been normalized. In particular, the C50C V4.3 file retains a drawing `REV: 14.4` and date `2025-3-07`; those fields alone do not establish the board installed in your robot. These are reference drawings, not an as-built electrical certification for every chassis.

- C30D references are indexed with the LSE550 / LSV550 family. The V2.1 [board CAD model](../../cad/controllers/C30D_V2_1.step) is separate from the V2.2/V2.3 electrical drawings.
- C63A references are indexed with the LSR180 / LSR200 / LSR280 / LSR300 family. This does not establish that every family member has that controller.
- C50C references here are indexed with LSR550. They are not automatically applicable to LSR550A. Use the separately published [LSR550A controller schematic](../LSR550A/LSR550A_C50C_Controller_Schematic.pdf) only after matching its hardware revision.
- A controller e-stop input label does not establish vehicle power isolation, active braking, stopping distance or communication-loss behavior. Those depend on the complete configuration and firmware.

## Version notes

The accompanying board notes specify that C30D V2.2 to V2.3 changes concern power circuitry and USB TVS protection, with unchanged dimensions, mounting holes, control use and code between those two versions. Do not extend that statement to V2.1 or to another controller.

C63A V2.1 to V2.2 changes include SWD wire order, CAN interface protection, power stability, a GH1.25-4PIN RS485 connector and an immersion-gold PCB finish. The notes describe unchanged dimensions, mounting holes, control use and code across V2.0/V2.1/V2.2; that does **not** mean unchanged connector wiring.

C50C V4.3 notes identify an OLED pin change from D5/D6 to E0/E1 with a changed wire order, an RS485 interface using UART2 D5/D6 (GH1.25-4PIN), moved SBUS, removal of the female-header Bluetooth interface, power/CAN circuit changes and version-identification resistors. C50C board selection must use its physical silkscreen: the accompanying note states there are no C50C V2.0 or V3.0 series. Do not use generational wording in older change notes as a board identification method.

## Development resources

[Shared communication guide](../../manuals/LucSys_ROS2_STM32_Communication_Guide.pdf) · [ROS 2 Humble description packages](../../README.md#ros-2-humble-platform-downloads)

Selected [complete STM32 source projects](../../firmware/README.md) are now available; choose the exact branch before use. The separately published [ROS 2 serial chassis driver](../../ros2/README.md) includes source and build instructions. Description packages are model/visualization resources, not a complete control stack. Never flash firmware or change motor-control parameters based only on a shared controller name.

Product selection and purchasing: [LucSys Robotics](https://www.lucsysrobotics.com).
