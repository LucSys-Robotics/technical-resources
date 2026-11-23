# LucSys Robotics — Resources by model

Choose your model below. Each entry brings together the ROS 2 driver, robot-description packages, STM32 source status and build instructions. Shared source is linked once, not copied into model folders.

**A model association is not confirmation of the firmware installed in your robot.** Match the delivered controller, firmware, motor/encoder configuration and protocol before hardware use. The shared ROS 2 driver has build and virtual-serial validation only; per-model hardware compatibility remains unconfirmed.

| Model | Model packages | STM32 reference status |
| --- | ---: | --- |
| [LSR100](LSR100/README.md) | 1 | C63A / 2026-01-27; delivered revision must match |
| [LSR180](LSR180/README.md) | 1 | C63A / 2026-01-27; delivered revision must match |
| [LSR200](LSR200/README.md) | 1 | C63A / 2026-01-27; delivered revision must match |
| [LSR200Auto](LSR200Auto/README.md) | 4 | Shared LSR200 reference confirmed; delivered revision must match |
| [LSR280](LSR280/README.md) | 1 | C63A / 2026-10-13; delivered revision must match |
| [LSR300](LSR300/README.md) | 2 | C63A / 2026-01-27; delivered revision must match |
| [LSR550](LSR550/README.md) | 12 | C50C / 2026-11-04; delivered revision must match |
| [LSR550A](LSR550A/README.md) | 2 | V1 / V2 / V2.1 references; delivered version unconfirmed |
| [LSE550](LSE550/README.md) | 1 | C30D 2.0 GMR / 2026-09-21; delivered revision must match |
| [LSV550](LSV550/README.md) | 1 | C30D 2.0 GMR / 2026-09-21; delivered revision must match |

[ROS 2 library](../ros2/README.md) · [STM32 library](../firmware/README.md) · [Manuals](../manuals/README.md)
