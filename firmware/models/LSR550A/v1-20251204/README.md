# LucSys Robotics — LSR550A V1 STM32 source

Snapshot: 2025-12-04. [Version selection](../README.md) · [LSR550A resource entry](../../../../models/LSR550A/README.md)

This is a versioned LSR550A chassis/arm-command integration reference, not proof of the firmware installed on a particular robot. It is not a separate complete arm-actuator firmware SDK. Do not substitute LSR550 firmware or assume the latest snapshot matches the delivered unit.

## Build and flash

1. Use Keil MDK on Windows with ARM Compiler **5.06 update 7 (build 960)** and STM32F4 device support. The project selects **STM32F407VE**. GCC/STM32CubeIDE compatibility is not established.
2. Choose the matching historical project: `4wd/USER/LUCSYS.uvprojx`, `mecanum/USER/LUCSYS.uvprojx` or `differential/USER/LUCSYS.uvprojx`. Each uses the `FreeRTOS` build target. The presence of the differential project is not confirmation of a currently offered sales configuration. The supplied version note associates V1 with a wired PS2 controller.
3. Check the actual board revision, motor/encoder setup, arm interface and firmware revision against the delivered unit before building. Rebuild all files in the selected target. Standard C headers come from the compiler. The optional FreeRTOS module-test header is guarded by `FREERTOS_MODULE_TEST` and is not part of a normal build.
4. Use the matching board's programming interface, probe and flash settings. Do not treat this archive as a tested drop-in update. Follow the programming-interface guidance in the [LSR550A manual](../../../../manuals/LSR550A_Development_Manual.pdf) (PDF pages 28–29), subject to exact board matching.
5. Before hardware use, verify directions, encoder feedback, arm-command routing and stop behavior with a controlled commissioning procedure. No communication timeout, braking, stopping-distance or safety guarantee is established by source availability.

## Validation and distribution

All project-listed files exist. Local quoted include paths were checked; unresolved standard headers are toolchain-provided and the FreeRTOS test hook is conditional. C/C++/assembly code tokens and strings were compared against the original with only authorized supplier-brand identifier substitution; control logic, calibration and safety settings were not changed.

**Native Keil compilation, flashing and vehicle tests have not been performed.** Generated binaries, IDE caches, unreferenced cleanup tools and bundled non-source drawings are not included. Original archives remain unchanged. Source files retain their original UTF-8 or GB18030 encoding; select the matching editor encoding for Chinese comments.

LucSys Robotics prepares and maintains this distribution. Third-party ST, ARM and FreeRTOS notices remain applicable. This maintenance notice grants no new blanket open-source license.
