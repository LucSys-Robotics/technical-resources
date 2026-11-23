# LucSys Robotics — LSR280 firmware source reference

Snapshot: lsr280-20261013. MCU selected in Keil: `STM32F407VETx`. Targets: `LUCSYS`.

## Applicability and editing locations

This C63A project is associated with LSR280, as confirmed by LucSys. Use the separate project for other LucSys models. Mode selection is implemented in LUCSYS_APP/robot_select_init.c. This model association does not confirm the firmware installed on a particular delivered robot, nor compatibility with every hardware revision or configuration. Do not extend it to LSR200Auto.

Control and initialization: LUCSYS_APP/RobotControl_task*.c and robot_select_init.c. Driver/CAN/encoder processing: LUCSYS_BSP and LUCSYS_APP/can_callback.c (inspect actual project references). Some motor control is delegated to external drivers; do not assume there is a universal on-MCU PID or raw encoder interface. Use the actual driver implementation and parameter structures. STM32 HAL, CMSIS, FreeRTOS and USB host components remain included with notices.

## Build and flash

1. Use Windows with Keil MDK and ARM Compiler 5.06 update 7 (build 960), as selected by the project, plus the matching STM32 device support pack. This is not a GCC/STM32CubeIDE project and free-tool compatibility is not asserted.
2. Open `MDK-ARM/LUCSYS.uvprojx`. Select the matching target and check source-group inclusion, controller revision, motor/encoder configuration and memory/flash settings.
3. Rebuild all target files in the IDE. The source files and configured local include directories were checked; an actual Keil build was not run in this macOS environment.
4. Select the hardware-matched debug/programming probe and flash algorithm in Keil, verify the generated HEX against the intended board, and program only after configuration review. No firmware binary is shipped as a tested drop-in update.
5. Commission with motors safely restrained and confirm directions, encoder feedback, limits and emergency-stop behavior. Publication does not establish delivered safety behavior or authorize changing safety logic.

## Verification and notices

All C/C++/assembly executable tokens and strings are unchanged by commercial-comment cleanup. The Keil project file is unchanged. No source file referenced by its targets is missing. Standard-library headers require the compiler; optional FreeRTOS test hooks and full-assert headers are conditional, not supplied active dependencies. Native Keil compilation and hardware tests were not run. Host-side tests, where present, exercise only their stated routines.

LucSys Robotics prepares and maintains this distribution. Third-party copyright/license notices remain intact. Project paths and referenced identifiers are consistent. LucSys maintains this distribution and documentation, not a claim of original authorship. No new open-source license is applied to proprietary application code. Commercial shop links, local IDE caches, generated binaries and unrelated embedded PDFs are not part of this source release.

[Electrical controller references](../../../../electrical/controllers/README.md) · [ROS 2 driver](../../../../ros2/chassis-drivers/humble-20260903/README.md)
