# LucSys Robotics — C30D firmware source reference

Snapshot: v2-gmr-20260921. MCU selected in Keil: `STM32F407ZG`. Targets: `FreeRTOS`.

## Applicability and editing locations

Selected C30D 2.0 GMR-encoder snapshot. The input folder identifies the C30D 2.0 board branch; this is not interchangeable with the separate Hall-encoder variant or C30D 1.0. It is indexed with LSE550/LSV550 for resource discovery, not an as-built confirmation for every model. Inspect BALANCE/robot_select_init.c and its headers for mode selection, encoder resolution and geometry. Inspect BALANCE/balance.c for the active PI loop and HARDWARE/encoder.c for capture setup. Exact delivered configuration remains a commissioning prerequisite.

## Build and flash

1. Use Windows with Keil MDK and ARM Compiler 5.06 update 7 (build 960), as selected by the project, plus the matching STM32 device support pack. This is not a GCC/STM32CubeIDE project and free-tool compatibility is not asserted.
2. Open `USER/LUCSYS.uvprojx`. Select the matching target and check source-group inclusion, controller revision, motor/encoder configuration and memory/flash settings.
3. Rebuild all target files in the IDE. The source files and configured local include directories were checked; an actual Keil build was not run in this macOS environment.
4. Select the hardware-matched debug/programming probe and flash algorithm in Keil, verify the generated HEX against the intended board, and program only after configuration review. No firmware binary is shipped as a tested drop-in update.
5. Commission with motors safely restrained and confirm directions, encoder feedback, limits and emergency-stop behavior. Publication does not establish delivered safety behavior or authorize changing safety logic.

## Verification and notices

All C/C++/assembly executable tokens and strings are unchanged by commercial-comment cleanup. The Keil project file is unchanged. No source file referenced by its targets is missing. Standard-library headers require the compiler; optional FreeRTOS test hooks and full-assert headers are conditional, not supplied active dependencies. Native Keil compilation and hardware tests were not run. Host-side tests, where present, exercise only their stated routines.

LucSys Robotics prepares and maintains this distribution. Third-party copyright/license notices remain intact. Project paths and referenced identifiers are consistent. LucSys maintains this distribution and documentation, not a claim of original authorship. No new open-source license is applied to proprietary application code. Commercial shop links, local IDE caches, generated binaries and unrelated embedded PDFs are not part of this source release.

[Electrical controller references](../../../../electrical/controllers/README.md) · [ROS 2 driver](../../../../ros2/chassis-drivers/humble-20260903/README.md)
