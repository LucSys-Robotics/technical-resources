# LucSys Robotics — C30D firmware source reference

Snapshot: v2-gmr-20260921. MCU selected in Keil: `STM32F407ZG`. Targets: `FreeRTOS`.

## Applicability and editing locations

Selected C30D 2.0 GMR-encoder snapshot. The input folder identifies the C30D 2.0 board branch; this is not interchangeable with the separate Hall-encoder variant or C30D 1.0. It is indexed with LSE550/LSV550 for resource discovery, not an as-built confirmation for every model. Inspect BALANCE/robot_select_init.c and its headers for mode selection, encoder resolution and geometry. Inspect BALANCE/balance.c for the active PI loop and HARDWARE/encoder.c for capture setup. Exact delivered configuration remains a commissioning prerequisite.

## Build and flash

1. Use Windows with Keil MDK and ARM Compiler 5.06 update 7 (build 960), as selected by the project, plus the matching STM32 device support pack. This is not a GCC/STM32CubeIDE project and free-tool compatibility is not asserted.
2. Open `USER/LUCSYS.uvprojx`. Select the matching target and check source-group inclusion, controller revision, motor/encoder configuration and memory/flash settings.
3. Rebuild all target files in the IDE. Check the source groups, include directories and device pack before building.
4. Select the hardware-matched debug/programming probe and flash algorithm in Keil, verify the generated HEX against the intended board, and program only after configuration review. No firmware binary is shipped as a tested drop-in update.
5. Commission with motors safely restrained and confirm directions, encoder feedback, limits and emergency-stop behavior. Publication does not establish delivered safety behavior or authorize changing safety logic.

## Distribution notices

LucSys Robotics maintains this distribution. Applicable third-party license and copyright notices remain in effect; no additional open-source license is granted.

## English documentation edition — 2026-11-23

Chinese source comments and accompanying TXT/Markdown explanations have been translated into English. Executable code, runtime messages, protocol strings, configuration values and firmware selection logic are unchanged. Runtime console output may therefore still contain Chinese. Changelogs describe historical source changes, not verified behavior of every delivered vehicle. Match the controller, hardware revision and firmware version before use.

TXT notes use `CHANGELOG.txt` (and `VERSION_NOTES.txt` where present); Keil project references were updated accordingly. Translation does not add hardware compatibility or safety guarantees.
