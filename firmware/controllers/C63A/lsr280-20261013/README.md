# LucSys Robotics — LSR280 firmware source reference

Snapshot: lsr280-20261013. MCU selected in Keil: `STM32F407VETx`. Targets: `LUCSYS`.

## Applicability and editing locations

This C63A project is associated with LSR280, as confirmed by LucSys. Use the separate project for other LucSys models. Mode selection is implemented in LUCSYS_APP/robot_select_init.c. This model association does not confirm the firmware installed on a particular delivered robot, nor compatibility with every hardware revision or configuration. Do not extend it to LSR200Auto.

Control and initialization: LUCSYS_APP/RobotControl_task*.c and robot_select_init.c. Driver/CAN/encoder processing: LUCSYS_BSP and LUCSYS_APP/can_callback.c (inspect actual project references). Some motor control is delegated to external drivers; do not assume there is a universal on-MCU PID or raw encoder interface. Use the actual driver implementation and parameter structures. STM32 HAL, CMSIS, FreeRTOS and USB host components remain included with notices.

## Build and flash

1. Use Windows with Keil MDK and ARM Compiler 5.06 update 7 (build 960), as selected by the project, plus the matching STM32 device support pack. This is not a GCC/STM32CubeIDE project and free-tool compatibility is not asserted.
2. Open `MDK-ARM/LUCSYS.uvprojx`. Select the matching target and check source-group inclusion, controller revision, motor/encoder configuration and memory/flash settings.
3. Rebuild all target files in the IDE. Check the source groups, include directories and device pack before building.
4. Select the hardware-matched debug/programming probe and flash algorithm in Keil, verify the generated HEX against the intended board, and program only after configuration review. No firmware binary is shipped as a tested drop-in update.
5. Commission with motors safely restrained and confirm directions, encoder feedback, limits and emergency-stop behavior. Publication does not establish delivered safety behavior or authorize changing safety logic.

## Distribution notices

LucSys Robotics maintains this distribution. Applicable third-party license and copyright notices remain in effect; no additional open-source license is granted.

## English documentation edition — 2026-11-23

Chinese source comments and accompanying TXT/Markdown explanations have been translated into English. Executable code, runtime messages, protocol strings, configuration values and firmware selection logic are unchanged. Runtime console output may therefore still contain Chinese. Changelogs describe historical source changes, not verified behavior of every delivered vehicle. Match the controller, hardware revision and firmware version before use.

TXT notes use `CHANGELOG.txt` (and `VERSION_NOTES.txt` where present); Keil project references were updated accordingly. Translation does not add hardware compatibility or safety guarantees.

## Unmapped reference branches

`REF1` and `REF2` are neutral internal identifiers for inherited reference branches. Their association with a LucSys model is unconfirmed. Renaming preserves their enum values, parameters and control logic; it does not authorize selecting them for a delivered robot. Do not infer an LSR model from these identifiers.
