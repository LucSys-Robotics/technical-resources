# LucSys Robotics — LSR550A V2 STM32 source

Snapshot: 2026-10-12. [Version selection](https://github.com/LucSys-Robotics/technical-resources/blob/main/firmware/models/LSR550A/README.md) · [LSR550A resource entry](https://github.com/LucSys-Robotics/technical-resources/blob/main/models/LSR550A/README.md)

This is a versioned LSR550A chassis/arm-command integration reference, not proof of the firmware installed on a particular robot. It is not a separate complete arm-actuator firmware SDK. Do not substitute LSR550 firmware or assume the latest snapshot matches the delivered unit.

## Build and flash

1. Use Keil MDK on Windows with ARM Compiler **5.06 update 7 (build 960)** and STM32F4 device support. The project selects **STM32F407VE**. GCC/STM32CubeIDE compatibility is not established.
2. Open `USER/LUCSYS.uvprojx`. Available targets are `Akm_Car`, `Diff_Car`, `Mec_Car`, `4WD_Car` and `Omni_Car`. Select only the target matching the delivered chassis; available targets are not a sales-configuration support matrix. The version note associates V2 with a multimode USB wireless controller; it does not establish V2/V2.1 interchangeability.
3. Check the actual board revision, motor/encoder setup, arm interface and firmware revision against the delivered unit before building. Rebuild all files in the selected target. Standard C headers come from the compiler. The optional FreeRTOS module-test header is guarded by `FREERTOS_MODULE_TEST` and is not part of a normal build.
4. Use the matching board's programming interface, probe and flash settings. Do not treat this archive as a tested drop-in update. Follow the programming-interface guidance in the [LSR550A manual](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LSR550A_Development_Manual.pdf) (PDF pages 28–29), subject to exact board matching.
5. Before hardware use, verify directions, encoder feedback, arm-command routing and stop behavior with a controlled commissioning procedure. No communication timeout, braking, stopping-distance or safety guarantee is established by source availability.

The project runs `USER/hexbinHandle.bat` before building to remove old HEX/BIN outputs and `USER/copyhex.bat` after building to copy HEX output. Use a dedicated project copy. 

## Distribution

LucSys Robotics maintains this source distribution. Third-party ST, ARM and FreeRTOS licenses and attribution remain applicable; no additional open-source license is granted.

## English documentation edition — 2026-11-23

Chinese source comments and accompanying TXT/Markdown explanations have been translated into English. Executable code, runtime messages, protocol strings, configuration values and firmware selection logic are unchanged. Runtime console output may therefore still contain Chinese. Changelogs describe historical source changes, not verified behavior of every delivered vehicle. Match the controller, hardware revision and firmware version before use.

TXT notes use `CHANGELOG.txt` (and `VERSION_NOTES.txt` where present); Keil project references were updated accordingly. Translation does not add hardware compatibility or safety guarantees.
