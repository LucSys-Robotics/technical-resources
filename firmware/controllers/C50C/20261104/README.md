# LucSys Robotics — C50C firmware source reference

Snapshot: 20261104. MCU selected in Keil: `STM32F407VE`. Targets: `Akm_Car, Diff_Car, Mec_Car, 4WD_Car, Omni_Car`.

## Applicability and editing locations

This selected LSR550-family snapshot has Akm_Car, Diff_Car, Mec_Car, 4WD_Car and Omni_Car targets. The source guide explicitly maps differential drive to Diff_Car and four-wheel drive to 4WD_Car. In CarType/4wd_robot_init.c, selector cases 2/3 are the premium independent-suspension branch (normal/heavy gear variants); in CarType/diff_robot_init.c cases 0/1 initialize the two-wheel differential branch. Match the delivered motors, encoder type, selector setting and controller before compiling/flashing. This is not a claim that every current LSR550 ships this version. It does not apply automatically to LSR550A.

The hardware-generation values V1_0/V1_1/V1_2 in the source must not be equated to schematic filename revisions. PI defaults and encoder constants: CarType/Inc/robot_init.h (VEL_KP, VEL_KI, GMR_500, EncoderMultiples). Per-configuration initialization: CarType/*_robot_init.c. Motion loop: BALANCE/balance_task.c. Encoder capture: HARDWARE/encoder.c. UART/CAN: HARDWARE/uartx.c, HARDWARE/can.c and BALANCE/*_callback.c. Motor output: HARDWARE/motor.c; D50A integration: HARDWARE/driver_d50a.c. Do not substitute the inactive historical BALANCE/control.c for the target's active implementation.

The included pre-build script deletes old HEX/BIN build products and the post-build script copies HEX output to the project root. These scripts were read but not executed on macOS. Keep builds in a dedicated project copy. Do not treat build-output removal as deletion of source files.

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

## English documentation edition — 2026-11-23

Chinese source comments and accompanying TXT/Markdown explanations have been translated into English. Executable code, runtime messages, protocol strings, configuration values and firmware selection logic are unchanged. Runtime console output may therefore still contain Chinese. Changelogs describe historical source changes, not verified behavior of every delivered vehicle. Match the controller, hardware revision and firmware version before use.

TXT notes use `CHANGELOG.txt` (and `VERSION_NOTES.txt` where present); Keil project references were updated accordingly. Earlier source releases remain available for comparison. Translation does not add hardware compatibility or safety guarantees.
