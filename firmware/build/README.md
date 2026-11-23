# Reproducible Windows / Keil build

Status: **not executed**. macOS validation has no Windows/Keil toolchain. Source-path checks and host simulations do not replace target compilation. All 9 projects / 21 targets in `keil-projects.json` remain NOT_COMPILED.

1. Use a fresh Windows checkout of this candidate at an ASCII path such as `C:\LucSys\technical-resources`. Build in a disposable copy; Keil can update project preferences and creates outputs.
2. Install licensed Keil MDK/uVision with **ARM Compiler 5.06 update 7 build 960**. Register that compiler in uVision; do not silently substitute Arm Compiler 6. Project XML records compiler selection and device pack versions.
3. Install **Keil.STM32F4xx_DFP 2.9.0** for C30D/C50C/LSR550A, and **3.1.1** for the two C63A projects. CMSIS, device headers and FreeRTOS source included in each project must remain intact. Match the selected target to actual hardware; successful compilation does not establish fitment.
4. Run from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File firmware\build\build-all.ps1 -UV4 'C:\Keil_v5\UV4\UV4.exe' -Output 'C:\LucSys\keil-logs'
```

The script rebuilds targets sequentially with `-r`, selects each target with `-t`, and captures a separate `-o` log. Review every warning, compiler/pack identity, linker output and generated artifact. Preserve logs and artifact SHA256 values with the candidate commit. Existing project hooks can remove previous HEX/BIN outputs and copy newly generated files, so use the disposable copy described above. Ensure ARM Compiler 5 fromelf is available to the build hooks. No flashing occurs. Do not declare a firmware version verified until its target build and hardware checks complete. The script itself has not been executed on Windows here.

Command syntax: [Keil official command-line documentation](https://www.keil.com/support/man/docs/uv4/uv4_commandline.asp).
