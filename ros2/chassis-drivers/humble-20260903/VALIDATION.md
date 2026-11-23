# Validation scope

- Ubuntu 22.04 / ROS 2 Humble ARM64 container: three packages built successfully with BUILD_TESTING=OFF.
- Source/header/message equivalence checked against the supplied snapshot, allowing package/header/executable renames, cosmetic logs and one empty-read guard only. Topic names, valid wire frames and vehicle-control policy unchanged.
- Python launch syntax, XML and YAML parsing checked. Optional full-stack launch dependencies are not bundled or runtime-certified.
- Virtual serial peer: zero bytes available does not publish fabricated feedback; valid velocity commands, odometry, IMU and voltage feedback tested. Reproduce after sourcing the built workspace with `python3 tests/serial_smoke.py`. This test opens a pseudo-terminal, not a robot device.
- Native STM32 compilation, physical disconnect/reconnect, hardware integration, braking and safety validation: NOT RUN.
- Inherited compiler warnings remain in arm-command byte packing. No arm hardware or arm-command behavior is certified by this release.
- No new license has been applied. Original third-party license text and authors remain in the source. Legacy device aliases and external optional-package names remain for compatibility.
