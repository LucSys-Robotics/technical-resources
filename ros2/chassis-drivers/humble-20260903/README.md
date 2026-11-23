# LucSys Robotics — ROS 2 chassis driver

Source snapshot: 2026-09-03. ROS 2 Humble / Ubuntu 22.04, not ROS 1. This is the serial hardware-driver package, its message package and serial-library dependency, not a complete navigation SDK.

## Scope and configuration

Use only with a controller implementing the matching serial protocol: 115200 baud by default, 11-byte command frames (0x7B / 0x7D, XOR checksum) and 24-byte base feedback frames. A matching connector alone does not establish compatibility. The shared resource can be evaluated for LSR550 and other LucSys chassis, but this release does not certify a delivered controller/firmware version or a mechanical configuration. LSR550A arm and automatic-charge integrations require their own matching firmware and hardware.

The `car_mode` parameter is retained, but in this snapshot the driver reads and lowercases it without selecting chassis kinematics. Do not use a mode string as proof of robot compatibility. Kinematics and encoder calibration are controller-side responsibilities. Optional launch files refer to external camera, LiDAR, navigation and IMU packages not bundled here; use the direct chassis-node command below for the verified minimal path.

## Build

Clone the public repository, then copy this directory's three package folders into a clean workspace:

```bash
mkdir -p ~/lucsys_driver_ws/src
cp -R technical-resources/ros2/chassis-drivers/humble-20260903/{lucsys_chassis_driver,lucsys_robot_msg,serial_ros2} ~/lucsys_driver_ws/src/
cd ~/lucsys_driver_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

The production build was executed in Ubuntu 22.04 / Humble ARM64. rosdep is a dependency-installation recipe; dependencies were installed explicitly in the validation container. The candidate test run reports 246 tests, zero errors/failures and 53 upstream cppcheck skips. A direct cppcheck run completed with two pre-existing alloca warnings in the unchanged third-party serial library. Third-party serial-library headers and notices retain their original terms. LucSys Robotics prepares and maintains the application-source distribution. Supplier-authorized application distribution is documented by LicenseRef-LucSys-Supplier-Authorization and lucsys_chassis_driver/NOTICE.md; no new open-source license is granted by this maintenance notice. Third-party terms remain applicable.

## Connect and inspect

Use the actual controller serial device and grant your account appropriate `dialout` access. No privileged vendor udev installer is included or required. Confirm wiring, serial protocol, controller configuration and safe commissioning conditions first.

```bash
ros2 run lucsys_chassis_driver lucsys_chassis_node --ros-args \
  -p usart_port_name:=/dev/ttyUSB0 -p serial_baud_rate:=115200
ros2 topic echo /odom
ros2 topic echo /imu/data_raw
ros2 topic echo /PowerVoltage
```

The default device is /dev/lucsys_controller and the default node is lucsys_robot; the command above overrides the port. Legacy compatibility is described below. Topic names and wire data were not changed by packaging.

A motion-command example, validated against a virtual serial peer only:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
# To send zero velocity from another terminal:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Software zero velocity is not hardware power isolation. No default vehicle timeout, stopping distance or braking guarantee is established here. The serial-library read timeout is not a motor-command watchdog. A receive-count guard discards empty reads before parsing. This does not implement a vehicle command timeout; physical disconnect/reconnect behavior remains unverified. Do not rely on this node as a safety controller.

## Interfaces and source locations

- `/cmd_vel`: `geometry_msgs/msg/Twist`, `Cmd_Vel_Callback()` in `lucsys_chassis_driver/src/lucsys_chassis.cpp`.
- `/odom`: `nav_msgs/msg/Odometry`, integrated from reported chassis velocities. This is not a raw per-wheel encoder-count topic.
- `/imu/data_raw`: `sensor_msgs/msg/Imu`; scaling must match controller IMU configuration.
- `/PowerVoltage`: `std_msgs/msg/Float32`, volts.
- Frame IDs: `odom_frame_id`, `robot_frame_id`, `gyro_frame_id`. This snapshot publishes odometry messages but does not call a TF broadcaster; provide the appropriate TF publisher separately without duplicating ownership.
- Other implemented topics cover charging, arm commands, RGB and ultrasonic data. Their presence does not establish installed hardware or verified behavior. `chassis_security` is retained unchanged and is not a certified safety interface.
- Frame layout: `lucsys_chassis_driver/include/lucsys_chassis_driver/lucsys_chassis.h`; parser: `Get_Sensor_Data()`.
- PID/PI gains and encoder resolution are in the matching STM32 project, not this ROS node.

Jetson or Raspberry Pi can host the driver when the operating system, dependencies and serial interface match. The actual build/test here used an ARM64 Linux container, not either physical computer.

## Validation

Three packages built. A pseudo-terminal peer exercised command serialization, checksum, 0.1 m/s feedback, IMU publication and 24 V test-frame decoding. These are synthetic inputs, not robot specifications. Hardware tests, disconnect behavior and vehicle safety tests were not performed. Existing compiler warnings include byte narrowing in arm-command packing; the only functional robustness change is rejecting empty serial reads before parsing; valid-frame processing and motor/safety command policy are unchanged.

See [electrical references](../../../electrical/controllers/README.md) and the [shared communication guide](../../../manuals/LucSys_ROS2_STM32_Communication_Guide.pdf). Match exact hardware before use.

## English documentation edition — 2026-11-23

Chinese source comments and accompanying TXT/Markdown explanations have been translated into English. Executable code, runtime messages, protocol strings, configuration values and firmware selection logic are unchanged. Runtime console output may therefore still contain Chinese. Changelogs describe historical source changes, not verified behavior of every delivered vehicle. Match the controller, hardware revision and firmware version before use.

TXT notes use `CHANGELOG.txt` (and `VERSION_NOTES.txt` where present); Keil project references were updated accordingly. Earlier source releases remain available for comparison. Translation does not add hardware compatibility or safety guarantees.

## Candidate cleanup and explicit startup

This customer candidate uses LucSys names without legacy device fallback or model aliases. Historical deployments have a separate repository-level compatibility overlay, excluded from customer archives. Third-party library authors and license notices are retained. Internal identifier renaming does not change controller values or protocols. Unconfirmed reference variants remain unchanged rather than being assigned to a LucSys model.

Install dependencies from the package manifests using `rosdep install --from-paths src --ignore-src -r -y`, then build and source the workspace. Base startup no longer implicitly starts EKF, model, camera or LiDAR components:

```bash
ros2 launch lucsys_chassis_driver lucsys_chassis_driver.launch.py usart_port_name:=/dev/ttyUSB0 car_mode:=<confirmed_configuration_key>
```

Select the actual serial device and confirmed controller configuration. No udev rule is installed automatically. The default /dev/lucsys_controller is never replaced by another device. Supply usart_port_name explicitly when using /dev/ttyUSB0 or another actual port. External ultrasonic avoidance requires an explicit installed package; it is never silently disabled.

To enable EKF, add `start_ekf:=true`; this requires `robot_localization`. To publish a model, install the matching existing LucSys model package and supply all of:

```text
start_state_publisher:=true model_package:=<exact_lucsys_description_package> model_name:=<existing_urdf_basename>
```

For example, `lucsys_lsr550a_mec_description` contains `lsr550a_mec.urdf`. This is a model selection example, not confirmation of a particular delivered controller/firmware. The launcher does not infer model compatibility or substitute a different model.

Sensor transforms from historical profiles are disabled by default. Publish only confirmed transforms; the standalone `robot_mode_description.launch.py` supports `publish_static_tf:=true`. Avoid publishing duplicate transforms already present in a URDF. The `lucsys_sensors.launch.py` entry starts only the base driver unless `start_lidar:=true` or `start_camera:=true` is requested. Requested external sensor drivers must be installed separately. Explicitly configured ultrasonic avoidance is never silently disabled when its package is missing.

Runtime and static validation are separate from Keil compilation and physical-robot validation. This candidate must not be treated as verified firmware for every hardware revision.

### Candidate verification

To run all registered checks after a test-enabled build:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
```

The maintenance-notice checker validates the declared notice format, not legal provenance. Third-party licenses remain unchanged. The inverse-square-root routine now uses a defined 32-bit memcpy representation instead of reading a float through a platform-sized long pointer; its original constant and Newton iteration are retained. Sanitizer tests cover 10,001 positive inputs. Virtual serial tests verify feedback and velocity frames, with and without EKF/model startup; these are not physical-robot tests.
