# LucSys Robotics — ROS 2 Humble Chassis Driver

Source snapshot: 2026-09-03. Ubuntu 22.04 / ROS 2 Humble. The package includes the chassis driver, message definitions and serial dependency.

## Build

Extract the driver ZIP, then copy its three package folders to a workspace:

```bash
mkdir -p ~/lucsys_driver_ws/src
cp -R lucsys_ros2_humble_20260903/{lucsys_chassis_driver,lucsys_robot_msg,serial_ros2} ~/lucsys_driver_ws/src/
cd ~/lucsys_driver_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

When using the repository source archive instead, copy the same three folders from `ros2/chassis-drivers/humble-20260903/`. ROS 2 build and virtual serial communication checks passed on Humble ARM64.

## Connect

Match the controller protocol and configuration first. The default protocol uses 115200 baud, 11-byte command frames, 24-byte chassis feedback frames, 0x7B/0x7D boundaries and XOR checksum. Grant the account serial-device access, then select the actual device:

```bash
ros2 launch lucsys_chassis_driver lucsys_chassis_driver.launch.py usart_port_name:=/dev/ttyUSB0
ros2 topic echo /odom
ros2 topic echo /imu/data_raw
ros2 topic echo /PowerVoltage
```

The default device is `/dev/lucsys_controller`; no other device is selected automatically. The default node name is `lucsys_robot`. The `car_mode` parameter is descriptive in this source snapshot; chassis kinematics and encoder calibration belong to the matched STM32 firmware. Do not use a model string as proof of hardware compatibility.

## Optional localization and models

Base startup does not require EKF, a model or sensor drivers. To start EKF, add `start_ekf:=true`. To publish the exact installed robot model, also supply `start_state_publisher:=true`, `model_package:=your_lucsys_description_package` and `model_name:=the_existing_urdf_basename`. For the LSR550A Mecanum geometry package these values are `lucsys_lsr550a_mec_description` and `lsr550a_mec`. Geometry selection does not select controller firmware.

Install the corresponding public model ZIP in workspace `src` and rebuild. Static sensor transforms are disabled by default; enable only measured/configuration-matched transforms, without duplicating URDF transforms. External LiDAR, camera, IMU and ultrasonic drivers require their respective packages. An explicitly enabled ultrasonic function requires its installed package; it is not silently disabled.

## Interfaces

- `/cmd_vel`: geometry_msgs/Twist; command serialization is in `Cmd_Vel_Callback()`.
- `/odom`: nav_msgs/Odometry from reported chassis velocity.
- `/imu/data_raw`: sensor_msgs/Imu; scaling must match the controller.
- `/PowerVoltage`: std_msgs/Float32, volts.
- Frame definitions: `lucsys_chassis_driver/include/lucsys_chassis_driver/lucsys_chassis.h`.
- Parser and command interfaces: `lucsys_chassis_driver/src/lucsys_chassis.cpp`.

Software zero velocity is not a hardware emergency stop. The serial read timeout is not a motor-command watchdog. Match the delivered firmware's stop, disconnect and power-isolation behavior before operation. Commission with motors safely restrained and emergency-stop access. Optional charging and arm interfaces require the corresponding hardware/firmware configuration.

## Software checks

```bash
colcon test --executor sequential
colcon test-result --verbose
```

Virtual serial checks exercise positive, negative, multi-axis and zero-velocity frames plus odometry, IMU and voltage feedback. These describe software test inputs, not vehicle performance specifications.

[Communication guide](https://github.com/LucSys-Robotics/technical-resources/blob/main/manuals/LucSys_ROS2_STM32_Communication_Guide.pdf) · [Electrical references](https://github.com/LucSys-Robotics/technical-resources/blob/main/electrical/controllers/README.md)

LucSys Robotics maintains the distribution. See the package NOTICE and third-party license files for the applicable terms.
