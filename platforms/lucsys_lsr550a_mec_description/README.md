# LucSys Robotics — LSR550A Technical Resources

## Technical Documentation

| Resource | Description | Download |
| --- | --- | --- |
| Development Manual | System architecture, robotic arm interfaces and development reference | [PDF](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/manuals/LSR550A_Development_Manual.pdf) |
| C50C Controller Schematic | Controller connections, power interfaces, CAN and motor-driver references | [PDF](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/electrical/LSR550A/LSR550A_C50C_Controller_Schematic.pdf) |
| Robotic Arm STEP Model | 3D mechanical assembly reference for the robotic arm | [STEP](https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/main/cad/LSR550A/LSR550A_Robotic_Arm.step) |
| ROS 2 Mesh / URDF | Robot description, meshes and RViz resources | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550a_mec_description.zip) |

The C50C schematic is a controller-level electrical reference. It is not a complete robot wiring-harness diagram or an approved wiring diagram for a specific delivered hardware revision. Confirm the controller revision and connector assignments against the delivered hardware before making connections.

This STEP file provides a robotic arm assembly reference and does not represent a complete chassis CAD model.

## ROS 2 Humble Model

ROS 2 Humble robot description.

Extract this package into your ROS 2 workspace `src` directory, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lucsys_lsr550a_mec_description
source install/setup.bash
ros2 launch lucsys_lsr550a_mec_description display.launch.py
```

Select a configuration with `model:=<configuration>`. Use `gui:=false` for the non-interactive joint-state publisher, `rviz:=false` to disable RViz, and `use_sim_time:=true` only when a simulation clock is available.

| Configuration | Product |
| --- | --- |
| `lsr550a_mec` | LSR550A-MEC |

The display launch publishes preview joint states. It does not start hardware drivers, motion control, navigation, MoveIt, or a physics simulator. Distribution authorization is described in LICENSE. Third-party attribution is retained in NOTICE.
