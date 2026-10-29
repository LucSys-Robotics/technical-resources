# LucSys Robotics — LSE550

ROS 2 Humble robot description.

Extract this package into your ROS 2 workspace `src` directory, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lucsys_lse550_description
source install/setup.bash
ros2 launch lucsys_lse550_description display.launch.py
```

Select a configuration with `model:=<configuration>`. Use `gui:=false` for the non-interactive joint-state publisher, `rviz:=false` to disable RViz, and `use_sim_time:=true` only when a simulation clock is available.

| Configuration | Product |
| --- | --- |
| `lse550_4wd_four_axis_arm` | LSE550-4WD |
| `lse550_4wd` | LSE550-4WD |
| `lse550_akm` | LSE550-AKM |
| `lse550_diff` | LSE550-DIFF |
| `lse550_mec_four_axis_arm` | LSE550-MEC |
| `lse550_mec_six_axis_arm` | LSE550-MEC |
| `lse550_mec` | LSE550-MEC |
| `lse550_omni` | LSE550-OMNI |
| `lse550_tank_four_axis_arm` | LSE550-TANK |
| `lse550_tank` | LSE550-TANK |

The display launch publishes preview joint states. It does not start hardware drivers, motion control, navigation, MoveIt, or a physics simulator. Distribution authorization is described in LICENSE. Third-party attribution is retained in NOTICE.
