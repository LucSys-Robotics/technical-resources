# LucSys Robotics — LSV550

ROS 2 Humble robot description.

Extract this package into your ROS 2 workspace `src` directory, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lucsys_lsv550_description
source install/setup.bash
ros2 launch lucsys_lsv550_description display.launch.py
```

Select a configuration with `model:=<configuration>`. Use `gui:=false` for the non-interactive joint-state publisher, `rviz:=false` to disable RViz, and `use_sim_time:=true` only when a simulation clock is available.

| Configuration | Product |
| --- | --- |
| `lsv550_4wd` | LSV550-4WD |
| `lsv550_akm` | LSV550-AKM |
| `lsv550_mec` | LSV550-MEC |

The display launch publishes preview joint states. It does not start hardware drivers, motion control, navigation, MoveIt, or a physics simulator. Distribution authorization is described in LICENSE. Third-party attribution is retained in NOTICE.
