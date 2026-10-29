# LucSys Robotics — LSR550-AKM-Flagship Independent Suspension

ROS 2 Humble robot description.

Extract this package into your ROS 2 workspace `src` directory, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lucsys_lsr550_akm_flagship_independent_suspension_description
source install/setup.bash
ros2 launch lucsys_lsr550_akm_flagship_independent_suspension_description display.launch.py
```

Select a configuration with `model:=<configuration>`. Use `gui:=false` for the non-interactive joint-state publisher, `rviz:=false` to disable RViz, and `use_sim_time:=true` only when a simulation clock is available.

| Configuration | Product |
| --- | --- |
| `lsr550_akm_flagship_independent_suspension` | LSR550-AKM-Flagship Independent Suspension |

The display launch publishes preview joint states. It does not start hardware drivers, motion control, navigation, MoveIt, or a physics simulator. Distribution authorization is described in LICENSE. Third-party attribution is retained in NOTICE.
