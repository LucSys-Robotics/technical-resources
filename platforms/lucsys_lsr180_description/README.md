# LucSys Robotics — LSR180

ROS 2 Humble robot description.

Extract this package into your ROS 2 workspace `src` directory, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lucsys_lsr180_description
source install/setup.bash
ros2 launch lucsys_lsr180_description display.launch.py
```

Select a configuration with `model:=<configuration>`. Use `gui:=false` for the non-interactive joint-state publisher, `rviz:=false` to disable RViz, and `use_sim_time:=true` only when a simulation clock is available.

| Configuration | Product |
| --- | --- |
| `lsr180` | LSR180 |

The display launch publishes preview joint states. It does not start hardware drivers, motion control, navigation, MoveIt, or a physics simulator. Distribution authorization is described in LICENSE. Third-party attribution is retained in NOTICE.
