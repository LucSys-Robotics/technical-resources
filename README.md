# LucSys Robotics — Mobile Robot Technical Resources

**Hardware integration · Chassis control · ROS development**

This technical resource collection helps engineering teams evaluate wheeled mobile robot hardware, communication interfaces and integration requirements before purchasing or deploying a platform.

## Technical guides

| Guide | Description |
|---|---|
| [Hardware and System Overview](Hardware_and_System_Overview.md) | Robot architecture, controllers, motors, encoders, sensors, and ROS integration |
| [Electrical Integration Overview](Electrical_Integration_Overview.md) | Power distribution, host connections and product-specific wiring reference coverage |
| [Chassis Kinematics](Chassis_Kinematics.md) | Differential, four-wheel skid-steer, Ackermann, Mecanum and omnidirectional models |
| [Control and Feedback Interfaces](Control_and_Feedback_Interfaces.md) | Shared serial/CAN protocols, motion commands, feedback fields and integration checks |

## Robot Platforms

LucSys Robotics offers a range of mobile robot platforms for research, education, and industrial applications.

ROS 2 Humble URDF descriptions, meshes, launch files and RViz configurations are available below. Other platform-specific resources are being added progressively.

For detailed product specifications and available configurations, please visit our [official website](https://www.lucsysrobotics.com).

For additional technical documents or pre-purchase evaluation resources, contact us at info@lucsysrobotics.com.

## ROS 2 Humble platform downloads

25 independent description packages provide 36 URDF configurations. LSE550 includes 10 configurations in one package; LSV550 includes 3 in one package. Each package includes its own meshes, launch files, RViz configurations, LICENSE and NOTICE.

Download one package ZIP below, or browse the [release page](https://github.com/LucSys-Robotics/technical-resources/releases/tag/ros2-humble-v1.0.0). Verify downloaded ZIPs using the release [SHA256SUMS.txt](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/SHA256SUMS.txt). ZIP downloads are stored in Releases, not duplicated in Git.

| Platform / sales configuration | ROS 2 package and documentation | URDF configurations | Download |
| --- | --- | ---: | --- |
| LSE550 | [lucsys_lse550_description](platforms/lucsys_lse550_description/) | 10 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lse550_description.zip) |
| LSR100 | [lucsys_lsr100_description](platforms/lucsys_lsr100_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr100_description.zip) |
| LSR180 | [lucsys_lsr180_description](platforms/lucsys_lsr180_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr180_description.zip) |
| LSR200 | [lucsys_lsr200_description](platforms/lucsys_lsr200_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr200_description.zip) |
| LSR200Auto-4WD | [lucsys_lsr200auto_4wd_description](platforms/lucsys_lsr200auto_4wd_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr200auto_4wd_description.zip) |
| LSR200Auto-Ackermann Lite | [lucsys_lsr200auto_ackermann_lite_description](platforms/lucsys_lsr200auto_ackermann_lite_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr200auto_ackermann_lite_description.zip) |
| LSR200Auto-AKM Pro | [lucsys_lsr200auto_akm_pro_description](platforms/lucsys_lsr200auto_akm_pro_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr200auto_akm_pro_description.zip) |
| LSR200Auto-AKM Ultra | [lucsys_lsr200auto_akm_ultra_description](platforms/lucsys_lsr200auto_akm_ultra_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr200auto_akm_ultra_description.zip) |
| LSR300mini | [lucsys_lsr300mini_description](platforms/lucsys_lsr300mini_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr300mini_description.zip) |
| LSR300pro | [lucsys_lsr300pro_description](platforms/lucsys_lsr300pro_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr300pro_description.zip) |
| LSR550-2WD | [lucsys_lsr550_2wd_description](platforms/lucsys_lsr550_2wd_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_2wd_description.zip) |
| LSR550-4WD-Premium Independent Suspension | [lucsys_lsr550_4wd_premium_independent_suspension_description](platforms/lucsys_lsr550_4wd_premium_independent_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_4wd_premium_independent_suspension_description.zip) |
| LSR550-4WD-Premium Oscillating Axle Suspension | [lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description](platforms/lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description.zip) |
| LSR550-AKM-Flagship Independent Suspension | [lucsys_lsr550_akm_flagship_independent_suspension_description](platforms/lucsys_lsr550_akm_flagship_independent_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_akm_flagship_independent_suspension_description.zip) |
| LSR550-AKM-Premium Oscillating Axle Suspension | [lucsys_lsr550_akm_premium_oscillating_axle_suspension_description](platforms/lucsys_lsr550_akm_premium_oscillating_axle_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_akm_premium_oscillating_axle_suspension_description.zip) |
| LSR550-AKM-Ultimate Oscillating Axle Suspension | [lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description](platforms/lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description.zip) |
| LSR550-MEC-Flagship Independent Suspension | [lucsys_lsr550_mec_flagship_independent_suspension_description](platforms/lucsys_lsr550_mec_flagship_independent_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_mec_flagship_independent_suspension_description.zip) |
| LSR550-MEC-Premium Independent Suspension | [lucsys_lsr550_mec_premium_independent_suspension_description](platforms/lucsys_lsr550_mec_premium_independent_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_mec_premium_independent_suspension_description.zip) |
| LSR550-MEC-Premium Oscillating Axle Suspension | [lucsys_lsr550_mec_premium_oscillating_axle_suspension_description](platforms/lucsys_lsr550_mec_premium_oscillating_axle_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_mec_premium_oscillating_axle_suspension_description.zip) |
| LSR550-MEC-Ultimate Oscillating Axle Suspension | [lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description](platforms/lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description.zip) |
| LSR550-Omni-Premium | [lucsys_lsr550_omni_premium_description](platforms/lucsys_lsr550_omni_premium_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_omni_premium_description.zip) |
| LSR550-Omni-Ultimate | [lucsys_lsr550_omni_ultimate_description](platforms/lucsys_lsr550_omni_ultimate_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550_omni_ultimate_description.zip) |
| LSR550A-4WD | [lucsys_lsr550a_4wd_description](platforms/lucsys_lsr550a_4wd_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550a_4wd_description.zip) |
| LSR550A-MEC | [lucsys_lsr550a_mec_description](platforms/lucsys_lsr550a_mec_description/) | 1 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsr550a_mec_description.zip) |
| LSV550 | [lucsys_lsv550_description](platforms/lucsys_lsv550_description/) | 3 | [ZIP](https://github.com/LucSys-Robotics/technical-resources/releases/download/ros2-humble-v1.0.0/lucsys_lsv550_description.zip) |

### Configuration index

| Platform | URDF configuration | Package |
| --- | --- | --- |
| LSE550-4WD | [`lse550_4wd_four_axis_arm`](platforms/lucsys_lse550_description/urdf/lse550_4wd_four_axis_arm.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-4WD | [`lse550_4wd`](platforms/lucsys_lse550_description/urdf/lse550_4wd.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-AKM | [`lse550_akm`](platforms/lucsys_lse550_description/urdf/lse550_akm.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-DIFF | [`lse550_diff`](platforms/lucsys_lse550_description/urdf/lse550_diff.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-MEC | [`lse550_mec_four_axis_arm`](platforms/lucsys_lse550_description/urdf/lse550_mec_four_axis_arm.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-MEC | [`lse550_mec_six_axis_arm`](platforms/lucsys_lse550_description/urdf/lse550_mec_six_axis_arm.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-MEC | [`lse550_mec`](platforms/lucsys_lse550_description/urdf/lse550_mec.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-OMNI | [`lse550_omni`](platforms/lucsys_lse550_description/urdf/lse550_omni.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-TANK | [`lse550_tank_four_axis_arm`](platforms/lucsys_lse550_description/urdf/lse550_tank_four_axis_arm.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSE550-TANK | [`lse550_tank`](platforms/lucsys_lse550_description/urdf/lse550_tank.urdf) | [lucsys_lse550_description](platforms/lucsys_lse550_description/) |
| LSR100 | [`lsr100`](platforms/lucsys_lsr100_description/urdf/lsr100.urdf) | [lucsys_lsr100_description](platforms/lucsys_lsr100_description/) |
| LSR180 | [`lsr180`](platforms/lucsys_lsr180_description/urdf/lsr180.urdf) | [lucsys_lsr180_description](platforms/lucsys_lsr180_description/) |
| LSR200 | [`lsr200`](platforms/lucsys_lsr200_description/urdf/lsr200.urdf) | [lucsys_lsr200_description](platforms/lucsys_lsr200_description/) |
| LSR200Auto-4WD | [`lsr200auto_4wd`](platforms/lucsys_lsr200auto_4wd_description/urdf/lsr200auto_4wd.urdf) | [lucsys_lsr200auto_4wd_description](platforms/lucsys_lsr200auto_4wd_description/) |
| LSR200Auto-Ackermann Lite | [`lsr200auto_ackermann_lite`](platforms/lucsys_lsr200auto_ackermann_lite_description/urdf/lsr200auto_ackermann_lite.urdf) | [lucsys_lsr200auto_ackermann_lite_description](platforms/lucsys_lsr200auto_ackermann_lite_description/) |
| LSR200Auto-AKM Pro | [`lsr200auto_akm_pro`](platforms/lucsys_lsr200auto_akm_pro_description/urdf/lsr200auto_akm_pro.urdf) | [lucsys_lsr200auto_akm_pro_description](platforms/lucsys_lsr200auto_akm_pro_description/) |
| LSR200Auto-AKM Ultra | [`lsr200auto_akm_ultra`](platforms/lucsys_lsr200auto_akm_ultra_description/urdf/lsr200auto_akm_ultra.urdf) | [lucsys_lsr200auto_akm_ultra_description](platforms/lucsys_lsr200auto_akm_ultra_description/) |
| LSR300mini | [`lsr300mini`](platforms/lucsys_lsr300mini_description/urdf/lsr300mini.urdf) | [lucsys_lsr300mini_description](platforms/lucsys_lsr300mini_description/) |
| LSR300pro | [`lsr300pro`](platforms/lucsys_lsr300pro_description/urdf/lsr300pro.urdf) | [lucsys_lsr300pro_description](platforms/lucsys_lsr300pro_description/) |
| LSR550-2WD | [`lsr550_2wd`](platforms/lucsys_lsr550_2wd_description/urdf/lsr550_2wd.urdf) | [lucsys_lsr550_2wd_description](platforms/lucsys_lsr550_2wd_description/) |
| LSR550-4WD-Premium Independent Suspension | [`lsr550_4wd_premium_independent_suspension`](platforms/lucsys_lsr550_4wd_premium_independent_suspension_description/urdf/lsr550_4wd_premium_independent_suspension.urdf) | [lucsys_lsr550_4wd_premium_independent_suspension_description](platforms/lucsys_lsr550_4wd_premium_independent_suspension_description/) |
| LSR550-4WD-Premium Oscillating Axle Suspension | [`lsr550_4wd_premium_oscillating_axle_suspension`](platforms/lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description/urdf/lsr550_4wd_premium_oscillating_axle_suspension.urdf) | [lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description](platforms/lucsys_lsr550_4wd_premium_oscillating_axle_suspension_description/) |
| LSR550-AKM-Flagship Independent Suspension | [`lsr550_akm_flagship_independent_suspension`](platforms/lucsys_lsr550_akm_flagship_independent_suspension_description/urdf/lsr550_akm_flagship_independent_suspension.urdf) | [lucsys_lsr550_akm_flagship_independent_suspension_description](platforms/lucsys_lsr550_akm_flagship_independent_suspension_description/) |
| LSR550-AKM-Premium Oscillating Axle Suspension | [`lsr550_akm_premium_oscillating_axle_suspension`](platforms/lucsys_lsr550_akm_premium_oscillating_axle_suspension_description/urdf/lsr550_akm_premium_oscillating_axle_suspension.urdf) | [lucsys_lsr550_akm_premium_oscillating_axle_suspension_description](platforms/lucsys_lsr550_akm_premium_oscillating_axle_suspension_description/) |
| LSR550-AKM-Ultimate Oscillating Axle Suspension | [`lsr550_akm_ultimate_oscillating_axle_suspension`](platforms/lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description/urdf/lsr550_akm_ultimate_oscillating_axle_suspension.urdf) | [lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description](platforms/lucsys_lsr550_akm_ultimate_oscillating_axle_suspension_description/) |
| LSR550-MEC-Flagship Independent Suspension | [`lsr550_mec_flagship_independent_suspension`](platforms/lucsys_lsr550_mec_flagship_independent_suspension_description/urdf/lsr550_mec_flagship_independent_suspension.urdf) | [lucsys_lsr550_mec_flagship_independent_suspension_description](platforms/lucsys_lsr550_mec_flagship_independent_suspension_description/) |
| LSR550-MEC-Premium Independent Suspension | [`lsr550_mec_premium_independent_suspension`](platforms/lucsys_lsr550_mec_premium_independent_suspension_description/urdf/lsr550_mec_premium_independent_suspension.urdf) | [lucsys_lsr550_mec_premium_independent_suspension_description](platforms/lucsys_lsr550_mec_premium_independent_suspension_description/) |
| LSR550-MEC-Premium Oscillating Axle Suspension | [`lsr550_mec_premium_oscillating_axle_suspension`](platforms/lucsys_lsr550_mec_premium_oscillating_axle_suspension_description/urdf/lsr550_mec_premium_oscillating_axle_suspension.urdf) | [lucsys_lsr550_mec_premium_oscillating_axle_suspension_description](platforms/lucsys_lsr550_mec_premium_oscillating_axle_suspension_description/) |
| LSR550-MEC-Ultimate Oscillating Axle Suspension | [`lsr550_mec_ultimate_oscillating_axle_suspension`](platforms/lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description/urdf/lsr550_mec_ultimate_oscillating_axle_suspension.urdf) | [lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description](platforms/lucsys_lsr550_mec_ultimate_oscillating_axle_suspension_description/) |
| LSR550-Omni-Premium | [`lsr550_omni_premium`](platforms/lucsys_lsr550_omni_premium_description/urdf/lsr550_omni_premium.urdf) | [lucsys_lsr550_omni_premium_description](platforms/lucsys_lsr550_omni_premium_description/) |
| LSR550-Omni-Ultimate | [`lsr550_omni_ultimate`](platforms/lucsys_lsr550_omni_ultimate_description/urdf/lsr550_omni_ultimate.urdf) | [lucsys_lsr550_omni_ultimate_description](platforms/lucsys_lsr550_omni_ultimate_description/) |
| LSR550A-4WD | [`lsr550a_4wd`](platforms/lucsys_lsr550a_4wd_description/urdf/lsr550a_4wd.urdf) | [lucsys_lsr550a_4wd_description](platforms/lucsys_lsr550a_4wd_description/) |
| LSR550A-MEC | [`lsr550a_mec`](platforms/lucsys_lsr550a_mec_description/urdf/lsr550a_mec.urdf) | [lucsys_lsr550a_mec_description](platforms/lucsys_lsr550a_mec_description/) |
| LSV550-4WD | [`lsv550_4wd`](platforms/lucsys_lsv550_description/urdf/lsv550_4wd.urdf) | [lucsys_lsv550_description](platforms/lucsys_lsv550_description/) |
| LSV550-AKM | [`lsv550_akm`](platforms/lucsys_lsv550_description/urdf/lsv550_akm.urdf) | [lucsys_lsv550_description](platforms/lucsys_lsv550_description/) |
| LSV550-MEC | [`lsv550_mec`](platforms/lucsys_lsv550_description/urdf/lsv550_mec.urdf) | [lucsys_lsv550_description](platforms/lucsys_lsv550_description/) |

### Install and preview

Extract the selected ZIP into `~/lucsys_ws/src/`. Alternatively, clone this repository and copy the selected package directories from `platforms/` into your workspace `src/` directory. From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch lucsys_lsr200_description display.launch.py
```

Replace the example package name with the selected package. For LSE550 and LSV550, choose a configuration with `model:=<configuration>` from the index above. Use `gui:=false` for the non-GUI joint-state publisher or `rviz:=false` to disable RViz. The preview does not start hardware drivers or motion controllers. Multiple packages can be installed together; simultaneous model previews require separate namespaces and TF prefixes.

LSR200 and LSR200Auto-4WD use the same verified base mechanical model. LSR200Auto-AKM Pro and AKM Ultra also share a base mechanical model. No unverified differences in sensors, dimensions or mechanical structure are implied by these sales configurations.

### Model usage terms

These model files are not distributed under an open-source license. Redistribution, resale or public republication requires prior written permission from the applicable rights holder(s). Read the LICENSE and NOTICE included in each package; third-party notices remain applicable. Contact [LucSys Robotics](https://www.lucsysrobotics.com) at info@lucsysrobotics.com.

## Before integration

- Confirm the exact platform configuration, voltage supply, and available connections.
- Use the electrical drawing for that product and hardware revision rather than one for another robot.
- Check the ROS/ROS 2 software version against the package provided for your configuration.
- Verify motion-axis directions and safety behavior on the delivered controller before sending live motion commands.

**Document scope:** These guides provide engineering reference information, not a substitute for product-specific specifications, service instructions or safety testing. Source code, firmware and deployment images are not part of this public collection. For product-specific connection drawings and software delivery information, contact LucSys Robotics.
