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

Platform-specific technical resources, including CAD drawings, URDF models, meshes, and ROS integration documentation, are being added progressively.

For detailed product specifications and available configurations, please visit our [official website](https://www.lucsysrobotics.com).

For additional technical documents or pre-purchase evaluation resources, contact us at info@lucsysrobotics.com.

## Before integration

- Confirm the exact platform configuration, voltage supply, and available connections.
- Use the electrical drawing for that product and hardware revision rather than one for another robot.
- Check the ROS/ROS 2 software version against the package provided for your configuration.
- Verify motion-axis directions and safety behavior on the delivered controller before sending live motion commands.

**Document scope:** These guides provide engineering reference information, not a substitute for product-specific specifications, service instructions or safety testing. Source code, firmware and deployment images are not part of this public collection. For product-specific connection drawings and software delivery information, contact LucSys Robotics.
