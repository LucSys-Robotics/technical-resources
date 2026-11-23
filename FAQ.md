# Development FAQ

[Browse by robot model](models/README.md) · [Development Source](https://github.com/LucSys-Robotics/technical-resources/releases/tag/development-source-20261123) · [Technical Documentation](manuals/README.md)

## Is STM32 firmware source available?

Yes. Complete, configuration-specific Keil projects are directly downloadable from each [model resource page](models/README.md). Select the controller, firmware version and chassis target before building. LSR550A V1/V2/V2.1 include chassis-side arm-command integration; separate arm-actuator firmware is outside those archives.

## What is included in the ROS 2 and model downloads?

The [Humble driver](ros2/chassis-drivers/humble-20260903/README.md) includes the chassis driver, message definitions and serial dependency. [Model packages](platforms/README.md) contain URDF, meshes and display launch resources. These are separate downloads; they are not a complete navigation or sensor SDK. Install optional sensor/localization packages as described in the setup guide.

## Do I need to request the public packages by email?

No. Download the source, models and published documents directly from the model page. Order-specific software images, custom drawings and services are governed by the selected order. Public source access does not itself grant a new open-source license; retain the terms supplied with each package.

## How do I build and select the firmware?

Use the [project-specific STM32 guides](firmware/README.md) and [Keil setup](firmware/build/README.md). Match controller revision, motors, encoders and the selected target. LSR200Auto shares the documented LSR200 source reference; that does not establish charging, docking or safety behavior for every configuration. LSR550A has its own version selector and must not use a substitute LSR550 project.

## Can I change motor PID settings or use my own computer?

PI/PID and encoder settings are in the matched STM32 project or motor driver; the ROS node does not supply universal gains. A computer with the stated ROS environment and a compatible serial interface can host the driver. Follow the [communication guide](manuals/LucSys_ROS2_STM32_Communication_Guide.pdf) and exact electrical drawing. Custom circuitry, firmware, sensor mounting and system validation remain project-specific engineering work.

## Where are wiring diagrams and CAD files?

Each model page links its available manuals and electrical references. [Controller schematics](electrical/controllers/README.md) and [controller CAD](cad/controllers/README.md) are revision-specific. A controller schematic or chassis drawing is not a complete robotic-arm wiring diagram; an arm/component CAD model is not a chassis top-plate engineering drawing. Check the stated scope rather than inferring missing connections or dimensions.

## What feedback and stop functions does the ROS driver provide?

The driver exposes odometry, IMU and voltage feedback as described in its [interfaces](ros2/chassis-drivers/humble-20260903/README.md#interfaces). Odometry is not raw individual-wheel encoder counts. Software zero velocity is not physical power isolation, and a serial read timeout is not a motor-command watchdog. Match the delivered firmware's stop and disconnect behavior before operation.

## Where can I select and order a robot?

Open [Browse by Robot Model](models/README.md) and use **Product Information & Ordering** for the verified product pages, hardware options and ordering information. Product options do not override the revision limits in a technical drawing or source project.
