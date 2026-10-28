# Control and Feedback Interfaces

**LucSys Robotics | Serial, CAN and Host-to-Chassis Integration**

## 1. Architecture and applicability

A host computer runs navigation and robot applications; the embedded chassis controller handles wheel-level motion control and sensor/drive feedback.

The **serial** and **CAN** control/feedback protocol families documented below apply across LucSys Robotics chassis platforms. The supported motion axes still depend on drivetrain hardware: for example, a fixed-wheel differential chassis cannot produce independent lateral translation. The actual connector, communication rate and controller revision must be matched to the delivered robot.

| Layer | Function |
|---|---|
| ROS / host application | Planning, navigation, high-level velocity targets |
| Communication interface | Transfers motion targets and controller feedback |
| Embedded controller | Converts targets into wheel commands and coordinates drive functions |
| Motors and sensors | Perform motion and return measurements |

## 2. Motion-command conventions

The documented command fields are **X longitudinal velocity**, **Y lateral velocity**, and **Z yaw rate**. X/Y velocity values use signed integers in **mm/s**; yaw-rate values use signed integers scaled by **1,000** from rad/s. Multibyte integers are transmitted **most-significant byte first**.

For ROS applications, the conventional body frame is X forward, Y left and positive yaw counterclockwise. The original controller descriptions contain inconsistent wording about the sign of the Z rotation axis. **Do not assume the controller yaw sign is identical to the ROS convention until verified on the actual firmware.** No precomputed turn-command hex examples are supplied in this guide.

## 3. Serial interface

The common serial message structures are:

| Direction | Length | Structure |
|---|---:|---|
| Host → chassis | 11 bytes | Header, two reserved bytes, X/Y/Z target velocities, XOR checksum, tail |
| Chassis → host | 24 bytes | Header, motor-enable flag, X/Y/Z measured velocities, raw XYZ acceleration and XYZ gyroscope data, battery voltage, XOR checksum, tail |

**Host-to-chassis command (byte positions):**

| Bytes | Field |
|---|---|
| 0 | Header `0x7B` |
| 1–2 | Reserved |
| 3–4 | X command, signed int16, mm/s |
| 5–6 | Y command, signed int16, mm/s |
| 7–8 | Z command, signed int16, rad/s × 1000 |
| 9 | XOR of bytes 0–8 |
| 10 | Tail `0x7D` |

**Chassis-to-host feedback (byte positions):**

| Bytes | Field |
|---|---|
| 0 | Header `0x7B` |
| 1 | `flag_stop` — `0x00` is documented as motor enabled; other values indicate disabled |
| 2–7 | X, Y and Z measured body velocities (3 × signed int16) |
| 8–13 | Raw X, Y and Z accelerometer readings (3 × signed int16) |
| 14–19 | Raw X, Y and Z gyroscope readings (3 × signed int16) |
| 20–21 | Battery voltage, mV |
| 22 | XOR of bytes 0–21 |
| 23 | Tail `0x7D` |

The documented ROS host-to-controller example uses **115200 baud**. Confirm the selected controller port and its active configuration before connecting. A firmware/programming port may not be suitable for continuous ROS communication.

## 4. CAN interface

The CAN protocol uses **11-bit standard identifiers** and eight-byte data frames:

| CAN ID | Direction | Contents |
|---|---|---|
| `0x101` | Chassis → host | Header, motor-enable flag, X/Y/Z velocity values |
| `0x102` | Chassis → host | Raw XYZ acceleration and X gyroscope data |
| `0x103` | Chassis → host | Raw Y/Z gyroscope data, battery voltage, checksum, tail |
| `0x181` | Host → chassis | X/Y/Z target velocity values and two reserved bytes |

The velocity units, scaling and byte order are the same as those described above for serial communication. The technical documentation identifies **500 kbit/s and 1 Mbit/s** controller variants; the correct CAN bit rate must be confirmed for each product configuration rather than inferred from its retail model name.

## 5. Ackermann steering command conversion

The documented command-conversion method applies to **LucSys Robotics Ackermann chassis configurations**. A requested front-wheel angle must be converted using that particular vehicle's geometry; the low-level body yaw-rate command is not a direct steering-angle field.

In an ideal bicycle model, with longitudinal speed $v$, wheelbase $L$ and equivalent steering angle $\delta$:

$$\omega = \frac{v}{L}\tan(\delta).$$

For a physical left or right front-wheel reference, take its location and the track width into account before converting to body yaw rate. The appropriate sign, steering limits and reference wheel must be verified for the selected chassis. Do not reuse fixed angles, wheelbase values or hex command examples from a different Ackermann model.

## 6. Feedback and integration checks

Raw accelerometer and gyroscope fields require the scaling and axis mapping configured in the installed firmware; they must not be treated as SI-unit values without conversion. Velocity feedback from wheel encoders is not the same as an absolute pose estimate. ROS odometry and sensor fusion belong to the higher-level software configuration.

Before direct control:

1. Confirm the robot's controller revision, connection pinout and communication speed.
2. Read a complete status frame and validate the framing and data fields.
3. Verify X/Y/Z direction signs with the chassis secured or in a controlled low-speed test.
4. Test the zero-motion command, emergency stop, remote takeover and communication-loss behavior independently.
5. Use the hardware and mechanical specifications for the exact platform; do not transplant command tables or electrical pinouts between robot models.

The message descriptions above are intended for technical evaluation and do not constitute a safety certification or a guarantee of firmware-specific stop behavior. Source code and firmware are provided separately as applicable to the purchased configuration.
