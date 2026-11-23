// Copyright 2026 LucSys Robotics - integration and maintenance changes only.
//
// LucSys Robotics distributes this source under supplier authorization.
// LucSys Robotics is responsible for integration and maintenance only.
// This notice does not grant an additional open-source license.
// Third-party components retain their respective licenses and notices.

#ifndef LUCSYS_CHASSIS_DRIVER__QUATERNION_SOLUTION_H_
#define LUCSYS_CHASSIS_DRIVER__QUATERNION_SOLUTION_H_

#include <cstdint>
#include <sensor_msgs/msg/imu.hpp>

extern volatile uint8_t imu_freeze;
extern volatile float q0, q1, q2, q3;

float InvSqrt(float number);
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az);
// External onboard IMU data // External onboard IMU data
extern sensor_msgs::msg::Imu onboard_imu;

#endif  // LUCSYS_CHASSIS_DRIVER__QUATERNION_SOLUTION_H_
