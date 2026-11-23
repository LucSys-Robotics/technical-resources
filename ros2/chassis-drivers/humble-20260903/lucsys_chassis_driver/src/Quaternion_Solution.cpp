// Copyright 2026 LucSys Robotics - integration and maintenance changes only.
//
// LucSys Robotics distributes this source under supplier authorization.
// LucSys Robotics is responsible for integration and maintenance only.
// This notice does not grant an additional open-source license.
// Third-party components retain their respective licenses and notices.

#include "lucsys_chassis_driver/Quaternion_Solution.h"
#include <cstdint>
#include <cstring>

// Sampling frequency
#define SAMPLING_FREQ 20.0f
// Instantiate the onboard IMU object //Instantiate the onboard IMU object
sensor_msgs::msg::Imu onboard_imu;

/**************************************
Date: May 31, 2020
Function: Reciprocal of the square root Used to find quaternions
***************************************/
float InvSqrt(float number)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "Requires 32-bit float");
  uint32_t bits;
  const float x = number * 0.5F;
  float y = number;
  std::memcpy(&bits, &y, sizeof(bits));
  bits = 0x5f375a86 - (bits >> 1);
  std::memcpy(&y, &bits, sizeof(y));
  y = y * (1.5F - (x * y * y));

  return y;
}
/**************************************
Date: May 31, 2020
Function: Quaternion solve
***************************************/
// 2 * proportional gain (Kp)
volatile float twoKp = 1.0f;
// 2 * integral gain (Ki)
volatile float twoKi = 0.0f;
// quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// integral error terms scaled by Ki
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

volatile uint8_t imu_freeze = 0;

void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if (imu_freeze) {
    printf("imu data freeze\n");
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
  // normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // First normalize the three-dimensional accelerometer vector by dividing by its magnitude
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Convert quaternions into the three elements in the third row of the direction cosine
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    // Error is the sum of the cross product of the estimated and measured directions of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // Calculate and apply point feedback (if enabled)
    if (twoKi > 0.0f) {
      // integral error scaled by Ki
      integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
      // apply integral feedback
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      // prevent integral windup
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  // pre-multiply common factors
  gx *= (0.5f * (1.0f / SAMPLING_FREQ));
  gy *= (0.5f * (1.0f / SAMPLING_FREQ));
  gz *= (0.5f * (1.0f / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  onboard_imu.orientation.w = q0;
  onboard_imu.orientation.x = q1;
  onboard_imu.orientation.y = q2;
  onboard_imu.orientation.z = q3;
}
