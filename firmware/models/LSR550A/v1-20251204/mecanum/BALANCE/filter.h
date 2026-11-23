#ifndef __FILTER_H
#define __FILTER_H
#include "system.h"
  /**************************************************************************
By LucSys Robotics
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
