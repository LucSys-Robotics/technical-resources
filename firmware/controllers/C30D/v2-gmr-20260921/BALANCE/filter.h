#ifndef __FILTER_H
#define __FILTER_H
#include "system.h"
  /**************************************************************************
Distribution maintenance: LucSys Robotics (authorized rebranding)
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
