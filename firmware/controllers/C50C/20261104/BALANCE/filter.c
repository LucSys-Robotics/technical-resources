#include "filter.h"
  /**************************************************************************
Distribution maintenance: LucSys Robotics 
**************************************************************************/
float K1 =0.02; 
float angle, angle_dot; 	
float Q_angle=0.001;// Process noise covariance
float Q_gyro=0.003;//0.003 Covariance of Process Noise The covariance of process noise is a matrix of one row and two columns
float R_angle=0.5;// Measuring the covariance of noise, i.e., measuring deviation
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**************************************************************************
Function: Simple Kalman filtering
Inlet parameters: acceleration, angular velocity
Return value: none
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //A priori estimation
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-differentiation of the covariance of the prior estimation error

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk - integral of the covariance derivative of the prior estimation error
	PP[0][1] += Pdot[1] * dt;   // = covariance of the prior estimation error
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-prior estimate
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //Posterior estimation error, covariance
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //A posterior estimation
	Q_bias	+= K_1 * Angle_err;	 //A posterior estimation
	angle_dot   = Gyro - Q_bias;	 //Derivative of the output (posterior estimate) = angular velocity
}

/**************************************************************************
Function: First-order complementary filtering
Inlet parameters: acceleration, angular velocity
Return value: none
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005);
}
