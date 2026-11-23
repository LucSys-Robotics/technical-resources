#include "4ws.h"
#include <math.h>

#define PI 3.14159265358979323846f
#define VEL_EPS 0.0001f //Zero-velocity threshold to avoid exact floating-point comparisons


void FourWSCar_param_init(RobotParam_t *car,float m_W,float m_L,float WheelTurnR,float wp)
{
	car->hardware.WheelSpacing = m_W;
	car->hardware.AxleSpacing = m_L;
	car->hardware.WheelTurnR = WheelTurnR;
	car->hardware.WheelPerimeter = wp;
	
	//The target/actual angle is initialized to a straight angle (AB: +90 degrees, CD: -90 degrees), matching the power-on zero position
	car->TargetMotorA.radian = PI/2.0f;
	car->TargetMotorB.radian = PI/2.0f;
	car->TargetMotorC.radian = -PI/2.0f;
	car->TargetMotorD.radian = -PI/2.0f;
	car->RealMotorA.radian = PI/2.0f;
	car->RealMotorB.radian = PI/2.0f;
	car->RealMotorC.radian = -PI/2.0f;
	car->RealMotorD.radian = -PI/2.0f;

	//Clear inverse-kinematics branch memory; start all wheels without flipping
	for(uint8_t i=0;i<4;i++) car->pathFlip[i] = 0;
}



//Physically limited version
// atan2 value range: (-180бу, 180бу]
// To limit the movement range of the wheels, the following restrictions are applied to the angles of the four wheels
// Series A Limit: (-90,270)
// Series B limit: (-90,270)
// Wheel C limits: (-270,90)
// D-axis limit: (-270, 90)
#define RAD_90  (1.57079632679489661923f)
#define RAD_50  (0.87266462599716478846f)
#define RAD_180 PI
#define RAD_220 (3.83972435438752506923f)    
#define RAD_270 (4.71238898038468985769f)
//Hysteresis margin of the path criterion. It only applies when "both candidates are within mechanical range" and does not affect the limit determination.
//See test harness T6 for the threshold basis: at 0 degrees, 28.8% of cases flip around the boundary; at 35 degrees this falls to a single-digit percentage.
//The trade-off is that when both roads are legal, you may detour up to 35 degrees more, but it will still be within mechanical limits.
#define RAD_HYST  (0.61086523819801535192f)

//Normalize angles to each working window: wheels A/B (-90,270], wheels C/D (-270,90]
static float ab_wrap(float a)
{
	while( a <= -RAD_90 ) a += RAD_180*2.0f;
	while( a >  RAD_270 ) a -= RAD_180*2.0f;
	return a;
}
static float cd_wrap(float a)
{
	while( a >   RAD_90 ) a -= RAD_180*2.0f;
	while( a <= -RAD_270) a += RAD_180*2.0f;
	return a;
}
//Difference normalized to (-180,180], used to compare which path is closer
/* Bounded absolute-position steering cannot wrap through its end stops. */

/**
 * AB wheel (left side) turns to target and selects the route
 *
 * Inverse kinematics provides two equivalent angle choices separated by 180 degrees: direct alignment, or turn 180 degrees and reverse wheel speed.
 * Judge the mechanical legitimacy of both first, then discuss which is closer бк order cannot be reversed, limits take precedence over path optimization.
 *
 * Regarding "never exceeding [-50,220]": the illegal zone within window (-90,270) only includes (-90,-50) and (220,270],
 * Widths of 40 degrees and 50 degrees. When c0 falls on the former, c1 = c0 + 180 falls at (90, 130); when c0 falls on the latter, c1 = c0 - 180 falls at (40, 90],
 * Both are within the legal zone. In other words, as long as one is illegal, the other must be legal; there is no unsolvable solution.
 *
 * flipped records which candidate was actually selected the last time (1 = chose the U-turn one), used for adding hysteresis.
 * Note that it records the "actual output," not an intermediate criterion бк otherwise, the results of limit correction will be disconnected from memory.
 */
void PathChange_AB(WheelInfo_t* wheel,float now_rad,uint8_t* flipped)
{
	float c0 = ab_wrap(wheel->radian);
	float c1 = ab_wrap(c0 + RAD_180);

	uint8_t ok0 = ( c0 >= -RAD_50 && c0 <= RAD_220 ) ? 1u : 0u;
	uint8_t ok1 = ( c1 >= -RAD_50 && c1 <= RAD_220 ) ? 1u : 0u;
	uint8_t use1;

	if( ok0 && ok1 )
	{
		//Both paths can be used to compare path length. Lag: Maintain the previous choice unless the other is noticeably closer
		float d0 = fabsf(c0 - now_rad);
		float d1 = fabsf(c1 - now_rad);
		use1 = ( d1 < d0 + ((*flipped) ? RAD_HYST : -RAD_HYST) ) ? 1u : 0u;
	}
	else
	{
		//Only one path within the mechanical range can only be taken, without participating in path comparison or delay
		use1 = ok1;
	}

	if( use1 ){ wheel->radian = c1; wheel->speed *= -1.0f; }
	else      { wheel->radian = c0; }

	*flipped = use1;
}

/**
 * CD wheel (right) turns to target for path selection: same as PathChange_AB, window and legal mirror with mirror
 */
void PathChange_CD(WheelInfo_t* wheel,float now_rad,uint8_t* flipped)
{
	float c0 = cd_wrap(wheel->radian);
	float c1 = cd_wrap(c0 + RAD_180);

	uint8_t ok0 = ( c0 >= -RAD_220 && c0 <= RAD_50 ) ? 1u : 0u;
	uint8_t ok1 = ( c1 >= -RAD_220 && c1 <= RAD_50 ) ? 1u : 0u;
	uint8_t use1;

	if( ok0 && ok1 )
	{
		float d0 = fabsf(c0 - now_rad);
		float d1 = fabsf(c1 - now_rad);
		use1 = ( d1 < d0 + ((*flipped) ? RAD_HYST : -RAD_HYST) ) ? 1u : 0u;
	}
	else
	{
		use1 = ok1;
	}

	if( use1 ){ wheel->radian = c1; wheel->speed *= -1.0f; }
	else      { wheel->radian = c0; }

	*flipped = use1;
}

void four_ws_InverseKinematics(float Vx,float Vy,float Vz,RobotParam_t *car)
{
	if( fabsf(Vx)<VEL_EPS && fabsf(Vy)<VEL_EPS && fabsf(Vz)<VEL_EPS )
	{
		//Zero speed: retain the last transmitted target angles and set only wheel speeds to zero
		car->TargetMotorA.speed = 0;
		car->TargetMotorB.speed = 0;
		car->TargetMotorC.speed = 0;
		car->TargetMotorD.speed = 0;
		return;
	}
	
	//Convert 2D Cartesian coordinate system coordinates to ROS coordinates
	float tmpVx=0,tmpVy=0;
	tmpVx = Vx;
	tmpVy = Vy;
	Vx = -tmpVy;
	Vy = tmpVx;
	
    // The speed of the x and y axes of wheel i
    float Vxi=0, Vyi=0;
	
	//Initial coordinates of the wheel
	float Car_Track = car->hardware.WheelSpacing;  //Track width (distance between left and right wheel centers), corresponding to the lateral wheel-coordinate component
	float Car_Base  = car->hardware.AxleSpacing;   //Wheelbase (distance between front and rear wheel centers), corresponding to the longitudinal wheel-coordinate component
	float WheelPoint[4][2] = {
		{-Car_Track / 2.0f, -Car_Base / 2.0f},  //A round: left rear
		{-Car_Track / 2.0f, Car_Base / 2.0f},   //B Round: Front left
		{Car_Track / 2.0f, Car_Base / 2.0f},    //C Wheel: Front right
		{Car_Track / 2.0f, -Car_Base / 2.0f}    //D Wheel: Right rear
	};
	
	//Define temporary data for 4 wheels
	WheelInfo_t car_wheel[4];
	
    for (uint8_t i = 0; i < 4; i++)
    {
        Vxi = Vx - WheelPoint[i][1] * Vz;
        Vyi = Vy + WheelPoint[i][0] * Vz;

        // Obtain wheel speed
        car_wheel[i].speed = sqrtf(Vxi * Vxi + Vyi * Vyi);

        // Obtain wheel angle
        car_wheel[i].radian = atan2f(Vyi, Vxi);
    }
	
    // atan2 value range: (-180бу, 180бу]
	// To limit the movement range of the wheels, the following restrictions are applied to the angles of the four wheels
    // Series A Limit: (-90,270)
    // Series B limit: (-90,270)
    // Wheel C limits: (-270,90)
    // D-axis limit: (-270, 90)
	PathChange_AB(&car_wheel[0],car->RealMotorA.radian,&car->pathFlip[0]);
	PathChange_AB(&car_wheel[1],car->RealMotorB.radian,&car->pathFlip[1]);
	PathChange_CD(&car_wheel[2],car->RealMotorC.radian,&car->pathFlip[2]);
	PathChange_CD(&car_wheel[3],car->RealMotorD.radian,&car->pathFlip[3]);
	
	//If the wheel has a turning radius, perform one iterative calculation
	if( fabsf(car->hardware.WheelTurnR - 0) > 0.001f )
	{
		//Based on the preliminary wheel angle, calculate the wheel's coordinate values
		float WheelPoint_Diff[4][2] = {
			{-Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[0].radian), -Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[0].radian)},
			{-Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[1].radian), Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[1].radian)},
			{Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[2].radian), Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[2].radian)},
			{Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[3].radian), -Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[3].radian)}
		};
		
		//Using new wheel coordinates, recalculate the true wheel angle (equivalent to one iteration)
		//The measured angle remains basically unchanged, and one iteration is sufficient
		for (uint8_t i = 0; i < 4; i++)
		{
			Vxi = Vx - WheelPoint_Diff[i][1] * Vz;
			Vyi = Vy + WheelPoint_Diff[i][0] * Vz;

			// Obtain wheel speed
			car_wheel[i].speed = sqrtf(Vxi * Vxi + Vyi * Vyi);

			// Obtain wheel angle
			car_wheel[i].radian = atan2f(Vyi, Vxi);
		}
		
		//Further restrict the speed and angle of new wheels
		PathChange_AB(&car_wheel[0],car->RealMotorA.radian,&car->pathFlip[0]);
		PathChange_AB(&car_wheel[1],car->RealMotorB.radian,&car->pathFlip[1]);
		PathChange_CD(&car_wheel[2],car->RealMotorC.radian,&car->pathFlip[2]);
		PathChange_CD(&car_wheel[3],car->RealMotorD.radian,&car->pathFlip[3]);
	}
	
	//Output target wheel speeds and angles, in m/s and radians (rad)
	car->TargetMotorA.speed = car_wheel[0].speed;
	car->TargetMotorA.radian = car_wheel[0].radian;
	car->TargetMotorB.speed = car_wheel[1].speed;
	car->TargetMotorB.radian = car_wheel[1].radian;
	car->TargetMotorC.speed = car_wheel[2].speed;
	car->TargetMotorC.radian = car_wheel[2].radian;
	car->TargetMotorD.speed = car_wheel[3].speed;
	car->TargetMotorD.radian = car_wheel[3].radian;
	
	
}

//Calculate the three-axis speed based on the wheel's angle and speed
void four_ws_kinematics(RobotParam_t* car)
{
	float Car_Track = car->hardware.WheelSpacing;  //Track width (distance between left and right wheel centers), corresponding to the lateral wheel-coordinate component
	float Car_Base  = car->hardware.AxleSpacing;   //Wheelbase (distance between front and rear wheel centers), corresponding to the longitudinal wheel-coordinate component
	
	float WheelPoint_Diff[4][2] = { 0 };
	
	//Account for the wheel's turning radius when nonzero
	if( fabsf(car->hardware.WheelTurnR - 0) > 0.001f )
	{
		WheelPoint_Diff[0][0]=-Car_Track/2.0f-car->hardware.WheelTurnR*sin(car->RealMotorA.radian);
		WheelPoint_Diff[0][1]=-Car_Base/2.0f+car->hardware.WheelTurnR*cos(car->RealMotorA.radian);
		WheelPoint_Diff[1][0]=-Car_Track/2.0f-car->hardware.WheelTurnR*sin(car->RealMotorB.radian);
		WheelPoint_Diff[1][1]= Car_Base/2.0f+car->hardware.WheelTurnR*cos(car->RealMotorB.radian);
		WheelPoint_Diff[2][0]= Car_Track/2.0f-car->hardware.WheelTurnR*sin(car->RealMotorC.radian);
		WheelPoint_Diff[2][1]= Car_Base/2.0f+car->hardware.WheelTurnR*cos(car->RealMotorC.radian);
		WheelPoint_Diff[3][0]= Car_Track/2.0f-car->hardware.WheelTurnR*sin(car->RealMotorD.radian);
		WheelPoint_Diff[3][1]=-Car_Base/2.0f+car->hardware.WheelTurnR*cos(car->RealMotorD.radian);
	}
	else
	{
		WheelPoint_Diff[0][0]=-Car_Track / 2.0f;
		WheelPoint_Diff[0][1]=-Car_Base / 2.0f;
		WheelPoint_Diff[1][0]=-Car_Track / 2.0f;
		WheelPoint_Diff[1][1]= Car_Base / 2.0f;
		WheelPoint_Diff[2][0]= Car_Track / 2.0f;
		WheelPoint_Diff[2][1]= Car_Base / 2.0f;
		WheelPoint_Diff[3][0]= Car_Track / 2.0f;
		WheelPoint_Diff[3][1]=-Car_Base / 2.0f;
	}
	
    float Vax = car->RealMotorA.speed * cos(car->RealMotorA.radian);
    float Vay = car->RealMotorA.speed * sin(car->RealMotorA.radian);
    float Vcx = car->RealMotorC.speed * cos(car->RealMotorC.radian);
    float Vcy = car->RealMotorC.speed * sin(car->RealMotorC.radian);
    float Vbx = car->RealMotorB.speed * cos(car->RealMotorB.radian);
    float Vby = car->RealMotorB.speed * sin(car->RealMotorB.radian);
    float Vdx = car->RealMotorD.speed * cos(car->RealMotorD.radian);
    float Vdy = car->RealMotorD.speed * sin(car->RealMotorD.radian);
    float yca = WheelPoint_Diff[2][1] - WheelPoint_Diff[0][1];
    float xac = WheelPoint_Diff[0][0] - WheelPoint_Diff[2][0];
    float ydb = WheelPoint_Diff[3][1] - WheelPoint_Diff[1][1];
    float xbc = WheelPoint_Diff[1][0] - WheelPoint_Diff[3][0];
    float Vz1 = (Vax-Vcx)/yca;
    float Vz2 = (Vay-Vcy)/xac;
    float Vz3 = (Vbx-Vdx)/ydb;
    float Vz4 = (Vby-Vdy)/xbc;
	
	//Calculate the average
    car->feedback_Vz = (Vz1+Vz2+Vz3+Vz4)/4.0f;
	
	float Vx=0,Vy=0;
	Vx += car->RealMotorA.speed*cos(car->RealMotorA.radian)+car->feedback_Vz*WheelPoint_Diff[0][1];
	Vx += car->RealMotorB.speed*cos(car->RealMotorB.radian)+car->feedback_Vz*WheelPoint_Diff[1][1];
	Vx += car->RealMotorC.speed*cos(car->RealMotorC.radian)+car->feedback_Vz*WheelPoint_Diff[2][1];
	Vx += car->RealMotorD.speed*cos(car->RealMotorD.radian)+car->feedback_Vz*WheelPoint_Diff[3][1];
	
	Vy += car->RealMotorA.speed*sin(car->RealMotorA.radian)-car->feedback_Vz*WheelPoint_Diff[0][0];
	Vy += car->RealMotorB.speed*sin(car->RealMotorB.radian)-car->feedback_Vz*WheelPoint_Diff[1][0];
	Vy += car->RealMotorC.speed*sin(car->RealMotorC.radian)-car->feedback_Vz*WheelPoint_Diff[2][0];
	Vy += car->RealMotorD.speed*sin(car->RealMotorD.radian)-car->feedback_Vz*WheelPoint_Diff[3][0];
	
	// car->feedback_Vx = Vx/4.0f;
	// car->feedback_Vy = Vy/4.0f;

	//Calculate the mean and convert the Cartesian coordinate system to ROS coordinates
	car->feedback_Vx = Vy/4.0f;
	car->feedback_Vy = -Vx/4.0f;
}

//Based on the calculated target steering angle, project the speed vectors corresponding to the three-axis speed of each wheel onto the target angle direction to obtain the signed wheel speed
//Ensure that the wheel speed symbol matches the given steering angle, preventing inconsistent flip decisions during the second inverse solution
void four_ws_WheelSpeedProjection(float Vx,float Vy,float Vz,RobotParam_t *car)
{
	//Convert 2D Cartesian coordinate velocity to ROS coordinates (consistent with inverse solution)
	float tmpVx = Vx;
	float tmpVy = Vy;
	Vx = -tmpVy;
	Vy = tmpVx;
	
	float Car_Track = car->hardware.WheelSpacing;  //Track width (distance between left and right wheel centers), corresponding to the lateral wheel-coordinate component
	float Car_Base  = car->hardware.AxleSpacing;   //Wheelbase (distance between front and rear wheel centers), corresponding to the longitudinal wheel-coordinate component
	float TurnR = car->hardware.WheelTurnR;
	
	WheelInfo_t* wheel[4] = { &car->TargetMotorA,&car->TargetMotorB,&car->TargetMotorC,&car->TargetMotorD };
	float WheelPoint[4][2] = {
		{-Car_Track / 2.0f, -Car_Base / 2.0f},  //A round: left rear
		{-Car_Track / 2.0f, Car_Base / 2.0f},   //B Round: Front left
		{Car_Track / 2.0f, Car_Base / 2.0f},    //C Wheel: Front right
		{Car_Track / 2.0f, -Car_Base / 2.0f}    //D Wheel: Right rear
	};
	
	for (uint8_t i = 0; i < 4; i++)
	{
		float px = WheelPoint[i][0];
		float py = WheelPoint[i][1];
		
		//Correct wheel coordinates when a steering eccentric radius exists
		if( fabsf(TurnR - 0) > 0.001f )
		{
			px -= TurnR*sinf(wheel[i]->radian);
			py += TurnR*cosf(wheel[i]->radian);
		}
		
		float Vxi = Vx - py * Vz;
		float Vyi = Vy + px * Vz;
		
		//Projecting along the target turning angle direction yields the signed wheel speed
		wheel[i]->speed = Vxi*cosf(wheel[i]->radian) + Vyi*sinf(wheel[i]->radian);
	}
}

