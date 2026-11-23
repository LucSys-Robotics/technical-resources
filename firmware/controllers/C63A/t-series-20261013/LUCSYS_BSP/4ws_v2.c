#include "4ws.h"
#include <math.h>

#define PI 3.14159265358979323846f
#define VEL_EPS 0.0001f //速度零判定阈值,避免浮点精确比较


void FourWSCar_param_init(RobotParam_t *car,float m_W,float m_L,float WheelTurnR,float wp)
{
	car->hardware.WheelSpacing = m_W;
	car->hardware.AxleSpacing = m_L;
	car->hardware.WheelTurnR = WheelTurnR;
	car->hardware.WheelPerimeter = wp;
	
	//目标/实际角度初始化为直行角度(AB:+90度,CD:-90度),与上电回零位置一致
	car->TargetMotorA.radian = PI/2.0f;
	car->TargetMotorB.radian = PI/2.0f;
	car->TargetMotorC.radian = -PI/2.0f;
	car->TargetMotorD.radian = -PI/2.0f;
	car->RealMotorA.radian = PI/2.0f;
	car->RealMotorB.radian = PI/2.0f;
	car->RealMotorC.radian = -PI/2.0f;
	car->RealMotorD.radian = -PI/2.0f;

	//逆解分支记忆清零,起始一律按"不翻转"起步
	for(uint8_t i=0;i<4;i++) car->pathFlip[i] = 0;
}



//物理限位版本
// atan2取值范围：(-180°, 180°]
// 为限制轮子的运动范围,对4个轮子的角度作以下限制
// a轮限制： (-90,270)
// b轮限制： (-90,270)
// c轮限制： (-270,90)
// d轮限制： (-270,90)
#define RAD_90  (1.57079632679489661923f)
#define RAD_50  (0.87266462599716478846f)
#define RAD_180 PI
#define RAD_220 (3.83972435438752506923f)    
#define RAD_270 (4.71238898038468985769f)
//路径判据的迟滞余量.仅在"两个候选都在机械范围内"时生效,不影响限位判定.
//取值依据见对拍工程 T6:0度时28.8%的工况在分界附近来回翻,加到35度降到个位数.
//代价是两条路都合法时可能多绕最多35度,但一定还在机械范围内.
#define RAD_HYST  (0.61086523819801535192f)

//角度归一化到各自的工作窗口:AB轮 (-90,270],CD轮 (-270,90]
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
//差值归一化到 (-180,180],用于比较哪条路更近
/* Bounded absolute-position steering cannot wrap through its end stops. */

/**
 * AB轮(左侧)转向目标选路
 *
 * 逆解出的角度只有两个可用形态,相差180度:直接到位,或掉头180度并把轮速反向。
 * 两者的机械合法性先判,再谈哪条更近 —— 顺序不能反,限位优先于路径优化。
 *
 * 关于"永远不会超出 [-50,220]":窗口(-90,270]内的非法区只有 (-90,-50) 和 (220,270],
 * 宽 40度 与 50度。c0落在前者时 c1=c0+180 落在(90,130),落在后者时 c1=c0-180 落在(40,90],
 * 两者都在合法带内。也就是说只要有一个非法,另一个必然合法,不存在无解。
 *
 * flipped 记录上一次实际选中的是哪个候选(1=选了掉头那条),用于加迟滞。
 * 注意它记的是"实际输出",不是中间判据 —— 否则限位纠正过的结果会与记忆脱节。
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
		//两条都能走,才比较路径长短.迟滞:维持上次的选择,除非另一条明显更近
		float d0 = fabsf(c0 - now_rad);
		float d1 = fabsf(c1 - now_rad);
		use1 = ( d1 < d0 + ((*flipped) ? RAD_HYST : -RAD_HYST) ) ? 1u : 0u;
	}
	else
	{
		//只有一条在机械范围内,只能走它,不参与路径比较也不加迟滞
		use1 = ok1;
	}

	if( use1 ){ wheel->radian = c1; wheel->speed *= -1.0f; }
	else      { wheel->radian = c0; }

	*flipped = use1;
}

/**
 * CD轮(右侧)转向目标选路:与 PathChange_AB 同理,窗口与合法带镜像
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
		//零速:保持上次下发的目标角度不变,只把轮速清零
		car->TargetMotorA.speed = 0;
		car->TargetMotorB.speed = 0;
		car->TargetMotorC.speed = 0;
		car->TargetMotorD.speed = 0;
		return;
	}
	
	//将二维笛卡儿坐标系坐标转换为ROS坐标
	float tmpVx=0,tmpVy=0;
	tmpVx = Vx;
	tmpVy = Vy;
	Vx = -tmpVy;
	Vy = tmpVx;
	
    // 车轮i的x和y轴速度
    float Vxi=0, Vyi=0;
	
	//车轮初始坐标
	float Car_Track = car->hardware.WheelSpacing;  //轮距(左右两轮中心距),对应轮子坐标的左右分量
	float Car_Base  = car->hardware.AxleSpacing;   //轴距(前后两轮中心距),对应轮子坐标的前后分量
	float WheelPoint[4][2] = {
		{-Car_Track / 2.0f, -Car_Base / 2.0f},  //A轮:左后
		{-Car_Track / 2.0f, Car_Base / 2.0f},   //B轮:左前
		{Car_Track / 2.0f, Car_Base / 2.0f},    //C轮:右前
		{Car_Track / 2.0f, -Car_Base / 2.0f}    //D轮:右后
	};
	
	//定义4个轮子临时数据
	WheelInfo_t car_wheel[4];
	
    for (uint8_t i = 0; i < 4; i++)
    {
        Vxi = Vx - WheelPoint[i][1] * Vz;
        Vyi = Vy + WheelPoint[i][0] * Vz;

        // 得出车轮速度
        car_wheel[i].speed = sqrtf(Vxi * Vxi + Vyi * Vyi);

        // 得出车轮角度
        car_wheel[i].radian = atan2f(Vyi, Vxi);
    }
	
    // atan2取值范围：(-180°, 180°]
	// 为限制轮子的运动范围,对4个轮子的角度作以下限制
    // a轮限制： (-90,270)
    // b轮限制： (-90,270)
    // c轮限制： (-270,90)
    // d轮限制： (-270,90)
	PathChange_AB(&car_wheel[0],car->RealMotorA.radian,&car->pathFlip[0]);
	PathChange_AB(&car_wheel[1],car->RealMotorB.radian,&car->pathFlip[1]);
	PathChange_CD(&car_wheel[2],car->RealMotorC.radian,&car->pathFlip[2]);
	PathChange_CD(&car_wheel[3],car->RealMotorD.radian,&car->pathFlip[3]);
	
	//若车轮存在转向半径,则进行一次迭代计算
	if( fabsf(car->hardware.WheelTurnR - 0) > 0.001f )
	{
		//根据初步取得的车轮角度,计算轮子的坐标值
		float WheelPoint_Diff[4][2] = {
			{-Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[0].radian), -Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[0].radian)},
			{-Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[1].radian), Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[1].radian)},
			{Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[2].radian), Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[2].radian)},
			{Car_Track/2.0f-car->hardware.WheelTurnR*sin(car_wheel[3].radian), -Car_Base/2.0f+car->hardware.WheelTurnR*cos(car_wheel[3].radian)}
		};
		
		//使用新轮子坐标,重新计算轮子真实角度(相当于一次迭代)
		//实测角度基本不变,迭代1次足够
		for (uint8_t i = 0; i < 4; i++)
		{
			Vxi = Vx - WheelPoint_Diff[i][1] * Vz;
			Vyi = Vy + WheelPoint_Diff[i][0] * Vz;

			// 得出车轮速度
			car_wheel[i].speed = sqrtf(Vxi * Vxi + Vyi * Vyi);

			// 得出车轮角度
			car_wheel[i].radian = atan2f(Vyi, Vxi);
		}
		
		//对新的轮速与角度再作规范限制
		PathChange_AB(&car_wheel[0],car->RealMotorA.radian,&car->pathFlip[0]);
		PathChange_AB(&car_wheel[1],car->RealMotorB.radian,&car->pathFlip[1]);
		PathChange_CD(&car_wheel[2],car->RealMotorC.radian,&car->pathFlip[2]);
		PathChange_CD(&car_wheel[3],car->RealMotorD.radian,&car->pathFlip[3]);
	}
	
	//输出目标轮速和角度,单位为 m/s 与 弧度rad
	car->TargetMotorA.speed = car_wheel[0].speed;
	car->TargetMotorA.radian = car_wheel[0].radian;
	car->TargetMotorB.speed = car_wheel[1].speed;
	car->TargetMotorB.radian = car_wheel[1].radian;
	car->TargetMotorC.speed = car_wheel[2].speed;
	car->TargetMotorC.radian = car_wheel[2].radian;
	car->TargetMotorD.speed = car_wheel[3].speed;
	car->TargetMotorD.radian = car_wheel[3].radian;
	
	
}

//根据轮子的角度和速度计算三轴速度
void four_ws_kinematics(RobotParam_t* car)
{
	float Car_Track = car->hardware.WheelSpacing;  //轮距(左右两轮中心距),对应轮子坐标的左右分量
	float Car_Base  = car->hardware.AxleSpacing;   //轴距(前后两轮中心距),对应轮子坐标的前后分量
	
	float WheelPoint_Diff[4][2] = { 0 };
	
	//轮子存在运动半径时需要考虑运动半径影响
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
	
	//计算平均值
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

	//计算平均值,将笛卡儿坐标系转换为ROS坐标
	car->feedback_Vx = Vy/4.0f;
	car->feedback_Vy = -Vx/4.0f;
}

//按已解算的目标转向角,将三轴速度对应的各轮速度矢量投影到目标角方向,得到带符号轮速
//保证轮速符号与已下发的转向角一致,避免二次逆解时翻转决策不一致的问题
void four_ws_WheelSpeedProjection(float Vx,float Vy,float Vz,RobotParam_t *car)
{
	//二维笛卡尔坐标系速度转换为ROS坐标(与逆解一致)
	float tmpVx = Vx;
	float tmpVy = Vy;
	Vx = -tmpVy;
	Vy = tmpVx;
	
	float Car_Track = car->hardware.WheelSpacing;  //轮距(左右两轮中心距),对应轮子坐标的左右分量
	float Car_Base  = car->hardware.AxleSpacing;   //轴距(前后两轮中心距),对应轮子坐标的前后分量
	float TurnR = car->hardware.WheelTurnR;
	
	WheelInfo_t* wheel[4] = { &car->TargetMotorA,&car->TargetMotorB,&car->TargetMotorC,&car->TargetMotorD };
	float WheelPoint[4][2] = {
		{-Car_Track / 2.0f, -Car_Base / 2.0f},  //A轮:左后
		{-Car_Track / 2.0f, Car_Base / 2.0f},   //B轮:左前
		{Car_Track / 2.0f, Car_Base / 2.0f},    //C轮:右前
		{Car_Track / 2.0f, -Car_Base / 2.0f}    //D轮:右后
	};
	
	for (uint8_t i = 0; i < 4; i++)
	{
		float px = WheelPoint[i][0];
		float py = WheelPoint[i][1];
		
		//存在转向偏心半径时修正轮子坐标
		if( fabsf(TurnR - 0) > 0.001f )
		{
			px -= TurnR*sinf(wheel[i]->radian);
			py += TurnR*cosf(wheel[i]->radian);
		}
		
		float Vxi = Vx - py * Vz;
		float Vyi = Vy + px * Vz;
		
		//沿目标转向角方向投影,得到带符号轮速
		wheel[i]->speed = Vxi*cosf(wheel[i]->radian) + Vyi*sinf(wheel[i]->radian);
	}
}

