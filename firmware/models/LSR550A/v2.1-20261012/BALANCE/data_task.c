#include "data_task.h"

//Structure for sending data
SEND_DATA Send_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;

S21C_SensorData_t s21c_board = { 
	.rangerA = 5,
	.rangerB = 5,
	.rangerC = 5,
	.rangerD = 5,
	.rangerE = 5,
	.rangerF = 5,
};

//Ultrasound data
#define RangerFRAME_HEAD 0xFA
#define RangerFRAME_TAIL 0xFC
#define RangerFRAME_LEN 19
static uint8_t rangerbuffer[RangerFRAME_LEN];

/**************************************************************************
Function: Robot Data Transmission Task: Sending robot status, IMU, speed, and other information to various interfaces.
Input   : none
Output  : none
Function: Robot data sending tasks, sending information such as robot status, IMU, and speed to various interfaces
Entry parameters: None
Return value: none
**************************************************************************/
TaskHandle_t data_TaskHandle = NULL;

void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//This task runs at a frequency of 20Hz
			vTaskDelayUntil(&lastWakeTime, F2T(DATA_TASK_RATE));
			//Assign the data to be sent
			//Assign values to the data to be sent
			data_transition(); 
			if(A280.joint_flag[0] || A280.joint_flag[1] || A280.joint_flag[2])
			{
					CAN_UpdateAndSend();
			}
			Usart1_SendTask();
			Usart3_SendTask();
//			CAN1_SendTask();
		}
}

/**************************************************************************
Functionality: Perform BCC (Block Check Character) verification on the input array and specified length
Input Parameters: Array start address, length to be verified
Return Value: BCC verification result
Author: LUCSYS
Function: BCC checks the incoming array and the length of the checksum
Entry parameter: array head address, length to be checked
Return value: BCC checksum result
Author: LUCSYS
**************************************************************************/
uint8_t Check_BCC(const uint8_t *data, uint16_t length) {
    uint8_t bcc = 0;
    for (uint16_t i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
Function: Assign values to the data sent by the serial port
Entry parameters: None
Return value: none
**************************************************************************/
static void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header // Frame header
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail // End of frame
	
	//Obtain the robot's 3-axis calculation speed based on kinematic correct function corresponding to different vehicle models for operation
	float *vel;
	#if defined AKM_CAR || defined DIFF_CAR
		vel = Kinematics_akm_diff(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder);
	#elif defined MEC_CAR || defined _4WD_CAR
		vel = Kinematics_mec_4wd(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder,robot.MOTOR_D.Encoder);
	#elif defined OMNI_CAR
		vel = Kinematics_omni(robot.MOTOR_A.Encoder,robot.MOTOR_B.Encoder,robot.MOTOR_C.Encoder);
	#endif
	
	//Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
	//Forward kinematics: calculate current three-axis velocity from current wheel speeds
	Send_Data.Sensor_Str.Vel.X_speed = vel[0]*1000; //chassis X-axis speed, 1000x increase in transmission
	Send_Data.Sensor_Str.Vel.Y_speed = vel[1]*1000; //chassis Y-axis speed, 1000-fold increase in transmission
	Send_Data.Sensor_Str.Vel.Z_speed = vel[2]*1000; //chassis Z-axis speed, 1000x increase in transmission
	
	//The acceleration of the triaxial acceleration // Triaxial accelerometer acceleration
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ROS coordinate X-axis // Accelerometer Y-axis converted to ROS coordinate X-axis
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ROS coordinate Y-axis // Accelerometer X-axis converted to ROS coordinate Y-axis
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ROS coordinate Z-axis // Accelerometer Z-axis converted to ROS coordinate Z-axis
	
	//The angular velocity of the triaxial gyro //Triaxial gyro angular velocity
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ROS coordinate X-axis // Gyroscope Y-axis converted to ROS coordinate X-axis
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ROS coordinate Y-axis // Gyroscope X-axis converted to ROS coordinate Y-axis
	
	if( 0 == robot_control.FlagStop ) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //If motor control is enabled, normally send Z-axis angular velocity
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //If the robot is stationary (motor control disabled), then the sent Z-axis angular velocity is 0
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;  
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//Battery voltage (here, the floating-point number is amplified by 1,000 times for transmission, and correspondingly, the receiver will reduce the data by 1,000 times after receiving it)
	Send_Data.Sensor_Str.Power_Voltage = robot.voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade // Frame header
	Send_Data.buffer[1]=robot_control.FlagStop; //Chassis software-disable flag
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//The cart's three-axis speed is split into two 8-bit data points for each axle and then sent
	Send_Data.buffer[2]=Send_Data.Sensor_Str.Vel.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.Vel.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Vel.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Vel.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Vel.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Vel.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU accelerometer three-axis acceleration, each axis is split into two 8-bit data before sending
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU angular velocity meter for three axes, with each axis split into two 8-bit data points for transmission
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//Battery voltage, split into two 8-bit data transmissions
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //Data parity calculations, Mode 1 is data parity
	Send_Data.buffer[22]=Check_BCC(Send_Data.buffer,22); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail // End of frame
	
	///////////////////////Automatic recharge of related variable assignment/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //Frame header assignment 0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//Frame tail assignment 0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)charger.ChargingCurrent;//Charging current assignment
	
	Send_AutoCharge_Data.AutoCharge_Str.RED = charger.RED_STATE; //Assign values to infrared marker bits
	Send_AutoCharge_Data.AutoCharge_Str.Charging = charger.Charging; //Whether a value is assigned at the charging marker

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//Frame header 0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//High 8 bits of charging current
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//Low 8 bits of charging current
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//Whether infrared marker bits have been received
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//Whether it is at the charging marker
	Send_AutoCharge_Data.buffer[5] = charger.AllowRecharge;									//Auto-recharge status
	Send_AutoCharge_Data.buffer[6] = Check_BCC(Send_AutoCharge_Data.buffer,6);				//Check bit
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//Frame end 0x7F
	///////////////////////Automatic recharge of related variable assignment/////////////////////
	
	////////////// Ultrasonic Data Frames /////////////
	rangerbuffer[0] = RangerFRAME_HEAD;
	rangerbuffer[RangerFRAME_LEN-1] = RangerFRAME_TAIL;
	rangerbuffer[1] = (short)(s21c_board.rangerA*1000)>>8;
	rangerbuffer[2] = (short)(s21c_board.rangerA*1000);
	rangerbuffer[3] = (short)(s21c_board.rangerB*1000)>>8;
	rangerbuffer[4] = (short)(s21c_board.rangerB*1000);
	rangerbuffer[5] = (short)(s21c_board.rangerC*1000)>>8;
	rangerbuffer[6] = (short)(s21c_board.rangerC*1000);
	rangerbuffer[7] = (short)(s21c_board.rangerD*1000)>>8;
	rangerbuffer[8] = (short)(s21c_board.rangerD*1000);
	rangerbuffer[9] = (short)(s21c_board.rangerE*1000)>>8;
	rangerbuffer[10] = (short)(s21c_board.rangerE*1000);
	rangerbuffer[11] = (short)(s21c_board.rangerF*1000)>>8;
	rangerbuffer[12] = (short)(s21c_board.rangerF*1000);
	rangerbuffer[13] = 0;
	rangerbuffer[14] = 0;
	rangerbuffer[15] = 0;
	rangerbuffer[16] = 0;
	rangerbuffer[17] = Check_BCC(rangerbuffer,17);
	////////////// Ultrasonic Data Frames /////////////
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
Function: Send data via Serial Port 1
Entry parameters: None
Return value: none
**************************************************************************/
static void Usart1_SendTask(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		uart1_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart1_send(rangerbuffer[i]);
		}	
	}
	
	if(SysVal.HardWare_charger==1 || SysVal.HardWare_Ranger==1)
	{
		//When recharge equipment exists, send auto-recharge variables to the upper layer
		for(i=0; i<8; i++)
		{
			uart1_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}

/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
Function: Send data via Serial Port 3
Entry parameters: None
Return value: none
**************************************************************************/
static void Usart3_SendTask(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		uart3_send(Send_Data.buffer[i]);
	}	 
	
	if( SysVal.HardWare_Ranger==1 )
	{
		for(i=0; i<RangerFRAME_LEN; i++)
		{
			uart3_send(rangerbuffer[i]);
		}	
	}
	
	if(SysVal.HardWare_charger==1 || SysVal.HardWare_Ranger==1)
	{
		//When recharge equipment exists, send auto-recharge variables to the upper layer
		for(i=0; i<8; i++)
		{
			uart3_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}

/**************************************************************************
Function: CAN1 sends data
Input   : none
Output  : none
Function: CAN1 sends data
Entry parameters: None
Return value: none
**************************************************************************/
static void CAN1_SendTask(void)
{
	u8 CAN_SENT[8],i;
	
	//24-byte data is sent in three groups, 0x101 0x102 0x103 using standard frame IDs
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//When recharge equipment exists, send recharge-related configuration data to the recharge equipment
	if(SysVal.HardWare_charger==1)
	{
		CAN_Send_AutoRecharge();
	}
}

/*-------------------- Positive kinematics correlation function for each vehicle model ------------------------*/
/*-------------------------------- Forward Kinematics Functions for Various Vehicle Models ------------------------------------*/
/**************************************************************************
Function Purpose: akm/diff Kinematic Analysis
Input Parameters: Left wheel speed, Right wheel speed, in meters per second (m/s)
Return Value: Robot's x, y, z velocities
Author: LUCSYS
Function: Ackermann/differential Kinematics correct answer
Entry parameters: left wheel speed, right wheel speed, unit m/s
Return value: robot x/y/z-axis velocities
Author: LUCSYS
**************************************************************************/
#if defined AKM_CAR || defined DIFF_CAR
static float* Kinematics_akm_diff(float motorA,float motorB)
{
	static float vel[3];
	//XYZ three-axis calculation speed
	vel[0] = (motorA + motorB)/2.0f;
	vel[1] = 0;
	vel[2] = (motorB - motorA)/robot.HardwareParam.WheelSpacing;
	
	return vel;
}

/**************************************************************************
Function Purpose: mec/4wd Kinematic Analysis
Input Parameters: Robot's four-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: LUCSYS
Function: Wheat wheel/four-wheel drive Kinematics correct answer
Entry parameters: speed of the robot's four wheels, unit m/s
Return value: robot x/y/z-axis velocities
Author: LUCSYS
**************************************************************************/
#elif defined MEC_CAR || defined _4WD_CAR
static float* Kinematics_mec_4wd(float motorA,float motorB,float motorC,float motorD)
{
	static float vel[3];
	//XYZ three-axis calculation speed
	vel[0] = (motorA+motorB+motorC+motorD)/4.0f;
	vel[1] = (motorA-motorB+motorC-motorD)/4.0f;
	vel[2] = (-motorA-motorB+motorC+motorD)/4.0f/( robot.HardwareParam.WheelSpacing + robot.HardwareParam.AxleSpacing );
	
	return vel;
}

/**************************************************************************
Function Purpose: omni Kinematic Analysis
Input Parameters: Robot's three-wheel speeds, in meters per second (m/s).
Return Value: Robot's x, y, z velocities
Author: LUCSYS
Function: Omnidirectional wheel forward kinematics
Entry parameters: speed of the robot's three wheels, unit: m/s
Return value: robot x/y/z-axis velocities
Author: LUCSYS
**************************************************************************/
#elif defined OMNI_CAR
static float* Kinematics_omni(float motorA,float motorB,float motorC)
{
	static float vel[3];
	//XYZ three-axis calculation speed
	vel[0] = (motorC - motorB)/2.0f/robot.HardwareParam.X_PARAMETER;
	vel[1] = (motorA*2 - motorB - motorC)/3.0f;
	vel[2] = (motorA + motorB + motorC )/ 3.0f /robot.HardwareParam.TurnRadiaus;
	
	return vel;
}

#endif


