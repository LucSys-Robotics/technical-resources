#include "balance.h"

//Whether the robot model is incorrectly marked
//Check if the robot model is incorrect in the flag position
int robot_mode_check_flag=0;
int A=1,B=1,C=1,DD=1; //Used for testing // Used for testing
int Time_count=0; //Time variable // Time variable

u8 command_lost_count=0;//Serial port and CAN control command loss time count; control stops after one second of loss

//========== PWM clears used variables ==========//
u8 start_check_flag = 0;//Mark whether PWM needs to be cleared
u8 wait_clear_times = 0;
u8 start_clear = 0;     //Marks start clearing PWM
u8 clear_done_once = 0; //Clear the completion flag
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

//Self-test parameters
int check_a,check_b,check_c,check_d;
u8 check_end=0;

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
Function: Kinematic inverse solution, calculates the target rotational speed of each wheel based on the target speed of the three axes
Entry parameters: target motion speed along the X, Y, and Z axes
Return value: none
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
 	static float amplitude=3.5; //Wheel target speed limit // Wheel target speed limit
	
	Vx=target_limit_float(Vx,-amplitude,amplitude);
	Vy=target_limit_float(Vy,-amplitude,amplitude);
	Vz=target_limit_float(Vz,-amplitude,amplitude);

	smooth_control.VX = Vx,
	smooth_control.VY = Vy,
	smooth_control.VZ = Vz;

	//Get the smoothed data 
	//Retrieve the smoothed data
	Vx=smooth_control.VX;
	Vy=smooth_control.VY;
	Vz=smooth_control.VZ;
	
	  #if _4WD //4WD wheel car // Four-wheel drive car
    
	  //Inverse kinematics //Inverse kinematics
   	MOTOR_A.Target = +0+Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_B.Target = -0+Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_C.Target = +0+Vx+Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_D.Target = -0+Vx+Vz*(Wheel_axlespacing+Wheel_spacing);
	
	  //Wheel (motor) target speed limit // Wheel (motor) target speed limit
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude);
		#endif
}

void Get_RC_ARM(void)
{
    u8 Flag_Move = 1;

    if (A280.control_mode == APP_Mode)
    {
        float step = 0.002f;
        float step_multi = 5.0f;


        //=====================================================================
        // Inverse-kinematics mode
        //=====================================================================
        if (A280.kinematic_mode == Inverse_Kinematic)
        {
            float new_x = A280.ee_x;
            float new_y = A280.ee_y;
            // Calculate the new position based on the orientation
            switch (Flag_Direction)
            {
                case 1:  new_y += step; Flag_Move = 1; break;
                case 3:  new_x += step; Flag_Move = 1; break;
                case 5:  new_y -= step; Flag_Move = 1; break;
                case 7:  new_x -= step; Flag_Move = 1; break;
                default: Flag_Move = 0; break;
            }

            // Judge first, then update
            if (Flag_Move)
            {
							  Flag_Move=0;
							  if(Arm2DOF_SetTargetEndEffector(&A280,new_x,new_y))
								{
									  UpdateMotorTargets_FromJoints(&A280,23);
								}

            }
        }

        //=====================================================================
        // Correct Mode (Maintains Original Logic)
        //=====================================================================
        else if (A280.kinematic_mode == Forward_Kinematic)
        {
					  float new_b = A280.joint_rad[1];
            float new_c = A280.joint_rad[2];
            switch (Flag_Direction)
            {
                case 1:  new_b += step * step_multi; Flag_Move = 1; break;
                case 3:  new_c += step * step_multi; Flag_Move = 1; break;
                case 5:  new_b -= step * step_multi; Flag_Move = 1; break;
                case 7:  new_c -= step * step_multi; Flag_Move = 1; break;
                default: Flag_Move = 0; break;
            }
            if(Flag_Move)
						{
							 Flag_Move=0;
							 Arm2DOF_SetTargetJoint(&A280,new_b, new_c);
							 UpdateMotorTargets_FromJoints(&A280,23);
						}
           
        }
        float joint_a=A280.joint_rad[0];
				
				uint8_t gripper_close=0,gripper_open=0;
        switch (Flag_Direction)
        {
            case 2:  joint_a+=step * step_multi;Flag_Move = 1; break;
            case 4:  gripper_close  = 1; break;
            case 6:  joint_a-=step * step_multi;Flag_Move = 1; break;
            case 8:  gripper_open   = 1; break;
            default: Flag_Move = 0; break;
        }
				if(gripper_open && A280.gripper>1)
				{
					 A280.gripper--;
					 Gripper_SetLevel(A280.gripper);					
				}else if(gripper_close && A280.gripper<(SERVO_LEVEL_MAX-1))
				{ 
					 A280.gripper++;
					 Gripper_SetLevel(A280.gripper);	
				}
				 if(Flag_Move)
				{
					 Flag_Move=0;				
					if(joint_a>A280.joint_min[0] && joint_a<A280.joint_max[0])
					{
						A280.joint_rad[0]=joint_a;
						UpdateMotorTargets_FromJoints(&A280,1);
					}					 
				}
       
    }
}


/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
Functions: FreeRTOS tasks, core motion control tasks
Entry parameters: None
Return value: none
**************************************************************************/
void Balance_task(void *pvParameters)
{
	   Arm2DOF_Init(&A280);
     u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        // This task runs at a frequency of 100Hz (10ms control once)
        //This task runs at 100Hz (controlled once every 10ms)
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        //Time count is no longer needed after 30 seconds
        //Time count, no longer needed after 30 seconds
        if(Time_count<3000)Time_count++;

        //Get the encoder data, that is, the real time wheel speed,
        //and convert to transposition international units
        //Obtain encoder data, i.e., the real-time speed of the wheels, and convert bits in international units
        Get_Velocity_Form_Encoder();

        command_lost_count++;//Serial port and CAN control command loss time count; control stops after 1 second of loss
        if(command_lost_count>RATE_100_HZ&&APP_ON_Flag==0&&Remote_ON_Flag==0&&PS2_ON_Flag==0)
            Move_X=0,Move_Y=0,Move_Z=0;
        if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands // Handle APP remote commands
        else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands // Handle model aircraft remote commands
        else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands // Handle PS2 controller commands

        //CAN, Usart 1, Usart 3 control can directly get the 2 axis target speed,
        //without additional processing
        //CAN, serial port 1, serial port 3 (ROS) control directly obtains the 2-axis target velocity without additional processing
        else      Drive_Motor(Move_X, Move_Y, Move_Z);  //CAN, serial port 1, serial port 3 (ROS) control

        if     (A280.control_mode==APP_Mode) 	Get_RC_ARM();
        else if(A280.control_mode==PS2_Mode) 	PS2_control_ARM();
        Key();
        //After the gyroscope initialization is complete, check whether the robot model was selected incorrectly
        //When the gyroscope is initialized, check whether the robot model is selected incorrectly
        if(CONTROL_DELAY<Time_count && Time_count<CONTROL_DELAY+200) //Advance 1 second to test // Move forward 1 second for testing
        {
            Drive_Motor(0.02,0, 0);
            robot_mode_check(); //Detection function // Detection function
        }
        else if(CONTROL_DELAY+200<Time_count && Time_count<CONTROL_DELAY+210) Drive_Motor(0, 0,0); //Stop forward control completed // Stop forward control is completed

        //If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0, or the model detection marker is 0
        //If battery voltage is normal, the enable switch is ON, and the software disable flag is 0, or the model detection flag is 0
        if(Turn_Off(Voltage)==0&&robot_mode_check_flag==0)
        {
           //Speed closed-loop control to calculate the PWM value of each motor, 
			//PWM represents the actual wheel speed					 
			//Closed-loop speed control calculates each motor's PWM; PWM represents actual wheel speed
			MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
			MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
			
			//Detect whether PWM needs to be cleared and perform cleanup automatically
			auto_pwm_clear();
			//Set different PWM control polarity according to different car models
			//Set different PWM control polarities according to different car models

			if (Car_Mode==0||Car_Mode==1||Car_Mode==2||Car_Mode==3)  
				Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm);  //MD36 motor series
			else                           
				Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,-MOTOR_C.Motor_Pwm,-MOTOR_D.Motor_Pwm);  //MD60 motor series

        }
        //If Turn_Off(Voltage) returns to 1, or the model detection marker is 1, the car is not allowed to move, and the PWM value is set to 0
        //If the Turn_Off (Voltage) return value is 1, or the model detection marker is set to 1, controlling the chassis is not allowed to move; PWM value is set to 0
        else	Set_Pwm(0,0,0,0);
    }
}

void PS2_control_ARM(void)
{
	  u8 Flag_Move = 0;
    int LX, LY, RY, RX;
    int Threshold = 20;  // Threshold to ignore small joystick movements

    // 128 is the median
    LX = -(PS2_LX - 128);
    LY = -(PS2_LY - 128);
    RX = -(PS2_RX - 128);
    RY = -(PS2_RY - 128);

    // Ignore small joystick movements
    if (LX > -Threshold && LX < Threshold) LX = 0;
    if (LY > -Threshold && LY < Threshold) LY = 0;
    if (RY > -Threshold && RY < Threshold) RY = 0;
    if (RX > -Threshold && RX < Threshold) RX = 0;
 
    // If in robotic arm control mode
    if (A280.control_mode==PS2_Mode)
    {
			
        static float step = 0.002f;
        float step_multi = 5.0f;

        // Acceleration/deceleration control
        if (PS2_KEY == 11)      step += 0.00001f;  // Acceleration
        else if (PS2_KEY == 9)  step -= 0.00001f;  // Slow down
				
		

					if (LY != 0 || LX != 0)
				{
					
						 float new_x = A280.ee_x;
						 float new_y = A280.ee_y;
				
						if (LY > 0) new_y += step,Flag_Move=1;
						if (LY < 0) new_y -= step,Flag_Move=1;
						if (LX > 0) new_x -= step,Flag_Move=1;
						if (LX < 0) new_x += step,Flag_Move=1;

						if (Flag_Move)
						{
								Flag_Move=0;
								if(Arm2DOF_SetTargetEndEffector(&A280,new_x,new_y))
								{
										UpdateMotorTargets_FromJoints(&A280,23);
								}

						}
				} else if (RY != 0 || RX != 0)
				{
					
						 float new_b = A280.joint_rad[1];
             float new_c = A280.joint_rad[2];
				
						if (RY > 0) new_b += step * step_multi, Flag_Move = 1;
						if (RY < 0) new_b -= step * step_multi, Flag_Move = 1;
						if (RX > 0) new_c -= step * step_multi, Flag_Move = 1;
						if (RX < 0) new_c += step * step_multi, Flag_Move = 1;

						if (Flag_Move)
						{
								Flag_Move=0;
								Arm2DOF_SetTargetJoint(&A280,new_b, new_c);
							  UpdateMotorTargets_FromJoints(&A280,23); 
						}
				}      

        //=====================================================================
        // Mechanical claw control
        //=====================================================================
        uint8_t gripper_close=0,gripper_open=0;
				if (PS2_KEY == 10) gripper_close  = 1;
        if (PS2_KEY == 12) gripper_open =1;
				if(gripper_open && A280.gripper>1)
				{
					 A280.gripper--;
					 Gripper_SetLevel(A280.gripper);					
				}else if(gripper_close && A280.gripper<(SERVO_LEVEL_MAX-1))
				{ 
					 A280.gripper++;
					 Gripper_SetLevel(A280.gripper);	
				}
				
				uint8_t joint_a_move=0;
				float joint_a=A280.joint_rad[0];
				if (PS2_KEY == 8) joint_a+=step * step_multi,joint_a_move = 1;
        if (PS2_KEY == 6)  joint_a-=step * step_multi,joint_a_move = 1;
				if(joint_a_move)
				{
					 joint_a_move=0;				
					if(joint_a>A280.joint_min[0] && joint_a<A280.joint_max[0])
					{
						A280.joint_rad[0]=joint_a;
						UpdateMotorTargets_FromJoints(&A280,1);
					}					 
				}
				
    }
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
Function: Assign values to PWM registers to control wheel speed and direction
Input parameters: PWM
Return value: none
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
    //Forward and reverse control of motor
    //Motor forward and reverse control
    if(motor_a>0)			AIN1=0,		AIN2=1;
    else 	            AIN1=1,		AIN2=0;
    //Motor speed control
    //Motor speed control
    TIM_SetCompare4(TIM8,myabs(motor_a*A));

    if(motor_b>0)			BIN1=0,		BIN2=1;
    else 	            BIN1=1,			BIN2=0;
    //Motor speed control
    //Motor speed control
    TIM_SetCompare3(TIM8,myabs(motor_b*B));

    //Forward and reverse control of motor
    //Motor forward and reverse control
    if(motor_c>0)			CIN2=0,		CIN1=1;
    else 	            CIN2=1,			CIN1=0;
    //Motor speed control
    //Motor speed control
    TIM_SetCompare2(TIM8,myabs(motor_c*C));

    if(motor_d>0)			DIN2=0,		DIN1=1;
    else 	           DIN2=1,			DIN1=0;
    //Motor speed control
    //Motor speed control
    TIM_SetCompare1(TIM8,myabs(motor_d*DD));
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
Function: Limit PWM values
Entry parameters: magnitude
Return value: none
**************************************************************************/
void Limit_Pwm(int amplitude)
{
    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
    MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
Function: Limit function
Entry parameters: magnitude
Return value: none
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
Function: check battery voltage, enable switch state and software-disable flag
Entry parameters: voltage
Return value: whether control is allowed; 1-not allowed, 0-allowed
**************************************************************************/
u8 Turn_Off( int voltage)
{
    u8 temp;
    //static int stop_count, enable_count;
    if(voltage<20||EN==0||Flag_Stop==1)
    {
        temp=1;
        PWMA=0;
        PWMB=0;
        PWMC=0;
        PWMD=0;
        AIN1=0;
        AIN2=0;
        BIN1=0;
        BIN2=0;
        CIN1=0;
        CIN2=0;
        DIN1=0;
        DIN2=0;
    }
    else
        temp=0;
    return temp;
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
Function: calculate absolute values
Entry parameter: long int
Return value: unsigned int
**************************************************************************/
u32 myabs(long int a)
{
    u32 temp;
    if(a<0)  temp=-a;
    else temp=a;
    return temp;
}


/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
Function: Calculates absolute values for floating-point data
Entry parameter: floating-point number
Return value: absolute value of the input
**************************************************************************/
float float_abs(float insert)
{
    if(insert>=0) return insert;
    else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)

Function: incremental PI controller
Entry parameters: encoder measurement (actual speed), target speed
Return value: motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) represents the previous deviation, and so on
pwm stands for incremental output
In our speed control closed-loop system, only PI control is used
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)
**************************************************************************/


int Incremental_PI_A (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation // Calculate the deviation
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>16800)Pwm=16800;
    if(Pwm<-16800)Pwm=-16800;
    Last_bias=Bias; //Save the last deviation //Save the last deviation
	
	//Clear the PWM flag; a value of 1 indicates PWM needs to be cleared
	if( start_clear ) 
	{
		//Gradually decreasing PWM removal reduces the impact of slight motor movement caused by motor release
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//When clearing completes, set the flags; use 4 bits, one for each of the 4 motors
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
    return Pwm;
}
int Incremental_PI_B (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation // Calculate the deviation
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>16800)Pwm=16800;
    if(Pwm<-16800)Pwm=-16800;
    Last_bias=Bias; //Save the last deviation //Save the last deviation
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
	}
    return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation // Calculate the deviation
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>16800)Pwm=16800;
    if(Pwm<-16800)Pwm=-16800;
    Last_bias=Bias; //Save the last deviation //Save the last deviation
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<2;
		else clear_state &= ~(1<<2);
	}
    return Pwm;
}
int Incremental_PI_D (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
//	static u8 clear_times = 0;
	
    Bias=Target-Encoder; //Calculate the deviation // Calculate the deviation
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>16800)Pwm=16800;
    if(Pwm<-16800)Pwm=-16800;
    Last_bias=Bias; //Save the last deviation //Save the last deviation
	
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//If all four motors are cleared, the removal task will be closed
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	
    return Pwm;
}

/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
Function: Processes commands sent from the app via serial port 2
Entry parameters: None
Return value: none
**************************************************************************/
void Get_RC(void)
{
    switch(Flag_Direction) //Handle direction control commands // Handle direction control commands
	{ 
		case 1:      Move_X=RC_Velocity;  	 Move_Z=0;        break;
		case 2:      Move_X=RC_Velocity;  	 Move_Z=-PI/4;  	break;
		case 3:      Move_X=0;      				 Move_Z=-PI/4;  	break;
		case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/4;  	break;
		case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;  	    break;
		case 6:      Move_X=-RC_Velocity;  	 Move_Z=PI/4;     break;
		case 7:      Move_X=0;     	 			 	 Move_Z=PI/4;  	  break;
		case 8:      Move_X=RC_Velocity; 	 	 Move_Z=PI/4;  	  break; 
		default:     Move_X=0;               Move_Z=0;  	    break;
	}

	if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation
	else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation // Right rotation
	
	#if _4WD
  if(Move_X<0) Move_Z = -Move_Z;
	#endif
	
	//Unit conversion, mm/s -> m/s
  //Unit conversion, mm/s - > m/s
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
  //Control target value is obtained and kinematics analysis is performed
	//Get control target values for kinematic analysis
	Drive_Motor(Move_X,Move_Y,Move_Z);
		
		
}




/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
Function: Handles PS2 controller control commands
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_control(void)
{
 	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick // Threshold, ignore small movements of the joystick
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128 is the median. The PS2 coordinate system and the ROS coordinate system define X and Y differently
		LY=-(PS2_LX-128);
		LX=-(PS2_LY-128);
		RY=-(PS2_RX-128);
	
	  //Ignore small movements of the joystick
		if(LX>-Threshold&&LX<Threshold)LX=0;
		if(LY>-Threshold&&LY<Threshold)LY=0;
		if(RY>-Threshold&&RY<Threshold)RY=0;
		if(LX==0) Move_X=Move_X/1.2f;
		if(RY==0) Move_Z=Move_Z/1.2f;
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down // Slow down
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //Handle PS2 controller control commands
		Move_X=LX;
		Move_Y=LY;
		Move_Z=RY;
		Move_X=Move_X*RC_Velocity/128;
		Move_Y=Move_Y*RC_Velocity/128;
		Move_Z=Move_Z*(PI/4)*(RC_Velocity/500)/128;
		
		//Z-axis data transformation
		#if _4WD
		if(Move_X<0) Move_Z = -Move_Z;	
		Move_Y = 0;
		#endif
		
		//Unit conversion, mm/s -> m/s
    //Unit conversion, mm/s - > m/s
		Move_X=Move_X/1000;
		Move_Y=Move_Y/1000;
		 
		//Control target value is obtained and kinematics analysis is performed
	  //Get control target values for kinematic analysis
		Drive_Motor(Move_X,Move_Y,Move_Z);		 		
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
Function: Processes remote control commands for model aircraft
Entry parameters: None
Return value: none
**************************************************************************/
void Remote_Control(void)
{
   //Data within 1 second after entering the model control mode will not be processed
	  //Data within 1 second after entering model aircraft control mode will not be processed
    static u8 thrice=100;
    int Threshold=100;

	  //limiter // Limit
    int LX,LY,RY,RX,Remote_RCvelocity; 
//	  static float Target_LX,Target_LY,Target_RY,Target_RX;
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

		// Front and back direction of left rocker. Control forward and backward.
	  //Left stick forward and backward direction. Controls forward and backward movement.
    LX=Remoter_Ch2-1500;
	
//		//Left joystick left and right. Control left and right movement.
//	  //Left stick left and right direction. Control left and right movement.
//    LY=Remoter_Ch4-1500;
	
		//Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//Right stick forward and backward. Throttle/acceleration/deceleration.
	  RX=Remoter_Ch3-1500;		//
	
    //Right stick left and right. To control the rotation. 
		//Right stick left and right direction. Controls rotation.
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;
		
		
//		if(LX==0) Target_LX=Target_LX/1.2f;
//		if(LY==0) Target_LY=Target_LY/1.2f;
//		if(RY==0) Target_RY=Target_RY/1.2f;
		
		
		//Throttle related // Throttle related
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//Processing remote control commands for model aircraft
    Move_X= LX; 
		Move_Y=-LY;
		Move_Z=-RY; 
    Move_X= Move_X*Remote_RCvelocity/500; 
		Move_Y= Move_Y*Remote_RCvelocity/500;
		Move_Z= Move_Z*(PI/4)/500;  
			 
		//Unit conversion, mm/s -> m/s
    //Unit conversion, mm/s - > m/s
		Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;
		
		//Z-axis data transformation
		#if _4WD
		if(Move_X<0) Move_Z = -Move_Z;	
		Move_Y = 0;
		#endif
		
		//Data within 1 second after entering the model control mode will not be processed
	  //Data within 1 second after entering model aircraft control mode will not be processed
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
			
		//Control target value is obtained and kinematics analysis is performed
	  //Get control target values for kinematic analysis
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
Function: Click the user button to update the gyroscope zero
Entry parameters: None
Return value: none
**************************************************************************/
void Key(void)
{
    u8 tmp;
    //Frequency of incoming tasks
    tmp=KEY_Scan(RATE_100_HZ,0);

    //Click
    if(tmp==single_click)
	{
		memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
	}		

    //Double-click
    else if(tmp==double_click) memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));

    //Long press
    else if(tmp==long_click)
    {
        oled_refresh_flag=1;
        oled_page++;
        if(oled_page>OLED_MAX_Page-1) oled_page=0;
    }
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
Function: Read encoder values and calculate wheel speed, unit m/s
Entry parameters: None
Return value: none
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
    float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; //Used to get raw encoder data

	//Obtain the original data of the encoder, and the polarity of different models of cars is also different
	//Obtain the original data of the encoder, and the polarity of different models of carts varies
	#if _4WD
	Encoder_A_pr= Read_Encoder(2); 
	Encoder_B_pr= Read_Encoder(3); 
	Encoder_C_pr=-Read_Encoder(4); 
	Encoder_D_pr=-Read_Encoder(5);
	#elif Omni
	Encoder_A_pr=-Read_Encoder(2);
	Encoder_B_pr=-Read_Encoder(3);
	Encoder_C_pr=-Read_Encoder(4);
	#endif

	//Collect encoder data before POST is complete
	if( check_end==0 )
	{
		check_a+=Encoder_A_pr;
		check_b+=Encoder_B_pr;
		check_c+=Encoder_C_pr;
		check_d+=Encoder_D_pr;
	}
	
	//The encoder converts the raw data to wheel speed in m/s
	//The encoder's raw data is converted into wheel speed, unit m/s
	MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
	MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;

}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
Function: smooths the speed of the three-axis target
Entry parameters: three-axis target speed
Return value: none
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
    float step=0.02; //Smooth step values

    //Smooth X-axis speed
    if(vx>smooth_control.VX)
    {
        smooth_control.VX+=step;
        if(smooth_control.VX>vx) smooth_control.VX=vx;
    }
    else if (vx<smooth_control.VX)
    {
        smooth_control.VX-=step;
        if(smooth_control.VX<vx) smooth_control.VX=vx;
    }
    else
        smooth_control.VX =vx;

    //Smooth Y-axis speed
    if(vy>smooth_control.VY)
    {
        smooth_control.VY+=step;
        if(smooth_control.VY>vy) smooth_control.VY=vy;
    }
    else if (vy<smooth_control.VY)
    {
        smooth_control.VY-=step;
        if(smooth_control.VY<vy) smooth_control.VY=vy;
    }
    else
        smooth_control.VY =vy;

    //Smooth Z-axis speed
    if(vz>smooth_control.VZ)
    {
        smooth_control.VZ+=step;
        if(smooth_control.VZ>vz) smooth_control.VZ=vz;
    }
    else if (vz<smooth_control.VZ)
    {
        smooth_control.VZ-=step;
        if(smooth_control.VZ<vz) smooth_control.VZ=vz;
    }
    else
        smooth_control.VZ =vz;

    //0 speed ensures stability when stationary
    if(vx==0&&smooth_control.VX<0.05f&&smooth_control.VX>-0.05f) smooth_control.VX=0;
    if(vy==0&&smooth_control.VY<0.05f&&smooth_control.VY>-0.05f) smooth_control.VY=0;
    if(vz==0&&smooth_control.VZ<0.05f&&smooth_control.VZ>-0.05f) smooth_control.VZ=0;
}
///**************************************************************************
//Function: Smoothing the front wheel steering speed to prevent excessive steering gear current
//Input   : Current servo PWM, Target servo PWM, Smooth value
//Output  : none
//Function: smooths the steering speed of the front wheels to prevent excessive servo current
//Entry parameters: Current servo control PWM value, target servo control PWM value, smoothing value
//Return value: none
//**************************************************************************/
//int Smooth_steering(int currentPWM, int targetPWM, float step)
//{
//	int threshold=7;
//	if     (targetPWM>currentPWM+threshold) currentPWM+=step;
//	else if(targetPWM<currentPWM-threshold) currentPWM-=step;
//	else                                    currentPWM =targetPWM;
//
//	return currentPWM;
//}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.
Input   : none
Output  : none
Function: prevents the potentiometer from selecting the wrong mode and causing initialization errors and erratic generator rotation.
Entry parameters: None
Return value: none
**************************************************************************/
void robot_mode_check(void)
{
	#define ERROR_PWM 7000 //PWM warning value
	static u8 once=1;
	static int last_a,last_b,last_c,last_d;
	static u8 times;
	
	if( once ) check_a=0,check_b=0,check_c=0,check_d=0,once=0;
	
	if( EN==1 && robot_mode_check_flag==0) //Retain the ability to skip self-test using the emergency stop switch
	{
		if(Time_count<CONTROL_DELAY+200) //Do not detect the following during the vehicle stop to prevent human interference
		{
			//If the cumulative value regresses, it indicates an abnormality in motor control. Usually, this is the result of two errors: the motor driver power wiring does not match the motor + encoder wiring does not match
			if( check_a<last_a-1000 || check_b<last_b-1000 || check_c < last_c-1000 || check_d <last_d-1000 )	
			{
				times++;
				if( times>2 ) robot_mode_check_flag=1,LED_G=0;
			}
			last_a = check_a,last_b = check_b,last_c = check_c,last_d = check_d;
			
			//If there is a negative number, it indicates the opposite direction: error types include: wrong model selection, driver wiring error, or encoder wiring error
			if( check_a<-3000 ||check_b<-3000 ||check_c<-3000 ||check_d<-3000 ) robot_mode_check_flag=1,LED_G=0;
		}

		//After obtaining a certain PWM parameter value, if the encoder data remains unchanged. Error types are: encoder not wired, driver not wired, or overweight
		if(float_abs(MOTOR_A.Motor_Pwm)>5500 && check_a<500) robot_mode_check_flag=1,LED_B=0;	
		if(float_abs(MOTOR_B.Motor_Pwm)>5500 && check_b<500) robot_mode_check_flag=1,LED_B=0;	
		if(float_abs(MOTOR_C.Motor_Pwm)>5500 && check_c<500) robot_mode_check_flag=1,LED_B=0;
		if(float_abs(MOTOR_D.Motor_Pwm)>5500 && check_d<500) robot_mode_check_flag=1,LED_B=0;
		
		//Last line of defense: PWM values that cannot be reached at normal 0.1m/s speeds. Error type: encoders A and B are reversed, or C/D are reversed, or the load has exceeded the motor's capacity
		if( float_abs(MOTOR_A.Motor_Pwm)>ERROR_PWM||float_abs(MOTOR_B.Motor_Pwm)>ERROR_PWM||\
			 float_abs(MOTOR_C.Motor_Pwm)>ERROR_PWM||float_abs(MOTOR_D.Motor_Pwm)>ERROR_PWM )
		{
			robot_mode_check_flag = 1;
			LED_B=1,LED_G=1;
		}
		
	}
}

void robot_slefcheck(void)
{
	if( smooth_control.VX==0&&smooth_control.VY==0&&smooth_control.VZ==0 )
	{
		if( MOTOR_A.Motor_Pwm> 16000&&MOTOR_B.Motor_Pwm<-16000 ||\
            MOTOR_A.Motor_Pwm<-16000&&MOTOR_B.Motor_Pwm> 16000 ||\
			MOTOR_C.Motor_Pwm> 16000&&MOTOR_D.Motor_Pwm<-16000 ||\
            MOTOR_C.Motor_Pwm<-16000&&MOTOR_D.Motor_Pwm> 16000  )
		{
			robot_mode_check_flag = 1;
		}
	}
}

//PWM elimination function
void auto_pwm_clear(void)
{
	//Simple assessment of the car's posture
	float y_accle = (float)(accel[1]/1671.84f);//Actual value of Y-axis acceleration
	float z_accle = (float)(accel[2]/1671.84f);//Actual Z-axis acceleration
	float diff;
	
	//Calculate the fusion of Y and Z accelerations; the closer this value is to 9.8, the more horizontal the car's attitude is
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM elimination detection
	if( smooth_control.VX !=0.0f || smooth_control.VZ != 0.0f || smooth_control.VY != 0.0f )
	{
		start_check_flag = 1;//Tag Requires PWM Clearing
		wait_clear_times = 0;//Reset the timer to clear the timer
		start_clear = 0;     //Reset the clear flag
		
		
		//Reset slope-detection data during motion
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //When the target speed changes from non-zero to zero, start a 2.5-second timer; if the car is not on a slope, clear the PWM
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//Only mark the PWM when the chassis is on a horizontal plane to prevent slippage when moving on slopes
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//Enable PWM clearing
				else clear_done_once = 1;//The cart is on the slope, marking clearance
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//After clearing the card, if cart pushing occurs, PWM will accumulate a certain value and then clear again after 10 seconds
	if( clear_done_once )
	{
		//Accumulation elimination is performed only when the chassis approaches the horizontal plane, preventing the chassis from rolling on the slope
		if( diff > 8.8f )
		{
			//After clearing is complete, PWM accumulates again and is cleared again
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//Enable PWM clearing
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}

/**************************************************************************
Function: input the target angle to the servo and output the corresponding PWM value
Inlet parameter: servo steering angle (unit: radians)
Return value: corresponding output PWM value (range: 500--2500)
**************************************************************************/
int SERVO_PWM_VALUE(float angle)
{
	    int Amplitude_H=2500, Amplitude_L=500; 
		int K=1000;
		float Ratio=0.6369; //180¡ã servo
	    //float Ratio = 0.4246; // 270¡ã servo
		int pwm_value;
			
		pwm_value=(SERVO_INIT-angle*K*Ratio); //Servo target PWM value
	    if(pwm_value<Amplitude_L)  Servo1=Amplitude_L;	
		if(pwm_value>Amplitude_H)  Servo1=Amplitude_H;	
	    return pwm_value;
	
}

/**************************************************************************
Function: Position-based PID controller
Entry parameters: encoder measurement position information, target position
Return value: motor PWM
According to the positional discrete PID formula
pwm=Kp*e(k)+Ki*¡Æe(k)+Kd[e£¨k£©-e(k-1)]
e(k) represents the current deviation
e(k-1) represents the previous deviation
¡Æe(k) represents the cumulative sum of e(k) and previous deviations; where k is 1, 2, k;
pwm represents output
**************************************************************************/
float Position_PID1 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //Calculate bias
	 Integral_bias+=Bias;	                                 //Calculate the integral of the deviation
	 Pwm=10*Bias/100+0*Integral_bias/100+18*(Bias-Last_Bias)/100;       //Position-based PID controller
	 Last_Bias=Bias;                                       //Save the previous deviation
	if(Pwm<-120)  Pwm=-120;	
    if(Pwm>120)  Pwm=120;
	 return Pwm;                                           //Incremental output
}

void Jiazhua_Set_Pwm(float velocity1)
{	
		Position1+=velocity1;		   //Integrate velocity to obtain servo position
	  //Register assigned to STM32
	    TIM9->CCR2=(int)Position1;   //Claw
}
