#include "balance_task.h"

//Related to car control
ROBOT_CONTROL_t robot_control;

//Variables related to the car self-test
ROBOT_SELFCHECK_t robot_check;
SemaphoreHandle_t motorsMutex;

//Variables related to robot parking mode
static ROBOT_PARKING_t park;
int Servo1=0;
float Position1=1500,Velocity1=0;
//Incremental PI controller
PI_CONTROLLER PI_MotorA,PI_MotorB,PI_MotorC,PI_MotorD,PI_Servo;

#if defined AKM_CAR
AKM_SERVO_UNLOCK_t ServoState;//Servo non-self-locking controller
#endif
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
		motorsMutex = xSemaphoreCreateMutex();

   
    while(1)
    {
        // This task runs at a frequency of 100Hz (10ms control once)
        //This task runs at 100Hz (controlled once every 10ms)
        vTaskDelayUntil(&lastWakeTime, F2T(BALANCE_TASK_RATE));

        //Time count is no longer needed after 30 seconds
        //Time count, no longer needed after 30 seconds
        if(SysVal.Time_count<3000) SysVal.Time_count++;
		
        //and convert to transposition international units
        //Obtain encoder data feedback and servo data feedback
        Get_Robot_FeedBack();
////			printf("aaaa \n");
		//Automatic recharge equipment, status, RC transmitter, and other signal monitoring
//		    Charger_DevCheck();
		
        //Function to Scan User Key Press Status
        //Scan user key status
        UserKey_Scan( BALANCE_TASK_RATE );
		
		//Robot parking status monitoring, clearing residual motor control signals. \
		  Reduce power consumption, reduce motor noise.
		//Robot parking status monitoring, clearing residual motor control input. Reduces power consumption and motor noise.
		    Robot_ParkingCheck();
		
		//Robot power-on self-test
        //After the gyroscope initialization is complete, check whether the robot model was selected incorrectly
        //When the gyroscope is initialized, check whether the robot model is selected incorrectly
        if(CONTROL_DELAY<SysVal.Time_count && SysVal.Time_count<CONTROL_DELAY+200)
        {
            Drive_Motor(0.2f,0,0);//Low-speed forward test
			//TODO: Self-inspection requires testing various vehicle models
            robot_mode_check();  //Detection function // Detection function
        }
        else if(CONTROL_DELAY+200<SysVal.Time_count && SysVal.Time_count<CONTROL_DELAY+230)
        {
						robot_check.check_end = 1;
            Drive_Motor(0,0,0); //Stop forward control completed // Stop forward control is completed
						SysVal.LED_delay = 500;
        }
		
		//After the self-check is completed, obtain the control commands for the robot.
        //After the self-test is complete, obtain the robot's control commands
        if(SysVal.Time_count>CONTROL_DELAY+230)
        {
					if( 0 == SysVal.SecurityLevel )//0 is the highest safety level, with robot control command loss protection
					{
						//TODO: Monitoring for loss. Different control methods require different frequency of detection; actual measurement is based on effectiveness.
						//uart, ros, can, usb ps2, Android app usable, Apple app cannot be used, RC model needs other implementation
						//TODO: safety level only applies to uart, ros, can. PS2, app, and model aircraft signal loss requires unconditional stopping
						robot_control.command_lostcount++;
						if( robot_control.command_lostcount>BALANCE_TASK_RATE )
							robot_control.Vx = 0 , robot_control.Vy = 0 , robot_control.Vz = 0;
					}				
					//Normal mode control command
					else
					{
						if      ( Get_Control_Mode(_APP_Control)  )    Get_APPcmd();      //Handle the APP remote commands // Handle APP remote commands
						else if ( Get_Control_Mode(_RC_Control)   )    Remote_Control();  //Handle model aircraft remote commands // Handle model aircraft remote commands
						else if ( Get_Control_Mode(_PS2_Control)  )    PS2_control();     //Handle PS2 controller commands // Handle PS2 controller commands
						
						//CAN, Uart x control can directly get the 3 axis target speed, 
						//without additional processing
						//CAN and serial port X control directly obtain 3-axis target velocity without additional processing
						else    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
						
						if     (A280.control_mode==APP_Mode) 	Get_RC_ARM();
					}
      }
     
//			

		//Deep self-test mode, accessible by the user when needed
		if( 1 == robot_check.DeepCheck && SysVal.Time_count>CONTROL_DELAY+300)
		{
			if( 1 == Deep_SelfCheck( BALANCE_TASK_RATE ) )
			{
				robot_check.DeepCheck = 0;
			}
			continue;
		}
		
		//Robot Operation Status Inspection. Inspection Items: Check if the voltage is too low, \
		if there are any self-test errors, and if the emergency stop switch has been pressed.   \
		The robot will not be allowed to be controlled if any of these conditions are not met.
		//Robot operating status check. Detection content: whether voltage is too low, whether the self-check is incorrect, whether the emergency stop switch has been pressed. If conditions are not met, the robot will not be allowed to be controlled
		robot_control.FlagStop = Turn_Off();
		
		#if defined AKM_CAR
			Servo_UnLock_Check(robot_control.FlagStop);//Monitoring of servo non-self-locking modes
		#endif
		
		//Robots are allowed to be controlled
		if( 0 == robot_control.FlagStop )
		{
			//Execute control, performing different controls according to different models and models.
			ResponseControl();
		}
		else //Disable control and clear control outputs using shaft-release mode
		{
			UnResponseControl(UN_LOCK);
		}
		
		
		//Save FLash parameters, including Ackermann parameters, correction factors, etc
		uint8_t flash_check = 0;
		flash_check = FlashParam_Save( &appkey.ParamSaveFlag ); //Save Flash parameters
		if( 1 == flash_check ) Buzzer_AddTask(1,100);      //Saved successfully, normal beep prompt for 1 second
		else if( 1 < flash_check ) Buzzer_AddTask(10,11); //Save failed, rapid beep with 10 warnings

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
		float Ratio=0.6369; //180° servo
	    //float Ratio = 0.4246; // 270° servo
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
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k) represents the current deviation
e(k-1) represents the previous deviation
∑e(k) represents the cumulative sum of e(k) and previous deviations; where k is 1, 2, k;
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

void Grap_Set_Pwm(float velocity1)
{	
		Position1+=velocity1;		   //Integrate velocity to obtain servo position
	  //Register assigned to STM32
	    TIM9->CCR2=(int)Position1;   //Claw
}









/*-------------------------------- Functions Related to PI Controller ------------------------------------*/
/*-------------------------------- PI Controller Related Functions ------------------------------------*/
/**************************************************************************
Functionality: Set PID Control Parameters - Configures the proportional (Kp) and integral (Ki) parameters for PID control.
Input Parameters: Kp parameter, Ki parameter.
Return Value: None.
Author: LUCSYS
Function: Set parameters for PID control
Entry parameters: Kp parameters, Ki parameters
Return value: none
Author: LUCSYS
**************************************************************************/
static void PI_SetParam(PI_CONTROLLER* p,int kp,int ki)
{
	//The PID parameter cannot be negative or 0.
	if( kp > 0 ) 
	{
		p->kp = kp;
	}
	if( ki > 0 ) 
	{
		p->ki = ki;
	}
}

void Set_Robot_PI_Param(int kp,int ki)
{
	if( kp > 0 ) robot.V_KP = kp;
	if( ki > 0 ) robot.V_KI = ki;
	
	//Set the four PI parameters
	PI_SetParam(&PI_MotorA,robot.V_KP,robot.V_KI);
	PI_SetParam(&PI_MotorB,robot.V_KP,robot.V_KI);
	PI_SetParam(&PI_MotorC,robot.V_KP,robot.V_KI);
	PI_SetParam(&PI_MotorD,robot.V_KP,robot.V_KI);
}

/**************************************************************************
Functionality: Incremental PI Controller Reset - Resets an incremental PI (Proportional-Integral) controller to its initial state.
Input Parameters: PI controller.
Return Value: None.
Author: LUCSYS
Function: Incremental PI controller reset
Entry parameters: PI controller
Return value: none
Author: LUCSYS
**************************************************************************/
static void PI_Controller_Reset(PI_CONTROLLER *p)
{
	p->Bias = 0;
	p->LastBias = 0;
	p->Output = 0;
}


/**************************************************************************
Functionality: Incremental PI Control - Implements the control logic for an 
               incremental PI (Proportional-Integral) controller using feedback and target values.
Input Parameters: PI controller, feedback value, target value.
Return Value: Control output result.
Author: LUCSYS
Function: incremental PI control
Entry parameters: PI controller, feedback value, target value
Return value: control output
Author: LUCSYS
**************************************************************************/
static int Incremental_MOTOR(PI_CONTROLLER* p,float current,float target)
{
	//Calculate bias
	p->Bias = target - current;
	
	//Calculating output
	p->Output += p->kp * ( p->Bias - p->LastBias ) + p->ki * p->Bias;
	
	//Output limiting
	if( p->Output >   FULL_DUTYCYCLE ) p->Output =   FULL_DUTYCYCLE;
	if( p->Output < - FULL_DUTYCYCLE ) p->Output = - FULL_DUTYCYCLE;
	
	//Save the current deviation
	p->LastBias = p->Bias;
	
	//Output
	return p->Output;
}

/**************************************************************************
Functionality: Incremental PI Control - Implements the control logic for an 
               incremental PI (Proportional-Integral) controller using feedback and target values.
Input Parameters: PI controller, feedback value, target value.
Return Value: Control output result.
Author: LUCSYS
Function: incremental PI control
Entry parameters: PI controller, feedback value, target value
Return value: control output
Author: LUCSYS
**************************************************************************/
#if defined AKM_CAR
static int Incremental_Servo(PI_CONTROLLER* p,float current,float target)
{
	//Servo independent PID parameters, not open for user modification. Adjusted to the optimal value
	float servo_kp = 0;
	float servo_ki = 0;
	int8_t servo_dir = 1;
	
	//Determine PI parameters based on the stroke of the rail and different vehicle models
	if( robot.type==2 || robot.type==3 ) //Top-spec oscillating-axle Ackermann chassis
	{
		//Due to mechanical structure, the travel for a right turn is relatively long, so you need to accelerate the steering speed
		if( target<0 || current<-200 ) servo_kp = 0.002f*1.3f , servo_ki = 0.006f*1.3f;
		else                           servo_kp = 0.002f , servo_ki = 0.006f;
	}
	else if( robot.type==4 || robot.type==5 ) //Top-spec independent-suspension Ackermann chassis
	{
		servo_dir = -1;//Servo reversed installation, with changes in execution direction
		
		//Due to mechanical structure, the travel for a right turn is relatively long, so you need to accelerate the steering speed
		if( target>0 || current<200 )  servo_kp = 0.002f*1.3f , servo_ki = 0.006f*1.3f;
		else                           servo_kp = 0.002f , servo_ki = 0.006f;
	}
	
	//Low-speed servo mode
	static uint16_t low_speedMode = 0;
	if( robot_control.ServoLow_flag ) 
	{
		servo_kp = 0.001,servo_ki = 0.003;
		low_speedMode++;
		if( low_speedMode > BALANCE_TASK_RATE*2 ) //Set the auto-exit time to 2 seconds
		{
			low_speedMode = 0;
			robot_control.ServoLow_flag = 0;//Automatically exit low-speed mode
		}
	}
	else
		low_speedMode = 0;
	
	//Calculate bias
	p->Bias = target - current ;
	
	//Output calculation
	p->Output += (servo_kp*servo_dir) * ( p->Bias - p->LastBias ) + (servo_ki*servo_dir) * p->Bias;
	
	//Output limiting
	if( p->Output > Akm_Servo.Max )  p->Output = Akm_Servo.Max;
	if( p->Output < Akm_Servo.Min )  p->Output = Akm_Servo.Min;
	
	//Save the current deviation
	p->LastBias = p->Bias;
	
	//Servo non-locking mode implementation. By adding flags to the judgment condition, non-self-locking mode can be disabled
	static uint16_t count = 0;
	if( robot_control.smooth_Servo == p->Output && robot_control.FlagStop==0 ) //Use this logic only when the robot allows control
	{
		count++;
		if( count>=BALANCE_TASK_RATE )
		{
			count=0;
			ServoState.UnLock = 1;           //Enter non-self-locking mode
			ServoState.UnLock_Pos =  current;//and record the servo's current position
			ServoState.UnLock_Target = target - Akm_Servo.Bias;//Record the target position of the servo
			ServoState.UnLock_Output = robot.SERVO.Output;//Record the servo's current PWM value
			return 0;//The servo does not lock itself after reaching the target angle
		}
	}
	else count = 0;
	
	#if 1
	//Smooth servo control to reduce acceleration
	//Slowly increase the servo value to the calculated target value
	if( robot_control.smooth_Servo > p->Output ) 
	{
		robot_control.smooth_Servo -= robot_control.smooth_ServoStep;
		if( robot_control.smooth_Servo <= p->Output ) robot_control.smooth_Servo = p->Output;
	}
	else if( robot_control.smooth_Servo < p->Output )
	{
		robot_control.smooth_Servo += robot_control.smooth_ServoStep;
		if( robot_control.smooth_Servo >= p->Output  ) robot_control.smooth_Servo = p->Output;
	}
	else
	{
		robot_control.smooth_Servo = p->Output;
	}
	
	return (int)robot_control.smooth_Servo;
	#else
		//Outputs directly without filtering
		return p->Output;
	#endif
}



/**************************************************************************
Function function: Determine the PWM value of the servo motor based on the position of the slide rail,
                   only available for top of the line models
Entrance parameters: servo encoder reading
Return value: Servo PWM
Author: LUCSYS
Function: Determines servo PWM value based on rail position, available only on top models
Entry parameters: servo encoder reading
Return value: servo PWM
Author: LUCSYS
**************************************************************************/
short get_ServoPWM(short TmpPos)
{
	//Formula fitting version, with good results, but each car has a different formula, so customers can fit it themselves
//	//Estimate the deviation between the servo position and the median value of the servo
//	//Top-spec oscillating-axle and independent-suspension servo directions are opposite.
//	if( robot.type == 2 || robot.type == 3 )
//	{
//		//Estimate servo PWM from slide-rail position; formula fitted from several sampled data sets
//		return 1.57e+03 + 4.04e-01*TmpPos + (-5.82e-05)*pow(TmpPos,2);
//	}
//	else if( robot.type == 4 || robot.type == 5 )
//	{
//		//Estimate servo PWM from slide-rail position; formula fitted from several sampled data sets
////		return 1.46e+03 + (-4.05e-01)*TmpPos + (-9.42e-07)*pow(TmpPos,2);
//		return 1.48e+03 + (-2.39e-01)*TmpPos + (4.90e-06)*pow(TmpPos,2);	
//	}
//	else
//		return 0;
	
	uint16_t pwm_val = Akm_Servo.Mid;
	
	//Estimated version
	if( robot.type==4||robot.type==5 )
	{
			 if( TmpPos>=500&&TmpPos<=1000 )    pwm_val = Akm_Servo.Mid - 250;
		else if( TmpPos>1000 )                  pwm_val = Akm_Servo.Mid - 450;
		else if( TmpPos<=-500&&TmpPos>=-1000 )   pwm_val = Akm_Servo.Mid + 250;
		else if( TmpPos<-1000 )                   pwm_val = Akm_Servo.Mid + 450;
	}
	else if( robot.type==2 || robot.type==3 )
	{
			 if( TmpPos>=500&&TmpPos<=1000 )    pwm_val = Akm_Servo.Mid + 250;
		else if( TmpPos>1000 )                  pwm_val = Akm_Servo.Mid + 450;
		else if( TmpPos<=-500&&TmpPos>=-1000 )   pwm_val = Akm_Servo.Mid - 250;
		else if( TmpPos<-1000 )                   pwm_val = Akm_Servo.Mid - 450;
	}
	
	return pwm_val;
}

/**************************************************************************
Function Function: Servo Calibration Function. When the servo is in non self-locking mode and 
the position of the servo changes due to external factors, this function can be used for calibration
Entrance parameters: None
Return value: None
Author: SHEELTEC
Function: servo correction function. When the servo is in non-self-locking mode, if external factors cause changes in its position, use this function for calibration
Entry parameters: None
Return value: none
Author: LUCSYS
**************************************************************************/
static void ServoCalibration(void)
{
	short tmp_pwm;
	tmp_pwm = get_ServoPWM( robot.SERVO.Encoder ); //Get the PWM value corresponding to the current location
	SERVO_TOP = tmp_pwm;                  //Initialize the servo position, which directly determines the servo position
	PI_Controller_Reset(&PI_Servo);       //Reset the servo's PI controller
	PI_Servo.Output = tmp_pwm;            //The output value of the PI controller will be used as the basis for speed smoothing.
	robot_control.smooth_Servo = tmp_pwm; //Smoothing value, which is the actual control quantity.
}

//Monitoring of servo non-self-locking modes
static void Servo_UnLock_Check(uint8_t car_stopflag)
{
	static uint8_t now = 0 ;
	static uint8_t last = 0;
	
	if(SysVal.Time_count < CONTROL_DELAY ) return;
	
	//Servo non-self-locking mode related
	if ( robot.type>=2 && robot.type!=9 ) //Top trim only
	{
		//Check chassis enable flag
		now = car_stopflag;
		if( now==1 && last==0 ) //The car is disabled
		{
			ServoState.UnLock_Pos = robot.SERVO.Encoder;//Record the position of the servo during inoperability
			ServoState.UnLock_Output = robot.SERVO.Output;//Servo PWM value
		}
		else if( now==0 && last==1 ) //The car is enabled
		{
			if( fabs( ServoState.UnLock_Pos - robot.SERVO.Encoder ) > 100 ) //If servo position is externally changed while disabled, recalibrate it
			{
				ServoState.UnLock = 0;
				ServoCalibration();//Correct the servo position
				robot_control.ServoLow_flag = 1;//Enter low-speed servo mode to ensure slow servo reset (top-spec Ackermann only)
			}
		}
		last = now;
		
		//Detect exit from non-self-locking mode
		if( 1 == ServoState.UnLock )
		{
			//During non-self-locking periods, the servo angle is externally altered and requires calibration.
			if( fabs( robot.SERVO.Encoder - ServoState.UnLock_Pos ) > 100 ) ServoState.wait_Calib = 1;
			
			//When the servo has a control load, release the non-self-locking and enter normal control mode
			if( fabs( robot.SERVO.Target - ServoState.UnLock_Target ) > 50 || 1 == ValChangeCheck(BALANCE_TASK_RATE,Akm_Servo.Bias,3)) 
			{
				if( ServoState.wait_Calib ) 
				{
					ServoState.wait_Calib=0,ServoCalibration();//If necessary, calibrate the servo first
					if( fabs(robot_control.smooth_Vx) < 0.001f ) robot_control.ServoLow_flag = 1;//If the vehicle is not moving, use low-speed correction during calibration
				}
				ServoState.UnLock = 0;//Release non-self-locking mode
			}		
			
		} 
	}//Vehicle type judgment end
}
#endif
/**************************************************************************
Functionality: PI Controller Clearing Function - Clears the output residue of a PI (Proportional-Integral) 
               controller when the robot is disabled, aiming to reduce power consumption and motor noise.
Input Parameters: PI controller.
Return Value: None.
Author: LUCSYS
Function: PI controller clears the function to clear output residue when the robot is prohibited, reducing power consumption and motor noise
Entry parameters: PI controller
Return value: none
Author: LUCSYS
**************************************************************************/
static uint8_t PI_Clear_Output(PI_CONTROLLER* p)
{
	u8 state = 0;//Clear flags
	
	if( p->Output > 0 ) p->Output -- ;
	if( p->Output < 0 ) p->Output ++ ;
	
	//When it approaches 0, set it directly to 0
	if( p->Output < 2.0f && p->Output > -2.0f ) p->Output = 0 , state = 1;
	
	//Return clear result: 0-incomplete; 1-cleared
	return state;
}

/*-------------------------------- Function for Inverse Kinematics Solution ------------------------------------*/
/*-------------------------------- Inverse Kinematics Functions for Various Vehicle Models ------------------------------------*/
#if defined AKM_CAR

//Input slider position to calculate the current front-left wheel steering angle
float SteeringStructure_ForwardKinematic(short ori_pos)
{
    float pos;
    float tmp_theta;
    float tmp_C;

    pos = (float)ori_pos*0.0244f + 103;

    tmp_C = ( -1500.5f - pow(pos,2) ) / ( 140.4f*sqrt( pow(pos,2) + 3340.8f ) );

    tmp_theta = atan2(pos,57.8f) - asin( tmp_C ) - PI;

    return tmp_theta/PI * 180.0f + 72.39f;
}

//Input front-left wheel steering angle; output target slide-rail potentiometer value
static int SteeringStructure_Reverse(float target_theta)
{
	float theta;
	float pos;
	
	//Top-spec oscillating-axle Ackermann chassis
	if( 2 == robot.type || 3 == robot.type )
	{
		theta = target_theta - 1.26f;   //6768.4f
		pos = 70.2f * cos(theta) + sqrt( 6084.0f - pow((70.2f*sin(theta)+57.8f),2) ) - 103;
		pos /= 0.0244f;
		
		//The output of the potentiometer must be within the reachable range.
		pos = target_limit_float(pos,-2000,2000);
	}
	
	//Top-spec independent-suspension Ackermann chassis
	else if( 4 == robot.type || 5 == robot.type )
	{
		theta = target_theta - 1.170f;    //24831.4f
		pos = 95.15f * cos(theta) + sqrt( 24025.0f - pow((95.15f*sin(theta)+56.5f),2) ) - 191.54f;
		pos /= 0.0244f;
		
		//The output of the potentiometer must be within the reachable range.
		pos = target_limit_float(pos,-2000,2000);
	}

	return (int)pos;

}

/**************************************************************************
Function function: Ackermann inverse kinematics solution
Entrance parameters: X-axis velocity (in m/s), left front wheel angle (in rad)
Return value: None
Author: LUCSYS
Function: Ackermann kinematic inverse solution
Entry parameters: X-axis speed (unit: m/s), left front wheel angle (unit: rad)
Return value: none
Author: LUCSYS
**************************************************************************/
static void InverseKinematics_akm(float Vx,float Vz) 
{
	//Before kinematic analysis, acquire the servo potentiometer calibration value for use in the calculations
	Akm_Servo.Bias = -get_DMA_ServoBias() / 5.0f;
	
	//Custom chassis: copy the appropriate kinematics according to servo type.
	if( robot.type==6 )
	{
	  
	}
	//SENIOR_AKM - High-end Ackermann
	else if(robot.type==0||robot.type==1||robot.type==9) 
	{
		//Ackermann car specific related variables // Ackermann car specific related variables
		float TurnR=0, Left_Angle=0;
		
		// For Ackerman small car, Vz represents the front wheel steering Angle
		//For the Ackermann car, Vz represents the right front wheel steering angle
		Left_Angle = Vz;
		
		//Limit left/right steering angles
		const float limt_angle = 30.0f;
		//Turning radius of the left limit
		float bd_turnR = robot.HardwareParam.AxleSpacing/tan(angle_to_rad(limt_angle)) + 0.5f*robot.HardwareParam.WheelSpacing;
		//Based on the ultimate turning radius, calculate the left wheel angle when turning right
		float bd_rightAngle = atan(robot.HardwareParam.AxleSpacing/((-bd_turnR)-0.5f*robot.HardwareParam.WheelSpacing));
		
		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//Front wheel steering angle limit (servo controls front wheel steering angle), unit: rad
		Left_Angle=target_limit_float(Left_Angle,bd_rightAngle,angle_to_rad(limt_angle));
		
		//Calculate the current turning radius based on the steering angle
		if(Left_Angle!=0)
			TurnR = robot.HardwareParam.AxleSpacing/tan(Left_Angle) + 0.5f*robot.HardwareParam.WheelSpacing;
		
		//Based on turning radius and track width, solve the speeds of the left and right wheels in reverse
		//Inverse kinematics //Inverse kinematics
		if(Left_Angle!=0)
		{	//Target velocity for both left and right wheels, measured in m/s
			robot.MOTOR_A.Target = Vx*(TurnR-0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
			robot.MOTOR_B.Target = Vx*(TurnR+0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
		}
		else //Control without steering
		{	
			robot.MOTOR_A.Target = Vx;
			robot.MOTOR_B.Target = Vx;
		}
		
		// The PWM value of the servo controls the steering Angle of the front wheel
		//Servo PWM value, which controls the steering angle of the front wheels
		//The correspondence between the front wheel angle and the servo value
//		robot.SERVO.Target    =  7.3f + 823.4f*Left_Angle + (-440.4f)*pow(Left_Angle,2) + (-28.8)*pow(Left_Angle,3);
		robot.SERVO.Target    =  7.85f + 847.0f*Left_Angle + (-394.0f)*pow(Left_Angle,2) + (-74.2f)*pow(Left_Angle,3);
		
		//Final servo output = median value + potentiometer correction value + calculated value
		robot.SERVO.Output = Akm_Servo.Mid + Akm_Servo.Bias + robot.SERVO.Target;
	}
	
	//TOP_AKM_BS - Top-tier Ackermann chassis swing suspension
	else if( robot.type==2 || robot.type==3 )
	{
		//Turning radius and left wheel angle
		float TurnR = 0, Left_Angle = 0;
		
		//For the Ackermann car, Vz represents the left front wheel steering angle
		Left_Angle = Vz;
		
		//Limit left/right steering angles
		const float limt_angle = 25.0f;
		
		//The link distance K when calculating the angle
		const float K = 0.326f;
		
		//Turning radius of the left limit
		float bd_turnR = robot.HardwareParam.AxleSpacing/tan(angle_to_rad(limt_angle)) + 0.5f*K;
		
		//Based on the ultimate turning radius, calculate the left wheel angle when turning right
		float bd_rightAngle = atan(robot.HardwareParam.AxleSpacing/((-bd_turnR)-0.5f*K));
		
		//Front wheel steering angle limit (servo controls front wheel steering angle), unit: rad
		Left_Angle=target_limit_float(Left_Angle,bd_rightAngle,angle_to_rad(limt_angle));//Note: Left + Right -
		
		//Calculation of Rail Target Values:
		robot.SERVO.Target = SteeringStructure_Reverse(Left_Angle);
		//robot. SERVO. Target+=Akm_Servo.Bias; By adding deviation values, the front wheel angle can be adjusted and compensated by a potentiometer
		
		//Calculate the current turning radius based on the steering angle
		if(Left_Angle!=0)
			TurnR = robot.HardwareParam.AxleSpacing/tan(Left_Angle) + 0.5f*K;
		
		//Calculation of Left and Right Wheel Target Values:
		//Based on turning radius and track width, solve the speeds of the left and right wheels in reverse
		//Inverse kinematics //Inverse kinematics
		if(Left_Angle!=0)
		{
			robot.MOTOR_A.Target = Vx*(TurnR-0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
			robot.MOTOR_B.Target = Vx*(TurnR+0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
		}
		else //Control without steering
		{
			robot.MOTOR_A.Target = Vx;
			robot.MOTOR_B.Target = Vx;
		}
		
	}
	
	else if ( robot.type==4 || robot.type==5 )
	{
		//Turning radius and left wheel angle
		float TurnR = 0, Left_Angle = 0;
		
		//For the Ackermann car, Vz represents the left front wheel steering angle
		Left_Angle = Vz;
		
		//Limit left/right steering angles
		const float limt_angle = 25.0f;
		
		//The link distance K when calculating the angle
		const float K = 0.441f;
		
		//Turning radius of the left limit
		float bd_turnR = robot.HardwareParam.AxleSpacing/tan(angle_to_rad(limt_angle)) + 0.5f*K;
		
		//Based on the ultimate turning radius, calculate the left wheel angle when turning right
		float bd_rightAngle = atan(robot.HardwareParam.AxleSpacing/((-bd_turnR)-0.5f*K));
		
		//Front wheel steering angle limit (servo controls front wheel steering angle), unit: rad
		Left_Angle=target_limit_float(Left_Angle,bd_rightAngle,angle_to_rad(limt_angle));//Note: Left + Right -
		
		//Calculation of Rail Target Values:
		robot.SERVO.Target = SteeringStructure_Reverse(Left_Angle);
		
		//Calculate the current turning radius based on the steering angle
		if(Left_Angle!=0)
			TurnR = robot.HardwareParam.AxleSpacing/tan(Left_Angle) + 0.5f*K;
		
		//Calculation of Left and Right Wheel Target Values:
		//Based on turning radius and track width, solve the speeds of the left and right wheels in reverse
		//Inverse kinematics //Inverse kinematics
		if(Left_Angle!=0)
		{
			robot.MOTOR_A.Target = Vx*(TurnR-0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
			robot.MOTOR_B.Target = Vx*(TurnR+0.5f*robot.HardwareParam.WheelSpacing)/TurnR;
		}
		else //Control without steering
		{
			robot.MOTOR_A.Target = Vx;
			robot.MOTOR_B.Target = Vx;
		}
	}
}

/**************************************************************************
Function function: Convert the target forward velocity Vx and target angular velocity Vz sent by
                the upper computer into the left front wheel steering angle of the Ackermann car
Entrance parameters: target forward velocity Vx, target angular velocity Vz, unit: m/s, rad/s
Return value: Left front wheel steering angle of Ackermann car, unit: rad
Function: Converts the target forward speed Vx and target angular velocity Vz sent from the host computer into the left front wheel rotation angle of the Ackermann car
Entry parameters: target forward velocity Vx, target angular velocity Vz, units: m/s, rad/s
Return value: Ackermann chassis front-left wheel steering angle, unit: rad
**************************************************************************/
float Akm_Vz_to_Angle(float Vx,float Vz)
{
    float TurnR, Angle_Left;//Turning radius, front-left wheel angle

    if(Vz!=0 && Vx!=0)
    {
		//Calculate the turning radius
		TurnR = Vx/Vz;
		
        //Ensure the turning radius does not exceed the calculation range of the kinematic formula
        if( TurnR > 0 && TurnR <= 0.5f*robot.HardwareParam.WheelSpacing ) TurnR = 0.5f*robot.HardwareParam.WheelSpacing+0.01f;
        if( TurnR < 0 && TurnR >= -0.5f*robot.HardwareParam.WheelSpacing ) TurnR = -0.5f*robot.HardwareParam.WheelSpacing-0.01f;
		
		//Determine the left front wheel's turning angle curvature based on the turning radius
		Angle_Left=atan(robot.HardwareParam.AxleSpacing/(TurnR - 0.5f*robot.HardwareParam.WheelSpacing));

    }
    else
    {
        Angle_Left=0;
    }

    return Angle_Left;
}

/**************************************************************************
Function function: Differential robot inverse kinematics solution
Entrance parameters: X-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author: LUCSYS
Function: kinematic inverse solution of differential robots
Inlet parameters: X-axis speed (unit: m/s), Z-axis speed (unit: rad/s)
Return value: none
Author: LUCSYS
**************************************************************************/
#elif defined DIFF_CAR
static void InverseKinematics_diff(float Vx,float Vz)
{
	robot.MOTOR_A.Target = Vx - Vz * robot.HardwareParam.WheelSpacing / 2.0f;
	robot.MOTOR_B.Target = Vx + Vz * robot.HardwareParam.WheelSpacing / 2.0f;
}

/**************************************************************************
Function function: inverse kinematics solution of the wheat wheel robot
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author: LUCSYS
Function: Kinematic inverse solution of the Mailun robot
Entry parameters: X-axis speed (unit: m/s), Y-axis speed (m/s), Z-axis speed (rad/s)
Return value: none
Author: LUCSYS
**************************************************************************/
#elif defined MEC_CAR
static void InverseKinematics_mec(float Vx,float Vy,float Vz)
{
    robot.MOTOR_A.Target = +Vy+Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_B.Target = -Vy+Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_C.Target = +Vy+Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_D.Target = -Vy+Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
}

/**************************************************************************
Function function: inverse kinematics solution of four-wheel drive robot
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author  :LUCSYS
Function: Kinematic inverse solution for four-wheel drive robots
Entry parameters: X-axis speed (unit: m/s), Y-axis speed (m/s), Z-axis speed (rad/s)
Return value: none
Author: LUCSYS
**************************************************************************/
#elif defined _4WD_CAR
static void InverseKinematics_4wd(float Vx,float Vz)
{
    robot.MOTOR_A.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_B.Target = Vx-Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_C.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
    robot.MOTOR_D.Target = Vx+Vz*(robot.HardwareParam.AxleSpacing+robot.HardwareParam.WheelSpacing);
}

/**************************************************************************
Function function: Inverse kinematics solution for omnidirectional wheeled robots
Entrance parameters: X-axis velocity (in m/s), Y-axis velocity (in m/s), Z-axis velocity (in rad/s)
Return value: None
Author: LUCSYS
Function: Omnidirectional wheel robot inverse kinematics
Entry parameters: X-axis speed (unit: m/s), Y-axis speed (m/s), Z-axis speed (rad/s)
Return value: none
Author: LUCSYS
**************************************************************************/
#elif defined OMNI_CAR
static void InverseKinematics_omni(float Vx,float Vy,float Vz)
{
    robot.MOTOR_A.Target =  Vy + robot.HardwareParam.TurnRadiaus*Vz;
    robot.MOTOR_B.Target = -robot.HardwareParam.X_PARAMETER*Vx - robot.HardwareParam.Y_PARAMETER*Vy + robot.HardwareParam.TurnRadiaus*Vz;
    robot.MOTOR_C.Target = +robot.HardwareParam.X_PARAMETER*Vx - robot.HardwareParam.Y_PARAMETER*Vy + robot.HardwareParam.TurnRadiaus*Vz;
}

#endif

/*-------------------------------- Control class related functions ------------------------------------*/
/*-------------------------------- Control Class Related Functions ------------------------------------*/
//Calculate the correction factor
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //Left-wheel correction, corresponding to a correction factor of 50~100 and 1.0~1.2 times
	{
		if( diffparam>=50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //Right wheel correction: 50~0 corresponds to a correction factor of 1.0~1.2 times
	{
		if( diffparam<=50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//If the condition is not met, the default is 1.
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
Function: Executes kinematic inverse solutions to calculate the target rotational speed of each wheel based on the target velocity of the three axes
Inlet parameters: X, Y, Z axis speeds
Return value: none
**************************************************************************/
static void Drive_Motor(float T_Vx,float T_Vy,float T_Vz)
{
	// Function to Limit Maximum Target Speed Input 
	//Maximum limit for the input target speed, unit m/s
	T_Vx = target_limit_float( T_Vx, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	T_Vy = target_limit_float( T_Vy, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	T_Vz = target_limit_float( T_Vz, -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	//Smoothly control the target speed input.
	//Smooth control of the input target speed
	if( charger.AllowRecharge==0 )
	{
		robot_control.smooth_Vx = Vel_SmoothControl(robot_control.smooth_Vx,T_Vx,robot_control.smooth_MotorStep);
		robot_control.smooth_Vy = Vel_SmoothControl(robot_control.smooth_Vy,T_Vy,robot_control.smooth_MotorStep);
		
		//For Ackermann chassis, Vz represents front-left wheel angle; do not smooth it here
		#if defined AKM_CAR
			robot_control.smooth_Vz = T_Vz;
		#else
			robot_control.smooth_Vz = Vel_SmoothControl(robot_control.smooth_Vz,T_Vz,robot_control.smooth_MotorStep);
		#endif	
	}
	else
	{
		robot_control.smooth_Vx = T_Vx;//In automatic recharge mode, the smoothing function is not used, improving the vehicle's response speed and docking.
		robot_control.smooth_Vy = T_Vy;
		robot_control.smooth_Vz = T_Vz;
	}

	// Call Inverse Kinematics Function for Corresponding Vehicle Model to Obtain Target Values for Wheels and Steering Angle
	//Call the kinematic inverse solution function for the corresponding model to obtain the target values and servo angles for each wheel
	#if defined AKM_CAR
		InverseKinematics_akm(robot_control.smooth_Vx,robot_control.smooth_Vz);
	#elif defined DIFF_CAR
		InverseKinematics_diff(robot_control.smooth_Vx,robot_control.smooth_Vz);
	#elif defined MEC_CAR
		InverseKinematics_mec(robot_control.smooth_Vx,robot_control.smooth_Vy,robot_control.smooth_Vz);
	#elif defined _4WD_CAR
		InverseKinematics_4wd(robot_control.smooth_Vx,robot_control.smooth_Vz);
	#elif OMNI_CAR
		InverseKinematics_omni(robot_control.smooth_Vx,robot_control.smooth_Vy,robot_control.smooth_Vz);
	#endif

	//Calculation of correction coefficient
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
	// Limit Final Wheel Speeds or Servo Travel After Kinematic Analysis.
	//After kinematic analysis, output the speed and servo angle of each wheel. Below is a limit for the final wheel speed or servo stroke.
	robot.MOTOR_A.Target = target_limit_float( robot.MOTOR_A.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	robot.MOTOR_B.Target = target_limit_float( robot.MOTOR_B.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
	
	#if defined AKM_CAR || defined DIFF_CAR
		robot.MOTOR_A.Target*=LeftWheelDiff;
		robot.MOTOR_B.Target*=RightWheelDiff;
	#elif defined MEC_CAR || defined _4WD_CAR
		robot.MOTOR_C.Target = target_limit_float( robot.MOTOR_C.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );
		robot.MOTOR_D.Target = target_limit_float( robot.MOTOR_D.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );	
		robot.MOTOR_A.Target*=LeftWheelDiff;
		robot.MOTOR_B.Target*=LeftWheelDiff;
		robot.MOTOR_C.Target*=RightWheelDiff;
		robot.MOTOR_D.Target*=RightWheelDiff;
	#elif defined OMNI_CAR
		robot.MOTOR_C.Target = target_limit_float( robot.MOTOR_C.Target , -robot_control.limt_max_speed , robot_control.limt_max_speed );	
		robot.MOTOR_B.Target*=LeftWheelDiff;
		robot.MOTOR_C.Target*=RightWheelDiff;
	#endif
}

/**************************************************************************
Function function: Control the corresponding function. After inverse kinematics solution, 
obtain the target values of each executing mechanism of the robot, and control the motion 
of the executing mechanism through PID control
Entrance parameters: None
Return value: None
Author: LUCSYS
Function: Control the corresponding function. After kinematic inverse solution, the target values of each actuator in the robot are derived, and the movement of the actuators is controlled via PID
Entry parameters: None
Return value: none
Author: LUCSYS
**************************************************************************/
static void ResponseControl(void)
{
	#if defined AKM_CAR
	//PI control
	robot.MOTOR_A.Output = Incremental_MOTOR( &PI_MotorA , robot.MOTOR_A.Encoder , robot.MOTOR_A.Target );
	robot.MOTOR_B.Output = Incremental_MOTOR( &PI_MotorB , robot.MOTOR_B.Encoder , robot.MOTOR_B.Target );
	
	//The servo of the top of the line Ackermann model only requires PI control. \
	The high-end model does not come with steering rail feedback
	//Only the servo on the top Ackermann model requires PI control. Higher trims do not have steering rail feedback.
	if( robot.type>=2 && robot.type!=9 && robot.type!=7 )
	{
		if(ServoState.UnLock==0)
			robot.SERVO.Output   = Incremental_Servo( &PI_Servo  , robot.SERVO.Encoder  ,  robot.SERVO.Target + Akm_Servo.Bias );	
	}
		
	//Output results to the actuator
	      if( robot.type == 6 )                      Set_Pwm(0,0,0,0,0); //Reserved custom chassis: set output signs according to motor type
	else if ( robot.type <= 1 || robot.type == 9)   Set_Pwm( robot.MOTOR_A.Output , robot.MOTOR_B.Output , 0 , 0 ,robot.SERVO.Output ); //High-spec Ackermann chassis: MD36 motor
	else if ( robot.type >= 2 && robot.type <= 5 )  Set_Pwm(-robot.MOTOR_A.Output ,-robot.MOTOR_B.Output , 0 , 0 ,robot.SERVO.Output ); //Top-spec Ackermann chassis: MD60 motor
	else if ( robot.type == 7 )                     Set_Pwm( robot.MOTOR_A.Output , robot.MOTOR_B.Output , 0 , 0 , robot.SERVO.Output ); //Dedicated for installation and testing. Servo open-loop control
	else if ( robot.type == 8 )                     Set_Pwm(-robot.MOTOR_A.Output ,-robot.MOTOR_B.Output , 0 , 0 , 1500 ); //Installation and testing only. Keep the servo in the middle position for calibration
	
	#elif defined DIFF_CAR
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	
	if( robot.type<=1 )  Set_Pwm( robot.MOTOR_A.Output , robot.MOTOR_B.Output ,0,0,0);//High-spec differential chassis: MD36 motor
	else                 Set_Pwm(-robot.MOTOR_A.Output ,-robot.MOTOR_B.Output ,0,0,0);//Top-spec differential chassis: MD60 motor
	
	#elif defined MEC_CAR
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	robot.MOTOR_D.Output = Incremental_MOTOR(&PI_MotorD,robot.MOTOR_D.Encoder,robot.MOTOR_D.Target);
	
	if( robot.type<=2 ) Set_Pwm( robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output, robot.MOTOR_D.Output,0);//High-spec Mecanum chassis: MD36 motor
	else                Set_Pwm(-robot.MOTOR_A.Output,-robot.MOTOR_B.Output,-robot.MOTOR_C.Output,-robot.MOTOR_D.Output,0);//Top-spec or flagship Mecanum chassis: MD60 motor
	
	#elif defined _4WD_CAR
	
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	robot.MOTOR_D.Output = Incremental_MOTOR(&PI_MotorD,robot.MOTOR_D.Encoder,robot.MOTOR_D.Target);
	
	if( robot.type<=3 ) Set_Pwm( robot.MOTOR_A.Output, robot.MOTOR_B.Output, robot.MOTOR_C.Output, robot.MOTOR_D.Output,0);//High-spec 4WD chassis: MD36 motor
	else                Set_Pwm(-robot.MOTOR_A.Output,-robot.MOTOR_B.Output,-robot.MOTOR_C.Output,-robot.MOTOR_D.Output,0);//Top-spec or flagship 4WD chassis: MD60 motor
	
	#elif defined OMNI_CAR
	 
	robot.MOTOR_A.Output = Incremental_MOTOR(&PI_MotorA,robot.MOTOR_A.Encoder,robot.MOTOR_A.Target);
	robot.MOTOR_B.Output = Incremental_MOTOR(&PI_MotorB,robot.MOTOR_B.Encoder,robot.MOTOR_B.Target);
	robot.MOTOR_C.Output = Incremental_MOTOR(&PI_MotorC,robot.MOTOR_C.Encoder,robot.MOTOR_C.Target);
	
	if( robot.type<=1 ) Set_Pwm(-robot.MOTOR_A.Output,-robot.MOTOR_B.Output, robot.MOTOR_C.Output,0,0);//High-spec omni chassis: MD36 motor
	else                Set_Pwm( robot.MOTOR_A.Output, robot.MOTOR_B.Output,-robot.MOTOR_C.Output,0,0);//Top-spec omni chassis: MD60 motor

	#endif
}

/**************************************************************************
Functionality: Non-Control Response Function - Executes when the robot encounters an error, prohibiting the robot's movement.
Input Parameters: Set the robot's stop mode: 0: Set to unclamp axes, allowing the robot to be freely pushed. 
                  1: Set to clamp axes, locking all actuators.
Return Value: None
Author: LUCSYS
Function: Non-control response function; this function will be executed when the robot encounters an error. Robot movement is prohibited
Input: robot stopping mode: 0-release shafts, allowing the robot to be pushed freely; 1-lock shafts, locking all actuators
Return value: none
Author: LUCSYS
**************************************************************************/
static void UnResponseControl(uint8_t mode)
{
	//Data validation
	if(mode!=UN_LOCK && mode!=LOCK) mode = UN_LOCK;
	
	//Disable with shaft release
	if(mode==UN_LOCK)
	{
		//Reset the PI controller to eliminate accumulation
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
		
		//Send 0 to stop the motor from turning
		Set_Pwm( 0 , 0 , 0 , 0 , 0 );
	}
	
	//Shaft-lock disable mode; chassis cannot be pushed
	else if( mode==LOCK )
	{
		Drive_Motor(0,0,0);
		ResponseControl();
	}
}

/**************************************************************************
Functionality: Send Control Values to Actuators - Distributes control values to each actuator (A, B, C, D wheels and servo).
Input Parameters: Control value for A wheel motor, control value for B wheel motor, control value for C wheel motor, 
                   control value for D wheel motor, control value for servo.
Return Value: None.
Author: LUCSYS
Function: Send control values to various actuators
Inlet parameters: Wheel A motor control quantity, wheel B motor control quantity, wheel C motor control quantity, D wheel motor control quantity, servo control quantity
Return value: none
Author: LUCSYS
**************************************************************************/
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)
{
	#if defined AKM_CAR || defined DIFF_CAR
	
		//Motor control
		if( m_a < 0 ) AIN1=1,AIN2=0;
		else          AIN1=0,AIN2=1;
		
		if(SysVal.HardWare_Ver==V1_0)
		{
			if( m_b > 0 ) V1_0_BIN1=1,V1_0_BIN2=0;
			else          V1_0_BIN1=0,V1_0_BIN2=1;
		}
		else if( SysVal.HardWare_Ver==V1_1 )
		{
			if( m_b > 0 ) BIN1=1,BIN2=0;
			else          BIN1=0,BIN2=1;
		}
		
		PWMA = abs(m_a);
		PWMB = abs(m_b);
	
		#if defined AKM_CAR
		//Servo control
		if( robot.type == 0 || robot.type == 1 || robot.type==9 )//High-spec Ackermann chassis
		{
			SERVO_SENIOR = servo;
		}
		else //Top-spec Ackermann chassis
		{
			SERVO_TOP = servo;
		}
		#endif
		
	#else 
		
		//Motor control
		if( m_a > 0 ) AIN1=0,AIN2=1;
		else          AIN1=1,AIN2=0;
		
		if(SysVal.HardWare_Ver==V1_0)
		{
			if( m_b > 0 ) V1_0_BIN1=0,V1_0_BIN2=1;
			else          V1_0_BIN1=1,V1_0_BIN2=0;
		}
		else if( SysVal.HardWare_Ver==V1_1 )
		{
			if( m_b > 0 ) BIN1=0,BIN2=1;
			else          BIN1=1,BIN2=0;
		}
	
		if( m_c > 0 ) CIN1=1,CIN2=0;
		else          CIN1=0,CIN2=1;
		
		if( m_d > 0 ) DIN1=1,DIN2=0;
		else          DIN1=0,DIN2=1;
		
		PWMA = abs(m_a);
		PWMB = abs(m_b);
		PWMC = abs(m_c);
		PWMD = abs(m_d);
	#endif
}

/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
Function: check battery voltage, enable switch state and software-disable flag
Entry parameters: voltage
Return value: whether control is allowed; 1-not allowed, 0-allowed
**************************************************************************/
static u8 Turn_Off(void)
{
    u8 temp = 0;
	static uint8_t saveflag = 0;
	
	//Voltage not meeting requirements: voltage below 20V and not in automatic recharge mode. (If automatic recharge is entered, voltage is ignored)
	if( robot.voltage < 20.0f && charger.AllowRecharge==0 ) temp = 1;

	//The emergency stop switch has been pressed
	if( EN==0 ) temp = 1;
	
	//Robot self-check error
	if( robot_check.errorflag==1 ) temp = 1;

	//The robot set the emergency stop via software
	if( robot_control.SoftWare_Stop==1 ) temp = 1;
	
	if( robot_control.control_arm==1 ) temp = 1;
	//The process of a car changing from disabled to enabled, clearing the target speed.
	if(temp==0 && saveflag==1)
	{
		robot_control.Vx = 0;
		robot_control.Vy = 0;
		robot_control.Vz = 0;
		robot_control.smooth_Vx = 0;
		robot_control.smooth_Vy = 0;
		robot_control.smooth_Vz = 0;
	}
	saveflag = temp;
	
	return temp;
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
            switch (appkey.DirectionFlag)
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
            switch (appkey.DirectionFlag)
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
        switch (appkey.DirectionFlag)
        {
            case 2:  joint_a+=step * step_multi;Flag_Move = 1; break;
            case 4:  gripper_close  = 1; break;
            case 6:  joint_a-=step * step_multi;Flag_Move = 1; break;
            case 8:  gripper_open   = 1; break;
            default: Flag_Move = 0; break;
        }
				if(gripper_open && A280.gripper>GRIPPER_LEVEL_MIN)
				{
					 A280.gripper--;
					 Gripper_SetLevel(A280.gripper);					
				}else if(gripper_close && A280.gripper<GRIPPER_LEVEL_MAX)
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
Function: Processes the command sent by APP through usart x
Input   : none
Output  : none
Function: Processes commands sent by the app via serial port x
Entry parameters: None
Return value: none
**************************************************************************/
static void Get_APPcmd(void)
{
	/*  APP joystick and speed direction
	     A
	     ^  +Vx       ^ +Vz
   H     |       B    ）
	     |
   G----------> -Vy  C
	     |
   F     |    D
	     E
	*/
	
	short m_sign = 1;         //Symbolic variables
	float base_vz= PI/4.0f;   //Z-axis speed reference
	
	//Non-omnidirectional chassis
	#if defined AKM_CAR || defined DIFF_CAR || defined _4WD_CAR
	
		#if defined AKM_CAR
			m_sign = -1;                  //Ackermann Vz represents front-wheel steering angle; some direction signs require adjustment.
			base_vz = angle_to_rad(30);
		#endif
	
	switch( appkey.DirectionFlag ) 
	{
		case 1: robot_control.Vx = robot_control.rc_speed , robot_control.Vz = 0               ; break; //A
		case 2: robot_control.Vx = robot_control.rc_speed , robot_control.Vz =-base_vz         ; break; //B
		case 3: robot_control.Vx = 0 ,                      robot_control.Vz =-base_vz         ; break; //C
		case 4: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz = (base_vz)*m_sign; break; //D
		case 5: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz = 0               ; break; //E
		case 6: robot_control.Vx =-robot_control.rc_speed , robot_control.Vz =-(base_vz)*m_sign; break; //F
		case 7: robot_control.Vx = 0 ,                      robot_control.Vz = base_vz         ; break; //G
		case 8: robot_control.Vx = robot_control.rc_speed , robot_control.Vz = base_vz         ; break; //H
		
		default : robot_control.Vx=0;robot_control.Vy=0;robot_control.Vz=0; break;
	}
	
	
	//Able to move the chassis in all directions
	#elif defined MEC_CAR || defined OMNI_CAR
	
	switch( appkey.DirectionFlag ) 
	{
		case 1: robot_control.Vx = robot_control.rc_speed , robot_control.Vy = 0                      ; break; //A
		case 2: robot_control.Vx = robot_control.rc_speed , robot_control.Vy =-robot_control.rc_speed ; break; //B
		case 3: robot_control.Vx = 0 ,                      robot_control.Vy =-robot_control.rc_speed ; break; //C
		case 4: robot_control.Vx =-robot_control.rc_speed , robot_control.Vy =-robot_control.rc_speed ; break; //D
		case 5: robot_control.Vx =-robot_control.rc_speed , robot_control.Vy = 0                      ; break; //E
		case 6: robot_control.Vx =-robot_control.rc_speed , robot_control.Vy = robot_control.rc_speed ; break; //F
		case 7: robot_control.Vx = 0 ,                      robot_control.Vy = robot_control.rc_speed ; break; //G
		case 8: robot_control.Vx = robot_control.rc_speed , robot_control.Vy = robot_control.rc_speed ; break; //H
		
		default : robot_control.Vx=0;robot_control.Vy=0;robot_control.Vz=0 , m_sign=0 ; break;
	}
	
	//If the cart does not have x/y axis commands, check the Z-axis commands
	if( m_sign==0 )
	{
		     if( appkey.TurnFlag==1 ) robot_control.Vz =  base_vz;
		else if( appkey.TurnFlag==2 ) robot_control.Vz = -base_vz;
		else                          robot_control.Vz = 0;
	}
	
	#endif

	//Unit conversion: mm/s - > m/s
	robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;
	robot_control.Vz = robot_control.Vz * ( robot_control.rc_speed/500.0f );//Z-axis speed is adjusted according to the remote control speed.

    //Control target value is obtained and kinematics analysis is performed
    //Get control target values for kinematic analysis
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
Function: Processes remote control commands for model aircraft
Entry parameters: None
Return value: none
**************************************************************************/
static void Remote_Control(void)
{
    //Data within 1 second after entering the model control mode will not be processed
    //Data within 1 second after entering model aircraft control mode will not be processed
    static u8 thrice=BALANCE_TASK_RATE;
    int Yuzhi=100; //Threshold to ignore small movements of the joystick // Threshold, ignore small movements of the joystick

    //limiter // Limit
    int LX,LY,RY,RX,Remote_RCvelocity;
	float base_vz = PI/4.0f ;//Z-axis speed reference
	
	//Ackermann Z represents front-left wheel angle; set reference to 30 degrees
	#if defined AKM_CAR
		base_vz = angle_to_rad(30.0f);
	#endif
	
	//Four channels with raw value limits
	remoter.ch1 = target_limit_int(remoter.ch1,1000,2000);
	remoter.ch2 = target_limit_int(remoter.ch2,1000,2000);
	remoter.ch3 = target_limit_int(remoter.ch3,1000,2000);
	remoter.ch4 = target_limit_int(remoter.ch4,1000,2000);

    //Front and back direction of left rocker. Control forward and backward.
    //Left stick forward and backward direction. Controls forward and backward movement.
    LX=remoter.ch2-1500;
    
	//The left joystick's horizontal directions control lateral \
	  movement for omnidirectional mobile vehicles.
    //Left joystick moves left and right; for omnidirectional trolleys, control lateral movement left and right
    LY=remoter.ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
    //Right stick forward and backward. Throttle/acceleration/deceleration.
    RX=remoter.ch3-1500;
	
    //Right stick left and right. To control the rotation.
    //Right stick left and right direction. Controls rotation.
    RY=-(remoter.ch1-1500);//Rotation

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	//Throttle adjustment speed Remote_RCvelocity value: 0~1000
    Remote_RCvelocity= robot_control.rc_speed + RX;
    if(Remote_RCvelocity<0)Remote_RCvelocity=0;

    //The remote control command of model aircraft is processed
    //Processing remote control commands for model aircraft
	
	robot_control.Vx = LX * (float)Remote_RCvelocity/500.0f;
	robot_control.Vy =-LY * (float)Remote_RCvelocity/500.0f;
	
	//Reference: base_vz, Own multiplier: RY/500, Value: [-1~1], Throttle multiplier: (Remote_RCvelocity/500.0f), Value: [0,2]
	robot_control.Vz = base_vz*((float)RY/500.0f) * ((float)Remote_RCvelocity/500.0f) ; 

	//For non-Ackermann chassis, reverse steering direction when moving backward to match joystick operation.
	#if !defined AKM_CAR
	if(  robot_control.Vx < 0 ) robot_control.Vz = -robot_control.Vz;
	#endif

    //Unit conversion, mm/s -> m/s
    //Unit conversion, mm/s - > m/s
	robot_control.Vx = robot_control.Vx/1000.0f;
	robot_control.Vy = robot_control.Vy/1000.0f;

    //Data within 1 second after entering the model control mode will not be processed
    //Data within 1 second after entering model aircraft control mode will not be processed
    if(thrice>0) robot_control.Vx=0,robot_control.Vy=0,robot_control.Vz=0,thrice--;

    //Control target value is obtained and kinematics analysis is performed
    //Get control target values for kinematic analysis
    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
Function: Handles PS2 controller control commands
Entry parameters: None
Return value: none
**************************************************************************/
#include "xbox360_gamepad.h"
#include "WiredPS2_gamepad.h"
//Xbox 360 gamepad key callback function
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	static float step=0.02;
	//Press the Start button
	if( keyid == Xbox360KEY_Menu && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	if( gamepad_brand == Xbox360 )
	{
		//Handle acceleration and deceleration
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			robot_control.rc_speed -= 50;
		else if( keyid == Xbox360KEY_RB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			robot_control.rc_speed += 50;
		
		if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
		
		if( keyid == Xbox360KEY_A )
		{
			robot_control.control_arm=!robot_control.control_arm;
		}
		
		if( keyid==Xbox360KEY_LEFT)
		{
			if(A280.gripper>GRIPPER_LEVEL_MIN)A280.gripper-=10;
			Gripper_SetLevel(A280.gripper);
		}
		else if(keyid==Xbox360KEY_RIGHT)
		{
			if(A280.gripper<GRIPPER_LEVEL_MAX)A280.gripper+=10;
			Gripper_SetLevel(A280.gripper);
		}
	}
	else if(  gamepad_brand == PS2_USB_Wiredless )
	{
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			robot_control.rc_speed += 50;
		else if( keyid == Xbox360_PaddingBit && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK ) )
			robot_control.rc_speed -= 50;
		if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
		
		if( keyid == Xbox360KEY_A )
		{
			robot_control.control_arm=!robot_control.control_arm;
		}

	}
	
	
	//Activate and cancel vibration
	if( keyid == Xbox360KEY_SELECT && event == GamePadKeyEvent_LONGCLICK )
	{
		if( GamePadInterface->Vib_EN )
		{
			GamePadInterface->SetVibration(0,127);
			vTaskDelay(50);
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
		}
		else
		{
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
			vTaskDelay(50);
			GamePadInterface->SetVibration(0,127);
		}	
	}
}

//Wired USB handle callback function
void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//Press the Start button
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	//Handle acceleration and deceleration
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed += 50;
	if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
}

//Classic PS2 controller callback function, non-USB version
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//Press the Start button
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	//Handle acceleration and deceleration
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		robot_control.rc_speed += 50;
	
	if( robot_control.rc_speed < 0 ) robot_control.rc_speed = 0;
}


//Handle vibration mapping function
static uint8_t map_to_vib(float x) {
    // Check the input range, limited to [0.2, 1.2].
    if (x < 0.1f) return 0;
    if (x > 1.2f) x = 1.2f;

    // Linear mapping
    float result = 255.0f * (x - 0.1f) / 1.1f;

    // Round and convert to uint8_t
    return (uint8_t)(result + 0.5f);
}


static float mapStickToAngleChange(float stickValue) {
    // Base sensitivity
    const float baseSensitivity = 0.0001f;
    
    // Calculate nonlinear factors – the larger the joystick value, the greater the variation
    float factor = 1.0f + (fabsf(stickValue) / 50.0f);
    
    // Calculate the angular change
    return stickValue * baseSensitivity * factor;
}

static float mapStickToPositionChange(float stickValue) {
    // Base sensitivity
    const float baseSensitivity = 0.00001f;
    
    // Calculate nonlinear factors – the larger the joystick value, the greater the variation
    float factor = 1.0f + (fabsf(stickValue) / 50.0f);
    
    // Calculate the angular change
    return stickValue * baseSensitivity * factor;
}

static void updateTargetAngle(float LX, float LY, float RX, float RY) {
	  uint8_t xy_move_flag=0;
	  uint8_t a_move_flag=0;
	  float new_a,new_x,new_y;
    new_x=A280.ee_x;
	  new_y=A280.ee_y;
	  if(RX!=0)
		{
			 new_a=A280.joint_rad[0];
			 float angleA = mapStickToAngleChange(RX);
			 new_a += angleA;
			 a_move_flag=1;
		}

    
//	  float angleD = mapStickToAngleChange(RY);
	  
	  if(LX!=0)
		{
			 float position_x = mapStickToPositionChange(-LX);
			 new_x+=position_x;
			 xy_move_flag=1;
		}
		if(LY!=0)
		{
			
			float position_y = mapStickToPositionChange(LY);
			new_y+=position_y;
			xy_move_flag=1;
		}
		
    
	  if(a_move_flag && new_a>A280.joint_min[0] && new_a<A280.joint_max[0])
		{
		
				A280.joint_rad[0]=new_a;
				UpdateMotorTargets_FromJoints(&A280,1);
		}	
    if(xy_move_flag && Arm2DOF_SetTargetEndEffector(&A280,new_x,new_y))
		{
				UpdateMotorTargets_FromJoints(&A280,23);
		}
	 
}

static void PS2_control(void)
{
	float LX=127,LY=127,RX=127,RY=127;
	float ThrottleTri = 255;
	float base_vz = PI/4.0f ;//Z-axis speed reference
	
	//Forward joystick
	LY = GamePadInterface->LY - 127;
	
	//Horizontal movement left and right
	LX = 127 - GamePadInterface->LX;
	
	//Clockwise/counterclockwise
	RX = 127 - GamePadInterface->RX;
	
	RY = GamePadInterface->RY - 127;
	//Joystick with a small amplitude filter
	if( fabs(LY)<20 ) LY = 0;
	if( fabs(LX)<20 ) LX = 0;
	if( fabs(RX)<20 ) RX = 0;
	
	//For Xbox360 gamepads with analog triggers, allow trigger-based control
	if( gamepad_brand == Xbox360 )
	{
		//When the forward joystick has no value, use the trigger value
		if( (int)LY == 0 )
		{
			if( GamePadInterface->LT == 0 && GamePadInterface->RT != 0 )
				ThrottleTri =  GamePadInterface->RT, LY = 127;
			else if( GamePadInterface->LT != 0 && GamePadInterface->RT == 0 )
				ThrottleTri =  -GamePadInterface->LT,LY = 127;
			else
				ThrottleTri = 0;
		}
	}
	
	//Joystick value mapping in non-analog mode for USB wired gamepad
	else if( gamepad_brand == PS2_USB_Wired ||  gamepad_brand == PS2_USB_WiredV2 )
	{
		if( fabs(RX)<0.0001f )
		{
			if( GamePadInterface->getKeyState(PS2KEY_4PINK) )
				RX = 127;
			else if( GamePadInterface->getKeyState(PS2KEY_2RED) )
				RX = -127;
		}
	}
	if(robot_control.control_arm){
		  updateTargetAngle(LX,LY,RX,RY);
	}else{
			robot_control.Vx = (LY/127.0f) * robot_control.rc_speed * (ThrottleTri/255.0f);
			robot_control.Vy = (LX/127.0f) * robot_control.rc_speed;
			robot_control.Vz = base_vz * (RX/127.0f) * ( robot_control.rc_speed/500.0f );
			
			#if !defined AKM_CAR
				if( robot_control.Vx<0 ) robot_control.Vz = -robot_control.Vz;
			#endif

			//Unit conversion, mm/s -> m/s
			//Unit conversion, mm/s - > m/s
			robot_control.Vx = robot_control.Vx/1000.0f;
			robot_control.Vy = robot_control.Vy/1000.0f;

			//Control target value is obtained and kinematics analysis is performed
			//Get control target values for kinematic analysis
			Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
	}
	
	//Vibration is transmitted to the handle based on acceleration response
	#include "bsp_gamepad.h"
	
	//Z-axis data to determine vibration intensity
	float now_z = imu.accel.z/1671.84f;
	static float last_z = 0;
	float strength = fabs(last_z - now_z);
	
	#if defined MEC_CAR || defined OMNI_CAR
	const float vib_strength = 0.6f;
	#else
	const float vib_strength = 0.1f;
	#endif
	
	//Map vibration to gamepad
	if( strength>vib_strength && SysVal.Time_count>CONTROL_DELAY)
	{
		if( GamePadInterface->SetVibration!=NULL )
			GamePadInterface->SetVibration(map_to_vib(strength),0);
	}
	last_z = now_z;
		
//	//TODO: Adapted for PS2 controls
//    float LX,LY,RY;
//    int Yuzhi=20; // Threshold to ignore small joystick movements
//	
//	float base_vz = PI/4.0f; // Z-axis speed reference
//	
//	//Digital-mode control conversion for USB wired PS2 gamepad
//	if( SysVal.HardWare_Ver==V1_1 )
//	{
//		if( ps2_type == Normal_PS2 )
//		{
//			if( ps2_val.RX==128 && ps2_val.RY==128 )
//			{
//				if( Read_PS2_KEY(R_4PINK )==PS2_KEY_ON )  ps2_val.RX  = 0;
//				if( Read_PS2_KEY(R_2RED  )==PS2_KEY_ON )  ps2_val.RX = 255;
//				if( Read_PS2_KEY(R_1GREEN)==PS2_KEY_ON )  ps2_val.RY = 0;
//				if( Read_PS2_KEY(R_3BLUE )==PS2_KEY_ON )  ps2_val.RY = 255;
// 			}
//		}
//	}
//	
//    //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
//    //128 is the median. The PS2 coordinate system and the ROS coordinate system define X and Y differently
//    LY=-(ps2_val.LX-128); // value range [-128,128]
//    LX=-(ps2_val.LY-128);
//    RY=-(ps2_val.RX-128);

//    //Ignore small movements of the joystick
//    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
//    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
//    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
//	
//	static uint8_t low_fre = 0;
//	if(++low_fre==10) // Reduce the frequency of key scans
//	{
//		low_fre = 0;
//		//PS2 controller: left trigger 1 accelerates, 2 slows down
//			 if( Read_PS2_KEY(L1_KEY)==PS2_KEY_ON ) robot_control.rc_speed+=20;
//		else if( Read_PS2_KEY(L2_KEY)==PS2_KEY_ON ) robot_control.rc_speed-=20;
//		
//		//Maximum/minimum speed limits
//		robot_control.rc_speed = target_limit_float(robot_control.rc_speed,0,robot_control.limt_max_speed*1000);
//	}

//	#if defined AKM_CAR
//		base_vz = angle_to_rad(30.0f); Ackermann Z-axis reference angle
//	#endif
//	
//    //Handle PS2 controller control commands
//    //Handle PS2 controller control commands
//	robot_control.Vx = (LX/128.0f) * robot_control.rc_speed;
//	robot_control.Vy = (LY/128.0f) * robot_control.rc_speed;
//	robot_control.Vz = base_vz * (RY/128.0f) * ( robot_control.rc_speed/500.0f );
//	
//	#if !defined AKM_CAR
//		if( robot_control.Vx<0 ) robot_control.Vz = -robot_control.Vz;
//	#endif

//    //Unit conversion, mm/s -> m/s
//    //Unit conversion, mm/s - > m/s
//    robot_control.Vx = robot_control.Vx/1000.0f;
//	robot_control.Vy = robot_control.Vy/1000.0f;

//    //Control target value is obtained and kinematics analysis is performed
//    //Get control target values for kinematic analysis
//    Drive_Motor(robot_control.Vx,robot_control.Vy,robot_control.Vz);
}

/**************************************************************************
Functionality: Robot Self-Check Function - Performs a self-check on the robot to verify if there are any errors in the wiring of motors, encoders, and drivers.
Input Parameters: None.
Return Value: None.
Author: LUCSYS
Function: Robot self-check function to check whether the robot motor, encoder, or driver has incorrect wiring
Entry parameters: None
Return value: none
Author: LUCSYS
**************************************************************************/
static void robot_mode_check(void)
{
	//TODO: Self-check procedure, faulty fault of the same motor is hard to judge.
	
	#define OVERLOAD_PWM 8000 //Overload PWM warning value
	#define ENCODER_LOST_PWM 5500 //PWM error threshold when encoder is not connected
	
    static u8 init=0;//Initialize
    if( 0 == init ) 
	{
		ROBOT_SELFCHECK_t_Init(&robot_check);
		init=1;
	}

    if( EN==1 && robot_check.errorflag ==0 ) //Retain the ability to skip self-test using the emergency stop switch
    {
		#if defined OMNI_CAR //The all-directional wheel chassis has a unique structure and independent self-check logic
			
			//1. During the omnidirectional wheel self-test, only motors B and C rotate. If there are values in line A, it indicates the driver or encoder wiring is connected incorrectly (/4 represents a quarter turn).
			if( robot_check.check_a > robot.HardwareParam.Encoder_precision/4 || \
				robot_check.check_a < -robot.HardwareParam.Encoder_precision/4	)
			{
				Buzzer_AddTask(1,20);
				robot_check.errorflag = 1;
			}
			
			//2. Motor reversal error caused by incorrect model or wiring
			if( robot_check.check_b > robot.HardwareParam.Encoder_precision/4 || \
				robot_check.check_c < -robot.HardwareParam.Encoder_precision/4 )
			{
				Buzzer_AddTask(2,20);
				robot_check.errorflag = 1;
			}
			
			//3. If any motor's PWM output reaches a threshold and the corresponding encoder data is very small (the wheel rotates less than 12°) or no data, it is considered that the encoder wire is not connected
			if( ( fabs( robot.MOTOR_B.Output ) > ENCODER_LOST_PWM && robot_check.check_b > -robot.HardwareParam.Encoder_precision/30 ) || \
				( fabs( robot.MOTOR_C.Output ) > ENCODER_LOST_PWM && robot_check.check_c < robot.HardwareParam.Encoder_precision/30 ) )
			{
				Buzzer_AddTask(3,20);
				robot_check.errorflag = 1;
			}
			
			//4. If the PWM output of any motor exceeds a greater threshold, it is considered overloaded
			if( fabs( robot.MOTOR_A.Output ) > OVERLOAD_PWM || fabs( robot.MOTOR_B.Output ) > OVERLOAD_PWM || \
				fabs( robot.MOTOR_C.Output ) > OVERLOAD_PWM )
			{
				Buzzer_AddTask(4,20);
				robot_check.errorflag = 1;
			}
			
		#else
			static int last_a=0,last_b=0,last_c=0,last_d=0;
			static u8 err_time = 0;
			
			//1. Any wheel reversed for 1/4 turn is considered to be due to the encoder or drive wire being reversed, or the wrong model selection.
			if( robot_check.check_a < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_b < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_c < -robot.HardwareParam.Encoder_precision/4  || \
				robot_check.check_d < -robot.HardwareParam.Encoder_precision/4  ) 
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(1,20);
			}
			
			//2. If any motor's PWM output reaches the threshold and the corresponding encoder data is very small (the wheel rotates less than 12°) or no data, it is considered that the encoder wire is not connected
			if( ( abs( robot.MOTOR_A.Output ) > ENCODER_LOST_PWM && robot_check.check_a < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_B.Output ) > ENCODER_LOST_PWM && robot_check.check_b < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_C.Output ) > ENCODER_LOST_PWM && robot_check.check_c < robot.HardwareParam.Encoder_precision/30 ) || \
				( abs( robot.MOTOR_D.Output ) > ENCODER_LOST_PWM && robot_check.check_d < robot.HardwareParam.Encoder_precision/30 )  )
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(2,20);
			}
			
			//3. If the PWM output of any motor exceeds a greater threshold, it is considered overloaded
			if( abs( robot.MOTOR_A.Output ) > OVERLOAD_PWM || abs( robot.MOTOR_B.Output ) > OVERLOAD_PWM || \
				abs( robot.MOTOR_C.Output ) > OVERLOAD_PWM || abs( robot.MOTOR_D.Output ) > OVERLOAD_PWM	)
			{
				robot_check.errorflag = 1;
				Buzzer_AddTask(3,20);
			}
			
			//4. Multiple cumulative value regressions occur, besides the abnormal in 1, it may also be a result of multiple errors cumulating together.
			if( abs(robot_check.check_a) < abs(last_a) - 50 || abs(robot_check.check_b) < abs(last_b) - 50 || abs(robot_check.check_c) < abs(last_c) - 50|| abs(robot_check.check_d) < abs(last_d) - 50 )
			{
				err_time++;
				if( err_time > 20 )  robot_check.errorflag = 1 , Buzzer_AddTask(4,20);	
			}
		
			last_a = robot_check.check_a;
			last_b = robot_check.check_b;
			last_c = robot_check.check_c;
			last_d = robot_check.check_d;
		#endif
    }
}

/**************************************************************************
Functionality: Read Feedback Values - Retrieves feedback values from each actuator of the robot.
Input Parameters: None.
Return Value: None.
Author: LUCSYS
Function: reads feedback values from various actuators of the machine
Entry parameters: None
Return value: none
Author: LUCSYS
**************************************************************************/
static void Get_Robot_FeedBack(void)
{
    //Retrieves the original data of the encoder
    //Retrieve the original data of the encoder
    short Encoder_A_pr,Encoder_B_pr;
	
	//Original encoder value, cleared after reading.
    Encoder_A_pr = Read_Encoder(Encoder_A);
    Encoder_B_pr = Read_Encoder(Encoder_B);
	
	#if !defined AKM_CAR && !defined DIFF_CAR
		short Encoder_C_pr;
		Encoder_C_pr = Read_Encoder(Encoder_C);
	#endif
	
	#if defined MEC_CAR || defined _4WD_CAR
		short Encoder_D_pr;
		Encoder_D_pr = Read_Encoder(Encoder_D);
	#endif
	
	//Calculate the correction factor
	float LeftWheelDiff = wheelCoefficient(robot_control.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(robot_control.LineDiffParam,0);
	
    //The encoder converts the raw data to wheel speed in m/s
    //The encoder's raw data is converted into wheel speed, unit m/s
	#if defined AKM_CAR
	
		//When the self-test is not completed, encoder data is collected for judgment
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a +=  Encoder_A_pr;
			robot_check.check_b += -Encoder_B_pr;
		}
		
		//Converts the raw encoder readings of both motors into m/s
		robot.MOTOR_A.Encoder =   Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
	
        //Gets the position of the slide, representing the front wheel rotation Angle
        //Obtain the position of the slide rail, representing the front wheel rotation angle. This value is collected by ADC2, and the DMA completes the handling; only the data needs to be processed here
		robot.SERVO.Encoder = get_DMA_SlideRes();
		robot.SERVO.Encoder = Slide_Mean_Filter(robot.SERVO.Encoder);//Smooth filter the collected results
	
	#elif defined DIFF_CAR
		//When the self-test is not completed, encoder data is collected for judgment
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a +=  Encoder_A_pr;
			robot_check.check_b += -Encoder_B_pr;
		}
		
		robot.MOTOR_A.Encoder =   Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;	
		
	#elif defined MEC_CAR || defined _4WD_CAR	
		//When the self-test is not completed, encoder data is collected for judgment
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a +=  Encoder_A_pr;
			robot_check.check_b +=  Encoder_B_pr;
			robot_check.check_c += -Encoder_C_pr;
			robot_check.check_d += -Encoder_D_pr;
		}
		robot.MOTOR_A.Encoder =   Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_B.Encoder =   Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_C.Encoder = - Encoder_C_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;
		robot.MOTOR_D.Encoder = - Encoder_D_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;	
		
	#elif defined OMNI_CAR	
		//When the self-test is not completed, encoder data is collected for judgment
		if( robot_check.check_end == 0 )
		{
			robot_check.check_a += -Encoder_A_pr;
			robot_check.check_b += -Encoder_B_pr;
			robot_check.check_c += -Encoder_C_pr;
		}
		
		robot.MOTOR_A.Encoder = - Encoder_A_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ;
		robot.MOTOR_B.Encoder = - Encoder_B_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / LeftWheelDiff;
		robot.MOTOR_C.Encoder = - Encoder_C_pr*BALANCE_TASK_RATE/( robot.HardwareParam.Encoder_precision )* robot.HardwareParam.Wheel_Circ / RightWheelDiff;	
		
	#endif
}

/*-------------------------------- Robot state detection related functions ------------------------------------*/
/*-------------------------------- Robot Status Detection Related Functions ------------------------------------*/
/**************************************************************************
Function Description: The function clears residual motor control signals when the robot \
  remains stationary on a non-sloping surface for a certain period of time. This helps  \
  reduce power consumption and motor noise of the robot.
Input Parameters: None
Return Value: None
Author: LUCSYS
Function: When the chassis remains stationary on a non-slope for more than a certain time, it removes residual motor control. This reduces robot power consumption and motor noise
Entry parameters: None
Return value: none
Author: LUCSYS
**************************************************************************/
static void Robot_ParkingCheck(void)
{
	#define PUSH_PWM 500 //Accumulated PWM threshold for carts; exceeding it will be cleared
	
    //Simple assessment of the car's posture
    float y_accle = (float)(imu.accel.y/1671.84f);//Actual value of Y-axis acceleration
    float z_accle = (float)(imu.accel.z/1671.84f);//Actual Z-axis acceleration
    float diff;

    //Calculate the fusion of Y and Z accelerations; the closer this value is to 9.8, the more horizontal the car's attitude is
    if( y_accle > 0 ) diff  = z_accle - y_accle;
    else diff  = z_accle + y_accle;
	
	if( robot.MOTOR_A.Target != 0 || robot.MOTOR_B.Target!=0 || \
		robot.MOTOR_C.Target != 0 || robot.MOTOR_D.Target!=0)
	{   //The car is under control, and the waiting flag position is set. Reset other detection variables
		park.wait_check = 1;  //Wait for detection
		park.timecore = 0;    //Reset and detect relevant variables
		park.clear_state = 0;
		park.start_clear = 0;
	}
	else 
	{
		if( park.wait_check==1 ) //Detect transition from nonzero velocity to 0
		{
			park.timecore++;
			if( park.timecore >= BALANCE_TASK_RATE*5 ) //The car has been stationary for more than 5 seconds
			{
				park.wait_check=0;
				park.timecore = 0;
				
				//If the robot is not on an incline, enter parking mode to clear motor control
				if( diff > 8.8f ) park.start_clear = 1 , park.clear_state = 0;
				else  park.clear_state |= 1<<7 ; //On the slope, control quantities cannot be cleared. Mark clearing completed
			}
		}
	}
	
	//Check whether the clearance has been completed. After clearing is complete, it will monitor whether the cart passively accumulates control (pushing behavior).
	if( ((park.clear_state>>7)&0x01)==1  )
	{
		if(diff > 8.8f )//The car is not on the slope as a prerequisite.
		{
			//Pushing the cart occurs
			if( abs(robot.MOTOR_A.Output) > PUSH_PWM || abs(robot.MOTOR_B.Output) > PUSH_PWM || \
				abs(robot.MOTOR_C.Output) > PUSH_PWM || abs(robot.MOTOR_D.Output) > PUSH_PWM)
			{
				park.timecore++;
				if( park.timecore >= BALANCE_TASK_RATE*10 ) //The car's PWM accumulated time exceeds 10 seconds
				{
					park.timecore = 0;
					//Clear control output on entering parking mode
					park.start_clear = 1 , park.clear_state = 0;
				}
			}
		}
	}
	
	//Activate parking mode to perform the cleanup task
	if( park.start_clear==1 )
	{
		if( 1 == PI_Clear_Output(&PI_MotorA) ) park.clear_state |= 1<<0 ;
		else                                   park.clear_state &= ~(1<<0);
		
		if( 1 == PI_Clear_Output(&PI_MotorB) ) park.clear_state |= 1<<1 ;
		else                                   park.clear_state &= ~(1<<1);
		
		if( 1 == PI_Clear_Output(&PI_MotorC) ) park.clear_state |= 1<<2 ;
		else                                   park.clear_state &= ~(1<<2);
		
		if( 1 == PI_Clear_Output(&PI_MotorD) ) park.clear_state |= 1<<3 ;
		else                                   park.clear_state &= ~(1<<3);
		
		//Four motors are cleared
		if( (park.clear_state&0xff)==0x0f ) 
		{
			park.start_clear = 0;    //Clearing task completed
			park.clear_state |= 1<<7;//Marker clearing completed
		}
	}
}

/**************************************************************************
Function function: Scan user buttons and perform different functions through different button states
Entry parameter: Frequency of executing scanning tasks
Return value: None
Author: SHEELTEC
Function: Scan user keys and perform different functions through different key states
Entry parameters: frequency of tasks performing scans
Return value: none
Author: LUCSYS
**************************************************************************/
static void UserKey_Scan(u16 rate)
{
	u8 state;
	
	//User key scanning on the board
	state = KEY_Scan(rate,0);
	if(state==single_click) //Click
	{
		charger.AllowRecharge = ! charger.AllowRecharge;
	}
	else if( state==double_click )//Double-click to recalibrate IMU values
	{	
		ImuData_copy(&imu.Deviation_gyro,&imu.Original_gyro);
		ImuData_copy(&imu.Deviation_accel,&imu.Original_accel);
	}
	
	else if( state==long_click ) //Long press
	{
		oled.page++;
		if( oled.page > oled.MAX_PAGE ) oled.page = 1;
	}
	
}

/**************************************************************************
Functionality: Check Automatic Recharge Station Status - Verifies the status of the automatic recharge station.
Input Parameters: None.
Return Value: None.
Author: LUCSYS
Function: Checks the status of automatic recharge equipment
Entry parameters: None
Return value: none
Author: LUCSYS
**************************************************************************/
static void Charger_DevCheck(void)
{
	//The user settings enter automatic recharge mode. At this point, first confirm whether the device exists on the system
	if( charger.AllowRecharge==1 )
		if( SysVal.HardWare_charger==0 ) charger.AllowRecharge=0,Find_Charging_HardWare();//No charging hardware is present; secondary confirmation is performed by accessing the characteristic code
	
	//When the device exists on the system, perform hardware offline monitoring of automatic recharge
	if(SysVal.HardWare_charger==1)
	{   //Recharge equipment hardware is offline; perform software reset of recharge equipment
		charger.OutLine_Check++;
		if( charger.OutLine_Check>RATE_100_HZ) auto_recharge_reset() , charger.OutLine_Check=BALANCE_TASK_RATE+1;//Use the owning task's frequency; here it is BALANCE_TASK_RATE
		
		//When an automatic recharge device is present, if the car detects low battery and receives infrared signals from the charging pile, it will automatically activate the automatic recharge function
		static u8 once=1;
		if( robot.voltage < 20.0f && charger.RED_STATE!=0 && once)
		{
			once = 0;
			charger.AllowRecharge = 1;
		}
	}
	
	//Remote control signal detection for model aircraft
	remoter.check_count++;//This variable is set to 0 when receiving the model aircraft throttle input
	if( remoter.check_count > Filter_Threshold )//Exceeding the set threshold indicates no model aircraft signal or interference signal
	{
		remoter.check_count = Filter_Threshold + 1;
		remoter.ch1 = 1500;remoter.ch2=1500;remoter.ch3=1000;remoter.ch4=1500;//Interference or no signal, all channel values are reset
	}
}

/*-------------------------------- Function auxiliary type related functions ------------------------------------*/
/*-------------------------------- Function Helper Class Related Functions ------------------------------------*/

/**************************************************************************
Function function: Save the steering variables of the Ackermann model's servo to Flash
Entry parameter: Flag bit for executing save
Return value: 1: Successfully saved 0: Not saved or failed to save
Author: LUCSYS
Function: Save Ackermann vehicle steering variables to Flash
Entry parameter: whether to execute the saved flag
Return value: 1-save succeeded; 0-no save required; >1-flash write error
Author: LUCSYS
**************************************************************************/
static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check=0;
	
	if(*flag==1)
	{
		*flag = 0;
		check = 1;
		taskENTER_CRITICAL();//Operate FLash to enter critical state to ensure data security
		int buf[7]={0};
		buf[0] = Akm_Servo.Min;
		buf[1] = Akm_Servo.Mid;
		buf[2] = Akm_Servo.Max;
		buf[3] = robot_control.LineDiffParam;
		buf[4] = *((int32_t*)&robot_control.rc_speed);
		buf[5] = robot.V_KP;
		buf[6] = robot.V_KI;
		check += Write_Flash( (u32*)buf , 7);
		
		taskEXIT_CRITICAL();//Exit critical section
		//If all writes succeed, check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0);//Read data with index 0
	if( read!=0xffffffff ) Akm_Servo.Min = read;
	
	read = Read_Flash(1);//Read data with subscript 1
	if( read!=0xffffffff ) Akm_Servo.Mid = read;
	
	read = Read_Flash(2);//Read data with subscript 2
	if( read!=0xffffffff ) Akm_Servo.Max = read;
	
	read = Read_Flash(3); //Correction factor
	if( read!=0xffffffff ) robot_control.LineDiffParam = read;
	
	
	read = Read_Flash(4);//Velocity
	if( read!=0xffffffff ) robot_control.rc_speed = *((float*)&read);
	if( robot_control.rc_speed < 0 || robot_control.rc_speed > 10000 )//Abnormal speed data filtering
		robot_control.rc_speed = 500;
	
	
	read = Read_Flash(5);//KP parameters
	if( read!=0xffffffff ) 
	{
		Set_Robot_PI_Param(read,-1);
	}
	
	read = Read_Flash(6);//KI parameter
	if( read!=0xffffffff ) 
	{
		Set_Robot_PI_Param(-1,read);
	}
}

//static uint8_t Akm_SaveServo_Param(uint8_t *flag)
//{
//	u8 check=0;
//	
//	if(*flag==1)
//	{
//		*flag = 0;
//		check = 1;
//		taskENTER_CRITICAL();// operate Flash entering critical section, ensure data safety
//		int buf[4]={Akm_Servo.Min,Akm_Servo.Mid,Akm_Servo.Max,robot_control.LineDiffParam};
//		check += Write_Flash( (u32*)buf , 4);
//		taskEXIT_CRITICAL();// exit critical section
//		//If all writes succeed, check==1
//	}

//	return check;
//}

//void Akm_ReadServo_Param(void)
//{
//	int read;
//	read = Read_Flash(0); Read data with index 0
//	if( read!=0xffffffff ) Akm_Servo.Min = read;
//	
//	read = Read_Flash(1); Read data with subscript 1
//	if( read!=0xffffffff ) Akm_Servo.Mid = read;
//	
//	read = Read_Flash(2); Read data with subscript 2
//	if( read!=0xffffffff ) Akm_Servo.Max = read;
//	
//	read = Read_Flash(3); Correction coefficient
//	if( read!=0xffffffff ) robot_control.LineDiffParam = read;
//	
//}

/**************************************************************************
Functionality: Velocity Smoothing Function - Gradually adjusts the speed to the target speed using a specified step value.
Input Parameters: Current speed, target speed, smoothing step value.
Return Value: Smoothed speed.
Author: LUCSYS
Function: Speed smoothing function to ensure speed reaches the target speed at a set step value
Entry parameters: current speed, target speed, smooth step values
Return value: smoothed velocity
Author: LUCSYS
**************************************************************************/
static float Vel_SmoothControl(float now_speed,float targetSpeed,float step)
{
	if( now_speed > targetSpeed )
	{
		now_speed -= step;
		if( now_speed<=targetSpeed ) now_speed = targetSpeed;
	}
	else
	{
		now_speed += step;
		if( now_speed>=targetSpeed ) now_speed = targetSpeed;
	}

	return now_speed;
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

static int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

/**************************************************************************
Function: Data sliding filtering
Input   : data
Output  : Filtered data
Function: Ackermann slide track data filtering
Entry parameters: newly collected data
Return value: filtered data
Author: LUCSYS
**************************************************************************/
#if defined AKM_CAR
#define FILTERING_TIMES 20
static int Slide_Mean_Filter(int data)
{
    u8 i;
    s32 Sum_Speed = 0;
    s16 Filter_Speed;
    static  s16 Speed_Buf[FILTERING_TIMES]= {0};
    for(i = 1 ; i<FILTERING_TIMES; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[FILTERING_TIMES - 1] =data;

    for(i = 0 ; i < FILTERING_TIMES; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
    return Filter_Speed;
}
#endif

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
//    int threshold=7;
//    if     (targetPWM>currentPWM+threshold) currentPWM+=step;
//    else if(targetPWM<currentPWM-threshold) currentPWM-=step;
//    else                                    currentPWM =targetPWM;

//    return currentPWM;
//}


/**************************************************************************
Functionality: Radian to Degree Conversion - Converts a given radian value to its corresponding degree value.
Input Parameters: Radian value.
Return Value: Degree value corresponding to the input radian.
Author: LUCSYS
Function: Radians to angles
Entry parameter: radian
Return value: angle in degrees corresponding to input radians
Author: LUCSYS
**************************************************************************/
float rad_to_angle(const float rad)
{
    return rad/PI*180.0f;
}

/**************************************************************************
Functionality: Degree to Radian Conversion - Converts a given degree value to its corresponding radian value.
Input Parameters: Degree value.
Return Value: Radian value corresponding to the input degree.
Author: LUCSYS
Function: angle to radian
Entry parameter: Angle
Return value: radians corresponding to input degrees
Author: LUCSYS
**************************************************************************/
float angle_to_rad(const float angle)
{
    return angle/180.0f*PI;
}


/*-------------------------------- Software initialization related functions ------------------------------------*/
/*-------------------------------- Software initialization-related functions ------------------------------------*/
/**************************************************************************
Functionality: Incremental PI Controller Initialization - Initializes an incremental PI 
               (Proportional-Integral) controller with specified parameters.
Input Parameters: PI controller, kp value (proportional gain), ki value (integral gain).
Return Value: None.
Author: LUCSYS
Function: Incremental PI controller initialization
Entry parameters: PI controller, kp value, ki value
Return value: none
Author: LUCSYS
**************************************************************************/
void PI_Controller_Init(PI_CONTROLLER* p,int kp,int ki)
{
	p->Bias = 0;
	p->LastBias = 0;
	p->Output = 0;
	p->kp = kp;
	p->ki = ki;
}

//Initialize the robot control related variables
void ROBOT_CONTROL_t_Init(ROBOT_CONTROL_t* p)
{
	//Adjustable parameters
	p->limt_max_speed = 3.5f;    //The robot limits the maximum movement speed in m/s
	p->rc_speed = 500;           //Robot remote control speed reference, unit: mm/s
	p->smooth_MotorStep = 0.02f; //Robot motor speed smoothing step value
	p->smooth_ServoStep = 20;    //Smoothing step value of the robot's servo speed
	p->SoftWare_Stop = 0;        //Robot software disable position
	
	//High-speed Ackermann chassis: set maximum motion speed limit to 6 m/s
	#if defined AKM_CAR
		if(robot.type==9) p->limt_max_speed = 6.0f;
		p->ServoLow_flag = 0;//Low-speed servo mode is off by default
	#endif
	
	//Read-only parameters, keep the default
	p->Vx = 0;
	p->Vy = 0;
	p->Vz = 0;
	p->smooth_Vx = 0;
	p->smooth_Vy = 0;
	p->smooth_Vz = 0;
	p->command_lostcount = 0;
	p->ControlMode = 1;
	p->FlagStop = 0;
	
	//Correction factor
	p->LineDiffParam = 50;
}

//Self-check variable initialization
static void ROBOT_SELFCHECK_t_Init(ROBOT_SELFCHECK_t* p)
{
	p->check_a = 0;
	p->check_b = 0;
	p->check_c = 0;
	p->check_d = 0;
	p->check_end = 0;
	p->errorflag  =0;
	p->DeepCheck = 0;
}

//Check if a variable has changed
//Entry parameters: the frequency at which the function is executed, the data to be checked, and the extent of the change indicating a change
//Return value: 1-changed; 0-unchanged
uint8_t ValChangeCheck(const uint16_t rate,const short checkval,const uint8_t changeEva)
{
	static uint16_t timecore;
	static short lastval;
	
	const uint8_t DivisionFac = 2;     //Frequency division factor
	if( rate < DivisionFac ) return 0;//Frequency too low to detect
	
	uint8_t changeflag = 0;//indicates whether there have been any changes

	timecore++;
	if( timecore >= rate/DivisionFac ) // One 500ms scan
	{
		timecore = 0;
		if( abs( lastval - checkval ) > changeEva )  changeflag = 1;
		else
		{
			changeflag=0;
		}
		lastval = checkval;
	}
	
	return changeflag ;
}

//Deep self-check procedure, which requires the cart to be set up and then run for inspection
static uint8_t Deep_SelfCheck( u16 RATE )
{	
	static uint32_t timecore = 0;
	static uint8_t errflag = 0;
	uint8_t check_ready = 0;
	
	//Measurement method: Given the encoder target value of a single wheel, observe the data conditions of each piece.
	
	/*
	Error Priority:
	1. If a negative number appears, it indicates the wrong model selection, or the encoder AB is connected incorrectly, or the motor output terminals are reversed
	2. If other encoders have values, it means the motor driver output port is connected incorrectly, or the encoder interface is connected incorrectly.
	3. If the encoder has no value + PWM is too large, it means the encoder is not connected or the motor interface is not connected
	*/

	timecore++;

	if( timecore==1 )
	{
		if( data_TaskHandle!=NULL ) vTaskSuspend(data_TaskHandle);//While the deep self-test program is running, suspend the data sending task (which occupies serial port 1) to avoid resource conflicts
		any_printf(USART1,"\r\n小车进入深度自检模式,电机的转动顺序为A、B、C、D.请观察转动情况与报错日志.\r\n");
		Buzzer_AddTask(1,20);//The buzzer indicates that deep self-test mode has been entered
	}
	
	//Set the speed of round A before 0~1 second
	if( timecore<RATE*2)
	{
		//Directly specify the motor's target value and allow responses
		robot.MOTOR_A.Target = 0.5f;
		robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;
		robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMB = 0;PWMC = 0;PWMD = 0;
	}
	else if( timecore==RATE*2 ) //Check for any abnormalities in Series A
	{
		any_printf(USART1,"================= 电机A ================\r\n");
		
		//Start checking the data
		if( robot.MOTOR_A.Encoder > 0.4f && robot.MOTOR_A.Encoder < 0.6f )
		{
			any_printf(USART1,"电机A正常.\r\n");
			//Normal Situation
			//Series A is normal
		}
		else
		{
			errflag = 1; //An error occurred
			
			//Check errors according to the error priority
			if( robot.MOTOR_A.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到A电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机A的正负极是否存在线序错误\r\n3.电机A的编码器AB相线序是否错误\r\n");
				//If a negative value appears at the corresponding interface, please check: 1. Is the model correct 2. Is the motor's positive and negative stages reversed 3. Is the wiring sequence of the encoder phase AB correct?
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机A时,检测到编码器接口B的信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、B是否存在调换\r\n否: 1.检查驱动器的A、B接口是否存在调换\r\n");
				//Encoder B has a value. Please check: 1. Whether the wiring between encoder interface A and interface B has been swapped. 2. Whether the driver's A and B interfaces have been swapped
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机A时,检测到编码器接口C的信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、C是否存在调换\r\n否: 1.检查驱动器的A、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder C has a value. Please check: 1. Whether the wiring between encoder A and port C has been swapped. 2. Whether there is a swap between driver A and C interfaces. 3. Whether the motherboard's horn holder has been swapped
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机A时,检测到编码器接口D的信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、D是否存在调换\r\n否: 1.检查驱动器的A、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder D has a value. Please check: 1. Whether the wiring between encoder A and interface D has been swapped. 2. Whether there is swapping between driver A and D interfaces. 3. Whether the board's horn holder has been swapped
			}
			else if( fabs(robot.MOTOR_A.Encoder) < 0.05f && PI_MotorA.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机A时,未检测到编码器信号.\r\n自检时电机A是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A是否存在松动或未接\r\n否: 1.检查电机A的电源线是否存在松动或未接 2.检查板子右下角左边的牛角座是否松动或未接\r\n");
				//The encoder has no value, but the corresponding interface PWM value is high. Please check: 1. Whether the encoder interface is loose or not connected. 2. Whether the motor positive and negative terminals are loose or not connected
			}
			
		}
		
		any_printf(USART1,"================= 电机A ================\r\n\r\n");
		
		//After inspection, reset the PI controller to prevent interference with the next motor inspection
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	else if( timecore>RATE*2 && timecore<RATE*4 ) //Series B control
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.5f;
		robot.MOTOR_C.Target = 0.0f;robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMA = 0;PWMC = 0;PWMD = 0;
	} 
	else if( timecore==RATE*4 )  //Round B examination
	{
		any_printf(USART1,"================= 电机B ================\r\n");
		
		if( robot.MOTOR_B.Encoder > 0.4f && robot.MOTOR_B.Encoder < 0.6f )
		{
			any_printf(USART1,"电机B正常.\r\n");
		}
		else
		{
			errflag = 1; //An error occurred
			
			if( robot.MOTOR_B.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到B电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机B的正负极是否存在线序错误\r\n3.电机B的编码器AB相线序是否错误\r\n");
				//If a negative value appears at the corresponding interface, please check: 1. Is the model correct 2. Is the motor's positive and negative stages reversed 3. Is the wiring sequence of the encoder phase AB correct?
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机B时,检测到编码器接口A的信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、B是否存在调换\r\n否: 1.检查驱动器的A、B接口是否存在调换\r\n");
				//Encoder B has a value. Please check: 1. Whether the wiring between encoder interface B and interface A has been swapped. 2. Whether the driver's interfaces B and A have been swapped
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机B时,检测到编码器接口C的信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、C是否存在调换\r\n否: 1.检查驱动器的B、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder C has a value. Please check: 1. Whether the wiring between encoder B and interface C has been swapped. 2. Whether there is swapping between driver B and C interfaces. 3. Whether the motherboard's horn holder has been swapped
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机B时,检测到编码器接口D的信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、D是否存在调换\r\n否: 1.检查驱动器的B、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder D has a value. Please check: 1. Whether the wiring between encoder B and port D has been swapped. 2. Whether there is a swap between driver B and D interfaces. 3. Whether the board's horn holder has been swapped
			}
			else if( fabs(robot.MOTOR_B.Encoder) < 0.05f && PI_MotorB.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机B时,未检测到编码器信号.\r\n自检时电机B是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B是否存在松动或未接\r\n否: 1.检查电机B的电源线是否存在松动或未接 2.检查板子右下角左边的牛角座是否松动或未接\r\n");
				//The encoder has no value, but the corresponding interface PWM value is high. Please check: 1. Whether the encoder interface is loose or not connected. 2. Whether the motor positive and negative terminals are loose or not connected
			}
		}
		any_printf(USART1,"================= 电机B ================\r\n\r\n");
		//After inspection, reset the PI controller to prevent interference with the next motor inspection
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	
	#if defined OMNI_CAR || defined _4WD_CAR || defined MEC_CAR
	
	else if( timecore>RATE*4 && timecore<RATE*6 )//C-wheel control
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.5f;robot.MOTOR_D.Target = 0.0f;
		ResponseControl();
		PWMA = 0;PWMB = 0;PWMD = 0;
	} 
	else if( timecore==RATE*6 )//Round C examination
	{
		any_printf(USART1,"================= 电机C ================\r\n");
		
		if( robot.MOTOR_C.Encoder > 0.4f && robot.MOTOR_C.Encoder < 0.6f )
		{
			any_printf(USART1,"电机C正常.\r\n");
		}
		else
		{
			errflag = 1; //An error occurred
			
			if( robot.MOTOR_C.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到C电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机C的正负极是否存在线序错误\r\n3.电机C的编码器AB相线序是否错误\r\n");
				//If a negative value appears at the corresponding interface, please check: 1. Is the model correct 2. Is the motor's positive and negative stages reversed 3. Is the wiring sequence of the encoder phase AB correct?
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机C时,检测到编码器接口A的信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、C是否存在调换\r\n否: 1.检查驱动器的A、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder B has a value. Please check: 1. Whether the wiring between encoder interface C and interface A has been swapped. 2. Whether the driver's C and A interfaces have been swapped. 3. Whether the board's horn base has been swapped
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机C时,检测到编码器接口B的信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、C是否存在调换\r\n否: 1.检查驱动器的B、C接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder C has a value. Please check: 1. Whether the wiring between encoder C and port B has been swapped. 2. Whether there is swapping between driver C and port B interfaces. 3. Whether the motherboard's horn socket has been swapped
			}
			else if( fabs(robot.MOTOR_D.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机C时,检测到编码器接口D的信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口C、D是否存在调换\r\n否: 1.检查驱动器的C、D接口是否存在调换\r\n");
				//Encoder D has a value. Please check: 1. Whether the wiring between encoder C and interface D has been swapped. 2. Whether there is a swap between driver C and D interfaces
			}
			else if( fabs(robot.MOTOR_C.Encoder) < 0.05f && PI_MotorC.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机C时,未检测到编码器信号.\r\n自检时电机C是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口C是否存在松动或未接\r\n否: 1.检查电机C的电源线是否存在松动或未接 2.检查板子右下角右边的牛角座是否松动或未接\r\n");
				//The encoder has no value, but the corresponding interface PWM value is high. Please check: 1. Whether the encoder interface is loose or not connected. 2. Whether the motor positive and negative terminals are loose or not connected
			}
		}
		any_printf(USART1,"================= 电机C ================\r\n\r\n");
		//After inspection, reset the PI controller to prevent interference with the next motor inspection
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	
	#endif /* if defined OMNI_CAR _4WD_CAR MEC_CAR */
	
	#if defined _4WD_CAR || defined MEC_CAR
	
	else if( timecore>RATE*6 && timecore<RATE*8 )//D-wheel control
	{
		robot.MOTOR_A.Target = 0.0f;robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;robot.MOTOR_D.Target = 0.5f;
		ResponseControl();
		PWMA = 0;PWMB = 0;PWMC = 0;
	}
	else if( timecore==RATE*8  )//Wheel D check
	{
		any_printf(USART1,"================= 电机D ================\r\n");
		
		if( robot.MOTOR_D.Encoder > 0.4f && robot.MOTOR_D.Encoder < 0.6f )
		{
			any_printf(USART1,"电机D正常.\r\n");
		}
		else
		{
			errflag = 1; //An error occurred
			
			if( robot.MOTOR_D.Encoder < - 0.2f ) 
			{
				any_printf(USART1,"检测到D电机编码器极性错误.\r\n");
				any_printf(USART1,"请检查：\r\n1.车型选择是否正确\r\n2.电机D的正负极是否存在线序错误\r\n3.电机D的编码器AB相线序是否错误\r\n");
				//If a negative value appears at the corresponding interface, please check: 1. Is the model correct 2. Is the motor's positive and negative stages reversed 3. Is the wiring sequence of the encoder phase AB correct?
			}
			else if( fabs(robot.MOTOR_A.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机D时,检测到编码器接口A的信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口A、D是否存在调换\r\n否: 1.检查驱动器的A、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder B has a value. Please check: 1. Whether the wiring between encoder interface D and port A has been swapped. 2. Whether the driver's interfaces D and A have been swapped. 3. Whether the board's horn holder has been swapped
			}
			else if( fabs(robot.MOTOR_B.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机D时,检测到编码器接口B的信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口B、D是否存在调换\r\n否: 1.检查驱动器的B、D接口是否存在调换 2.检查板子右下角的两个牛角座是否存在调换\r\n");
				//Encoder C has a value. Please check: 1. Whether the wiring between encoder D and interface B has been swapped. 2. Whether the driver interfaces D and B have been swapped. 3. Whether the board's horn holder has been swapped
			}
			else if( fabs(robot.MOTOR_C.Encoder) > 0.2f ) 
			{
				any_printf(USART1,"驱动电机D时,检测到编码器接口C的信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口C、D是否存在调换\r\n否: 1.检查驱动器的C、D接口是否存在调换\r\n");
				//Encoder D has a value. Please check: 1. Whether the wiring between encoder C and interface D has been swapped. 2. Whether there is a swap between driver C and D interfaces
			}
			else if( fabs(robot.MOTOR_D.Encoder) < 0.05f && PI_MotorD.Output > 5000 ) 
			{
				any_printf(USART1,"驱动电机D时,未检测到编码器信号.\r\n自检时电机D是否转动?\r\n");
				any_printf(USART1,"是: 1.检查编码器接口D是否存在松动或未接\r\n否: 1.检查电机D的电源线是否存在松动或未接 2.检查板子右下角右边的牛角座是否松动或未接\r\n");
				//The encoder has no value, but the corresponding interface PWM value is high. Please check: 1. Whether the encoder interface is loose or not connected. 2. Whether the motor positive and negative terminals are loose or not connected
			}
		}
		
		any_printf(USART1,"================= 电机D ================\r\n");
		
		//After inspection, reset the PI controller to prevent interference with the next motor inspection
		PI_Controller_Reset(&PI_MotorA);
		PI_Controller_Reset(&PI_MotorB);
		PI_Controller_Reset(&PI_MotorC);
		PI_Controller_Reset(&PI_MotorD);
	}
	#endif /* if defined  _4WD_CAR MEC_CAR */
	
	else
	{
		check_ready = 1;
		robot.MOTOR_A.Target = 0.0f;
		robot.MOTOR_B.Target = 0.0f;
		robot.MOTOR_C.Target = 0.0f;
		robot.MOTOR_D.Target = 0.0f;
		Set_Pwm(0,0,0,0,0);
		any_printf(USART1,"\r\n\r\n");
		Buzzer_AddTask(1,20);//Buzzer prompt indicates that deep self-test has been completed
		
		//Deep self-test completes with no errors, resuming the pending task.
		if( 0 == errflag  )
			if( data_TaskHandle!=NULL ) vTaskResume(data_TaskHandle);
		
		timecore = 0;
		errflag = 0;
	}

	return check_ready;
	
}
