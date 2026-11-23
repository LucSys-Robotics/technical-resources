#include "control.h"	
#include "filter.h"	
  /**************************************************************************
Distribution maintenance: LucSys Robotics 
**************************************************************************/

u8 Flag_Target,Flag_Change;                             //Related flags
u8 temp1;                                               //Temporary variables
float Voltage_Count,Voltage_All; 											  //Voltage sampling related variables
float Gyro_K=0.004;     				  											//Gyroscope scale factor
int j;
#define a_PARAMETER          (0.275f)   
#define T 0.320f    //0.145f
#define L 0.315f    //0.17f
#define K 570.8f
/**************************************************************************
Function: mathematical model of the cart's motion
Entry parameters: Y-axis speed and angle
Return value: none
**************************************************************************/
void Kinematic_Analysis(float Vy,float angle)
{
        Target_A   = Vy*(1+T*tan(angle)/2/L);
        Target_B   = Vy*(1-T*tan(angle)/2/L);
	      Servo=SERVO_INIT-angle*K;
}
/**************************************************************************
Function: All control codes are contained here
         5ms timed interrupt triggered by the MPU6050 INT pin
         Strictly ensure synchronization of sampling and data processing times
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //Clear the interrupt flag on LINE5
		   Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //Provide the main function with a precise delay of 50ms
			 }
		  if(Flag_Target==1)                                                  //5ms reads the values from the gyroscope and accelerometer once
			{
					if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//If the serial port control flag is unlocked, enter serial port control mode
					Read_DMP();                                                           //===Update posture
			  	Key();//Scan key changes
			    return 0;	                                               
			}                                                                     	 //==== 10ms control once. To ensure the time reference for M method speed measurement, encoder data is read first
			UA_Encoder=Read_Encoder(2);                                          //==== Read the value of the encoder
			Encoder_A=UA_Encoder/25;
			Position_A+=Encoder_A;                                                 //=== Integrate to get velocity
			UB_Encoder=-Read_Encoder(3);                                          //==== Read the value of the encoder
			Encoder_B=UB_Encoder/25;
			Position_B+=Encoder_B;                                                 //=== Integrate to get velocity
	  	Read_DMP();                                                            	//===Update posture
  		Led_Flash(100);                                                       	 //==== LED flashing; Normal mode 1 second changes the indicator light status once
			Voltage_All+=Get_battery_volt();                                      	 //Multiple sampling accumulations
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//Calculate the average value to obtain the battery voltage
		  if(PS2_KEY==4)PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;									
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1) CAN_N_Usart_Control();    //After receiving unlock command from serial or CAN remote, enable CAN and serial control input
			if(RC_Velocity>0&&RC_Velocity<15)  RC_Velocity=15;                   //Avoid the motor's low-speed nonlinear region
			if(Turn_Off(Voltage)==0)               //==== If there is no abnormal battery voltage
			 { 			 	
				 if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)      Get_RC(Run_Flag);//=== If neither serial port nor CAN control is enabled, receive Bluetooth remote control
				 Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //=== Speed closed-loop control calculates the final PWM of motor A
				 Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //=== Speed closed-loop control calculates motor B's final PWM
				 Xianfu_Pwm(6900);                     //=== PWM limit
				 Set_Pwm(-Motor_A,-Motor_B,Servo);     //==== Assign value to PWM registers
			 }
			 else	Set_Pwm(0,0,SERVO_INIT);    //==== Assign value to PWM registers
	 }
	 return 0;	 
} 


/**************************************************************************
Function: assign values to PWM registers
Input parameters: PWM
Return value: none
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
	  
   	if(motor_a<0)			INA2=1,			INA1=0;
			else 	          INA2=0,			INA1=1;
		PWMA=myabs(motor_a);
	
		if(motor_b<0)			INB2=1,			INB1=0;
		else 	            INB2=0,			INB1=1;
		PWMB=myabs(motor_b);
	    SERVO=servo;
	
}

/**************************************************************************
Function: Restrict PWM assignment
Entry parameters: magnitude
Return value: none
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;
    if (Servo>1930)	 Servo=1930;
//	  if (Servo<1035)	 Servo=1035;	
}
/**************************************************************************
Function: Set speed during position PID control
Entry parameters: None, amplitude
Return value: none
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//The operating speed of motor A in position control mode
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //The operating speed of motor A in position control mode
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//The operating speed of motor B in position control mode
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//The operating speed of motor B in position control mode
}
/**************************************************************************
Function: Modify the chassis's operating status with a button press
Entry parameters: None
Return value: none
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100); 
	if(tmp==2)Flag_Show=!Flag_Show;//Double-click to control display mode
}

/**************************************************************************
Function: Abnormal motor shutdown
Entry parameters: voltage
Return value: 1-abnormal; 0-normal
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1110)//Battery voltage too low, shutting down the motor
			{	                                                
      temp=1;      
      PWMA=0;
      PWMB=0;					
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
Function: absolute value function
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
Function: Incremental PI controller
Entry parameters: encoder measurements, target speed
Return value: motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) represents the previous deviation, and so on
pwm stands for incremental output
In our speed control closed-loop system, only PI control is used
pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //Calculate bias
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //Incremental PI controller
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //Save the previous deviation
	 return Pwm;                         //Incremental output
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //Calculate bias
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //Incremental PI controller
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //Save the previous deviation
	 return Pwm;                         //Incremental output
}

/**************************************************************************
Function: Remote control of the chassis via serial commands
Entry parameters: serial port instructions
Return value: none
**************************************************************************/
void Get_RC(u8 mode)
{
//	  float step=0.3f; Set the speed control step value.
	  if(mode==0)//Velocity
		{	
				 switch(Flag_Direction)   //Directional control
				 { 
				 case 1:      Move_Y=RC_Velocity;  	 	 Angle=0;        break;
				 case 2:      Move_Y=RC_Velocity;  	 	 Angle=PI/4;   	 break;
				 case 3:      Move_Y=0;      				 	 Angle=0;   	   break;
				 case 4:      Move_Y=-RC_Velocity;  	 Angle=-PI/4;    break;
				 case 5:      Move_Y=-RC_Velocity;  	 Angle=0;        break;
				 case 6:      Move_Y=-RC_Velocity;  	 Angle=PI/4;     break;
				 case 7:      Move_Y=0;     	 			 	 Angle=0;        break;
				 case 8:      Move_Y=+RC_Velocity; 	 	 Angle=-PI/4;    break; 
				 default:     Move_Y=0;                Angle=0;        break;
			 } 
//				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
//				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
//				if(Angle<-RC_Velocity) Angle=-RC_Velocity;	
//				if(Angle>RC_Velocity)  Angle=RC_Velocity;	 
	 }
		 Kinematic_Analysis(Move_Y,Angle);//Get control target values for kinematic analysis
}

/**************************************************************************
Function: Receives CAN or serial control commands for processing
Entry parameters: None
Return value: none
**************************************************************************/
void CAN_N_Usart_Control(void)
{
//			int flag_1, flag_2;
			int RX,LY;
		  int Yuzhi=20;
			if(CAN_ON_Flag==1||Usart_ON_Flag==1) 
			{
//			 if(rxbuf[1]) flag_1 = 1; else flag_1 = -1; // Direction control bit
//			 if(rxbuf[3]) flag_2 = 1; else flag_2 = -1; // Direction control bit
//			 Target_A=flag_1*rxbuf[0];
//			 Target_B=flag_2*rxbuf[1];
			 if(rxbuf[1]==0)Move_Y=rxbuf[0]; //Identify movement direction
			 else           Move_Y=-rxbuf[0]; //Velocity
			 Angle=(rxbuf[2]-90)*PI/180;   //Angle acquisition
			}
			else if (PS2_ON_Flag==1)
	    {
	     RX=PS2_RX-128;
			 LY=PS2_LY-128;
			 if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //Eliminate nonlinear areas
			 if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		   Angle= RX*PI/4/120;
		   Move_Y=-LY/2.84;	 
		  //if(Move_Y<0)Angle=-Angle;	  
	    }
			Kinematic_Analysis(Move_Y,Angle);//Get control target values for kinematic analysis
}
