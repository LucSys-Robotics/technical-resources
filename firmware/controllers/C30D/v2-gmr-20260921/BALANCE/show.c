#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;


u8 oled_refresh_flag;
u8 oled_page=0;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
Function: reads battery voltage, alarms with buzzer, enables self-test, sends data to the app, displays tasks on the OLED screen
Entry parameters: None
Return value: none
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz // This task runs at 10Hz
		
		//The buzzer briefly beeps at startup to remind you on startup
		//The buzzer will beep briefly when the machine is switched on
		if(SysVal.Time_count<50)Buzzer=1; 
		else if(SysVal.Time_count>=51 && SysVal.Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(LowVoltage_2>=2 && LowVoltage_2%10==2)Buzzer_count=0;  // Resets every 10 cycles
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing // Buzzer is buzzing
		else if(Buzzer_count==5)Buzzer=0;
		
		//Read the battery voltage //Read battery voltage
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All=0;
		
		if(Get_Charging_HardWare==1)
		{	//If the voltage is too low, receive an infrared signal and activate the automatic recharge function
			if(Voltage<10&&RED_STATE) Allow_Recharge=1;
		}
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds // Make sure the buzzer only rings for 0.5 seconds
		if(LowVoltage_2>=1)LowVoltage_2++; // Accumulate as long as > = 1
		
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery // 10.5V, first buzzer when low battery
		else if(Voltage<10 && LowVoltage_2==0)LowVoltage_2=1; //Initializes only when it equals 0

		if(Voltage>=10.5f && (LowVoltage_1 || LowVoltage_2))
		{	//Voltage returns to normal, low battery alarm canceled (no alarm activated to avoid interrupting the startup buzzer)
			LowVoltage_1=0;
			LowVoltage_2=0;
			Buzzer_count=25;
			Buzzer=0;
		}

		APP_Show();	 //Send data to the APP //Send data to the APP
		
		if(oled_refresh_flag) OLED_Clear(),oled_refresh_flag=0;
		else oled_show(); //Tasks are displayed on the screen //Display Tasks on Screen
   }
}

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
Function: OLED display displays tasks
Entry parameters: None
Return value: none
**************************************************************************/
extern volatile u8 clear_state;

uint8_t show_usb_enum = 0;
void OLED_ShowGamePadState(void)
{
	show_usb_enum = 1;
}

void oled_show(void)
{   
	int Car_Mode_Show;
	
	//Collect the tap information of the potentiometer, 
		 //and display the car model to be fitted when the car starts up in real time
		 //Read potentiometer setting and display the chassis type selected for startup in real time
		 Divisor_Mode=2048/CAR_NUMBER+5;
		POT_val = Get_adc_Average(Potentiometer,10);
		 Car_Mode_Show=(int) (POT_val/Divisor_Mode); 
		 if(Car_Mode_Show>7)Car_Mode_Show=7; 
		  //Car_Mode_Show=0;
	
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM is reset but not immediately refreshed to prevent screen glitches
	if(Check==0)
	{
		if( show_usb_enum == 1 )
		{
			OLED_DrawBMP(22,2,22+84,2+5,gImage_usb);
			static u8 show_delay=0;
			if(++show_delay<RATE_10_HZ*2)
			{
				return;
			}
			else 
			{
				show_delay=0;
				show_usb_enum = 0;
			}
		}
//		if( show_usb_enum && GamePadDebug.enmu_state == EnumDone)
//		{
//			if(clear) 
//			{
//				clear=0,oled_refresh_flag=1; During the enumeration, the following information is displayed on the clear screen
//				return; Perform one screen clear
//			}
//			OLED_DrawBMP(32,1,96,7,gImage_usb_bmp);
//			OLED_ShowString(12,50,"USB Init..");
//			OLED_Refresh_Line();
//			show_done = 1;
//			return;
//		}
//		else if( show_usb_enum && show_done )
//		{
//			if(show_done) // Enumeration successful, delay 1 second to display screen info
//			{
//				static u8 show_delay=0;
//				if(++show_delay<RATE_10_HZ)
//				{
//					OLED_ShowString(12,50,"USB Init OK   ");
//					OLED_Refresh_Line();
//					return;
//				}
//				show_done = 0;
//				show_delay=0;	
//				oled_refresh_flag = 1; After the delay ends, the screen clears and normal data is displayed
//				show_usb_enum = 0;
//			}
//			clear=1; Enumeration complete, wait for the next enumeration to clear again
//		}
		///////////// usb ps2 device plug/unplug prompt /////////////
	
	if(oled_page==0)
	{
		 Voltage_Show=Voltage*100; 
		memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM is reset but not immediately refreshed to prevent screen glitches
		 if(Check==0)//The car displays normally when the self-check mode is not enabled // The car displays normally when self-check mode is off
		 {	
			 //The first line of the display displays the content//
			 //Display content in line 1 //
			 if(Allow_Recharge) OLED_ShowString(0,0,"RCM ");
			 else
			 {
				 switch(Car_Mode_Show)
				 {
					case Mec_Car:       OLED_ShowString(0,0,"Mec  "); break; 
					case Omni_Car:      OLED_ShowString(0,0,"Omni "); break; 
					case Akm_Car:       OLED_ShowString(0,0,"Akm  "); break; 
					case Diff_Car:      OLED_ShowString(0,0,"Diff "); break; 
					case FourWheel_Car: OLED_ShowString(0,0,"4WD  "); break; 
					case Tank_Car:      OLED_ShowString(0,0,"Tank "); break; 
					case Mec_Car_V550:  OLED_ShowString(0,0,"Mec-V"); break;
					case FourWheel_Car_V550: OLED_ShowString(0,0,"4WD-V"); break;
				 } 
			 }
			 
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
			 {
				 //The Mec_car and omni_car show Z-axis angular velocity
				 //Mecanum and omni chassis display Z-axis angular velocity
				if(Charging)
				{	//Displays charging current while the car is charging
					OLED_ShowString(80,0," ");
					OLED_ShowString(85,0,"   ");
					OLED_ShowString(60,0,"Cur:");
					oled_showfloat(Charging_Current/1000.0f,90,0,1,2);
				}
				else
				{	//The chassis displays the Z-axis angular velocity when not charging
					//Display Z-axis angular velocity // Display the angular velocity of the Z-axis
					OLED_ShowString(76,0,"    ");
					OLED_ShowString(120,0," ");
					OLED_ShowString(60,0,"GZ");
					if( imu.gyro.z<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-imu.gyro.z,5,12);
					else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, imu.gyro.z,5,12);		
				}
			 }
			 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car_V550)
			 {
				 //Displays whether the PS2 is ready
//				 if(usb_ps2_ready==1)
//				 {
//					OLED_ShowString(55,0,"ps2 ready  ");
//				 }
//				 else
//				 {
//					 OLED_ShowString(55,0,"ps2 outline");
//				 }
//	
				 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
				 //Ackermann, differential, 4WD and tracked chassis display gyroscope zero offset
				 OLED_ShowString(55,0,"BIAS");
				 if( imu.Deviation_accel.z<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-imu.Deviation_accel.z,3,12);  //Zero-drift data of gyroscope Z axis
				 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, imu.Deviation_accel.z,3,12);	//Gyroscope Z-axis zero-drift data
			 }
			 //The first line of the display displays the content//
			 //Display content in line 1 //
			 

			 //The second line of the display displays the content//
			 //Display content in line 2 //
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car||Car_Mode==Mec_Car_V550||Car_Mode==FourWheel_Car_V550)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
				//Mecanum, omni and 4WD chassis display motor A target and actual speeds
				OLED_ShowString(0,10,"A");
				if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
															OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
				else                 	OLED_ShowString(15,10,"+"),
															OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
				
				if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
															OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,10,"+"),
															OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
				 //Ackermann, differential and tracked chassis display Z-axis angular velocity
				 OLED_ShowString(00,10,"GYRO_Z:");
				 if( imu.gyro.z<0)  OLED_ShowString(60,10,"-"),
												 OLED_ShowNumber(75,10,-imu.gyro.z,5,12);
				 else            OLED_ShowString(60,10,"+"),
												 OLED_ShowNumber(75,10, imu.gyro.z,5,12);			
			 }	 
			 //The second line of the display displays the content//
			 //Display content in line 2 //
			 
			 //Lines 3 and 4 of the display screen display content//
			 //Display content in lines 3 and 4 //
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550||Car_Mode==Mec_Car_V550)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
				//Mecanum, omni and 4WD chassis display motor B target and actual speeds
				OLED_ShowString(0,20,"B");		
				if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
				else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
				
				if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
				
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
				//Mecanum, omni and 4WD chassis display motor C target and actual speeds
				OLED_ShowString(0,30,"C");
				if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
				else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
					
				if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
				 //Ackermann, differential and tracked chassis display motor A target and actual speeds
				 OLED_ShowString(0,20,"L:");
				 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
																OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
				 else                 	OLED_ShowString(15,20,"+"),
																OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
				 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
																OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,20,"+"),
																OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
				 //Ackermann, differential and tracked chassis display motor B target and actual speeds
				 OLED_ShowString(0,30,"R:");
				 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
																OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
				 else                 	OLED_ShowString(15,30,"+"),
																OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
					
				 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
																OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,30,"+"),
																OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

	//			 if( Remoter_Ch1<0)	    OLED_ShowString(15,20,"-"),
	//															OLED_ShowNumber(20,20,-Remoter_Ch1,5,12);
	//			 else                 	OLED_ShowString(15,20,"+"),
	//															OLED_ShowNumber(20,20, Remoter_Ch1,5,12);  
	//			 if( Remoter_Ch2<0)	    OLED_ShowString(60,20,"-"),
	//															OLED_ShowNumber(75,20,-Remoter_Ch2,5,12);
	//			 else                 	OLED_ShowString(60,20,"+"),
	//															OLED_ShowNumber(75,20, Remoter_Ch2,5,12);
	//			 if( Remoter_Ch3<0)	    OLED_ShowString(15,30,"-"),
	//															OLED_ShowNumber(20,30,-Remoter_Ch3,5,12);
	//			 else                 	OLED_ShowString(15,30,"+"),
	//															OLED_ShowNumber(20,30, Remoter_Ch3,5,12);  
	//			 if( Remoter_Ch4<0)	    OLED_ShowString(60,30,"-"),
	//															OLED_ShowNumber(75,30,-Remoter_Ch4,5,12);
	//			 else                 	OLED_ShowString(60,30,"+"),
	//															OLED_ShowNumber(75,30, Remoter_Ch4,5,12);
			 }
			 //Lines 3 and 4 of the display screen display content//
			 //Display content in lines 3 and 4 //
			 
			 //Line 5 of the display displays the content//
			 //Display content in line 5 //
			 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car||Car_Mode==Mec_Car_V550||Car_Mode==FourWheel_Car_V550)
			 {
				  //Mec_Car Display the target speed and current actual speed of motor D
					//Mecanum chassis displays motor D target and actual speeds
					OLED_ShowString(0,40,"D");
					if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
																OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
					else                 	OLED_ShowString(15,40,"+"),
																OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
					if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
																OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
					else                 	OLED_ShowString(60,40,"+"),
																OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Omni_Car)
			 {
				  // The Omni_car shows Z-axis angular velocity (1000 times magnification) in rad/s
					//Omnidirectional wheel cart displays Z-axis angular velocity (magnified 1000 times), unit: rad/s
					OLED_ShowString(0,40,"MOVE_Z"); 			
					if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																							OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
					else                              	OLED_ShowString(60,40,"+"),
																							OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
			 }
			 else if(Car_Mode==Akm_Car)
			 {
				  //Akm_Car displays the PWM value of the Servo
					//Ackermann chassis displays servo PWM value
					OLED_ShowString(00,40,"SERVO:");
					if( Servo<0)		      OLED_ShowString(60,40,"-"),
																OLED_ShowNumber(80,40,-Servo,4,12);
					else                 	OLED_ShowString(60,40,"+"),
																OLED_ShowNumber(80,40, Servo,4,12); 
			 }
			 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
				 //Differential trolleys and tracked vehicles display PWM values for left and right motors
																 OLED_ShowString(00,40,"MA");
				 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
																 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(20,40,"+"),
																 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
																 OLED_ShowString(60,40,"MB");
				 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
																 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(80,40,"+"),
																 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
			 }
			 //Line 5 of the display displays the content//
			 //Display content in line 5 //
				 
			 //Displays the current control mode // Display the current control mode
			 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
			 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
			 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
			 else if ((Usart1_ON_Flag || Usart5_ON_Flag)==1) OLED_ShowString(0,50,"USART");
				else if (nav_walk==1)      OLED_ShowString(0,50,"nav");
				else                       OLED_ShowString(0,50,"ROS  ");
				
			 //Displays whether controls are allowed in the current car
			 //Shows whether the current chassis is allowed to be controlled
			 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
			 else                      OLED_ShowString(45,50,"OFF"); 
				
																	OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
										  OLED_ShowString(88,50,".");
																	OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
										  OLED_ShowString(110,50,"V");
			 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
			}	
		 
			OLED_Refresh_Gram();
		}
	
		else if(oled_page==1)
		{
			//Automatic recharge kit debug information
			OLED_ShowString(07,00,"LA  LB  RB  RA");
			OLED_ShowNumber(0+9,10,L_A,1,12);
			OLED_ShowNumber(30+9,10,L_B,1,12);
			OLED_ShowNumber(60+9,10,R_B,1,12);
			OLED_ShowNumber(90+9,10,R_A,1,12);
			OLED_ShowString(0,30,"cur:");
			OLED_ShowString(75,30,"A");
			oled_showfloat(Charging_Current/1000.0f,30,30,2,2);
			//IP display

			OLED_ShowString(0, 50,"IP:");
			OLED_ShowNumber(24,50,Received_IP[0],3,12);
			OLED_ShowNumber(48,50,Received_IP[1],3,12);
			OLED_ShowNumber(72,50,Received_IP[2],3,12);
			OLED_ShowNumber(96,50,Received_IP[3],3,12);
			OLED_Refresh_Gram();	
		}
	}
	else
	 {
		 if(check_parameter.Process==0)							//User self-test code
		 {
			 OLED_ShowCHinese(00, 00, "请先阅读使用手册");
			 OLED_ShowCHinese(00, 16, "  单击用户按键  ");
			 OLED_ShowCHinese(00, 32, "    确认开始    ");
			 OLED_ShowString(104, 50, "0/8");
		 }
		 
		 if(check_parameter.Process==1)					
		 {
			 OLED_Show_POT();
			 OLED_ShowCHinese12(8, 9, "请转动电位器：");
			 switch(Car_Mode_Show)
			 {
				case Mec_Car:       OLED_ShowString(92,10,"Mec "); break;
				case Omni_Car:      OLED_ShowString(92,10,"Omni"); break;
				case Akm_Car:       OLED_ShowString(92,10,"Akm "); break;
				case Diff_Car:      OLED_ShowString(92,10,"Diff"); break;
				case FourWheel_Car: OLED_ShowString(92,10,"4WD "); break;
				case Tank_Car:      OLED_ShowString(92,10,"Tank"); break;
				case Mec_Car_V550:  OLED_ShowString(92,10,"MecV"); break;
				case FourWheel_Car_V550: OLED_ShowString(92,10,"4WDV"); break;
			 }
			 OLED_ShowCHinese12(12, 22, "退出前需调回原车型");
			 OLED_ShowCHinese(00, 34, "  模式：电位器  ");
			 OLED_ShowString(104, 50, "1/8");
		 }
		 if(check_parameter.Process==2)
		 {
			 OLED_ShowCHinese(24, 00, "架空车轮后");
			 OLED_ShowCHinese(00, 16, "单击开始高速转动");
			 OLED_ShowCHinese(24, 32, "模式：电机");
			 OLED_ShowString(104, 50, "2/8");
		 }
		 if(check_parameter.Process==3)
		 {
				if( MOTOR_A.Encoder<0)	OLED_ShowString(00,5,"A-"),
																OLED_ShowNumber(20,5,-MOTOR_A.Encoder*1000,5,12);
				else                		OLED_ShowString(00,5,"A+"),
																OLED_ShowNumber(20,5, MOTOR_A.Encoder*1000,5,12);
				
				if( MOTOR_B.Encoder<0)	OLED_ShowString(70,5,"B-"),
																OLED_ShowNumber(90,5,-MOTOR_B.Encoder*1000,5,12);
				else                		OLED_ShowString(70,5,"B+"),
																OLED_ShowNumber(90,5, MOTOR_B.Encoder*1000,5,12);
				
				if( MOTOR_C.Encoder<0)	OLED_ShowString(00,20,"C-"),
																OLED_ShowNumber(20,20,-MOTOR_C.Encoder*1000,5,12);
				else                		OLED_ShowString(00,20,"C+"),
																OLED_ShowNumber(20,20, MOTOR_C.Encoder*1000,5,12);
				
				if( MOTOR_D.Encoder<0)	OLED_ShowString(70,20,"D-"),
																OLED_ShowNumber(90,20,-MOTOR_D.Encoder*1000,5,12);
				else                		OLED_ShowString(70,20,"D+"),
																OLED_ShowNumber(90,20, MOTOR_D.Encoder*1000,5,12);
				OLED_ShowCHinese(24, 32, "模式：电机");
				OLED_ShowString(104, 50, "2/8");
		 }
		 if(check_parameter.Process==4)
		 {
				OLED_ShowCHinese(00, 00, "    单击进入    ");
				OLED_ShowCHinese(00, 32, "  模式：陀螺仪  ");
			  OLED_ShowString(104, 50, "3/8");
		 }
		 if(check_parameter.Process==5)
		 {
			 OLED_ShowCHinese(00, 00, "晃动小车观察数据");
			 OLED_ShowString(00, 16, "G");
			 if(imu.gyro.x<0)							 OLED_ShowString(16, 16, "-"),
																	 OLED_ShowNumber(24, 16, -imu.gyro.x, 5, 12);
			 else							 					 OLED_ShowString(16, 16, "+"),
																	 OLED_ShowNumber(24, 16, imu.gyro.x, 5, 12);
			 
			 if(imu.gyro.y<0)							 OLED_ShowString(16, 26, "-"),
																	 OLED_ShowNumber(24, 26, -imu.gyro.y, 5, 12);
			 else							 					 OLED_ShowString(16, 26, "+"),
																	 OLED_ShowNumber(24, 26, imu.gyro.y, 5, 12);
			 
			 if(imu.gyro.z<0)	 						OLED_ShowString(16, 36, "-"),
																	OLED_ShowNumber(24, 36, -imu.gyro.z, 5, 12);
			 else						 						OLED_ShowString(16, 36, "+"),
																	OLED_ShowNumber(24, 36, imu.gyro.z, 5, 12);
			 
			 OLED_ShowString(64, 16, "|");
			 OLED_ShowString(64, 26, "|");
			 OLED_ShowString(64, 36, "|");
			 
			 OLED_ShowString(72, 16, "A");
			 if(imu.accel.x<0)							 OLED_ShowString(80, 16, "-"),
																	 OLED_ShowNumber(88, 16, -imu.accel.x, 5, 12);
			 else							 					 OLED_ShowString(80, 16, "+"),
																	 OLED_ShowNumber(88, 16, imu.accel.x, 5, 12);
			 
			 if(imu.accel.y<0)							 OLED_ShowString(80, 26, "-"),
																	 OLED_ShowNumber(88, 26, -imu.accel.y, 5, 12);
			 else							 					 OLED_ShowString(80, 26, "+"),
																	 OLED_ShowNumber(88, 26, imu.accel.y, 5, 12);
			 
			 if(imu.accel.z<0)							 OLED_ShowString(80, 36, "-"),
																	 OLED_ShowNumber(88, 36, -imu.accel.z, 5, 12);
			 else							 					 OLED_ShowString(80, 36, "+"),
																	 OLED_ShowNumber(88, 36, imu.accel.z, 5, 12);
			 
			 OLED_ShowString(104, 50, "3/8");
		 }
		 if(check_parameter.Process==6)
		 {
			 OLED_ShowCHinese(00, 00, "    单击进入    ");
			 OLED_ShowCHinese(8, 32, "模式：六路舵机");
			 OLED_ShowString(104, 50, "4/8");
		 }
		 if(check_parameter.Process==7 || check_parameter.Process==8 || check_parameter.Process==9 || check_parameter.Process==10 || check_parameter.Process==11 || check_parameter.Process==12)
		 {
			 OLED_ShowString(00, 00, "1:"),OLED_ShowNumber(16, 00, check_parameter.Servo_count[0], 5, 12);
			 OLED_ShowString(64, 00, "2:"),OLED_ShowNumber(80, 00, check_parameter.Servo_count[1], 5, 12);
			 OLED_ShowString(00, 10, "3:"),OLED_ShowNumber(16, 10, check_parameter.Servo_count[2], 5, 12);
			 OLED_ShowString(64, 10, "4:"),OLED_ShowNumber(80, 10, check_parameter.Servo_count[3], 5, 12);
			 OLED_ShowString(00, 20, "5:"),OLED_ShowNumber(16, 20, check_parameter.Servo_count[4], 5, 12);
			 OLED_ShowString(64, 20, "6:"),OLED_ShowNumber(80, 20, check_parameter.Servo_count[5], 5, 12);
				OLED_ShowCHinese(8, 32, "模式：六路舵机");
			 OLED_ShowString(104, 50, "4/8");
		 }
		 if(check_parameter.Process==13)
		 {
				OLED_ShowCHinese(00, 00, "    单击进入    ");
				OLED_ShowCHinese(00, 32, "  模式：蜂鸣器  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(check_parameter.Process==14)
		 {
			 OLED_ShowCHinese(24, 16, "蜂鸣器：");
			 if(Buzzer==1)		OLED_ShowCHinese(88, 16, "开");
			 else							OLED_ShowCHinese(88, 16, "关");
			 OLED_ShowCHinese(00, 32, "  模式：蜂鸣器  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(check_parameter.Process==15)
		 {
			 OLED_ShowCHinese(00, 00, "  拨动使能开关  ");
			 OLED_ShowCHinese(32, 16, "开关：");
			 if(EN==1)					OLED_ShowCHinese(80, 16, "开");
			 else								OLED_ShowCHinese(80, 16, "关");
			 OLED_ShowCHinese(8, 32, "模式：使能开关");
			 OLED_ShowString(104, 50, "6/8");
		 }
		 if(check_parameter.Process==16)
		 {
			  OLED_ShowCHinese(00, 00, "确认蓝牙红灯常亮");
			  OLED_ShowCHinese(24, 16, "后单击进入");
				OLED_ShowCHinese(24, 32, "模式：蓝牙");
			  OLED_ShowString(104, 50, "7/8");
		 }
		 if(check_parameter.Process==17)
		 {
			 OLED_ShowCHinese(8, 00, "小车行动状态：");
			 if(check_parameter.Usart2_check_dir==1)				OLED_ShowCHinese(48, 16, "向前");
			 else if(check_parameter.Usart2_check_dir==2)		OLED_ShowCHinese(48, 16, "右上");
			 else if(check_parameter.Usart2_check_dir==3)		OLED_ShowCHinese(48, 16, "向右");
			 else if(check_parameter.Usart2_check_dir==4)		OLED_ShowCHinese(48, 16, "右下");
			 else if(check_parameter.Usart2_check_dir==5)		OLED_ShowCHinese(48, 16, "向后");
			 else if(check_parameter.Usart2_check_dir==6)		OLED_ShowCHinese(48, 16, "左下");
			 else if(check_parameter.Usart2_check_dir==7)		OLED_ShowCHinese(48, 16, "向左");
			 else if(check_parameter.Usart2_check_dir==8)		OLED_ShowCHinese(48, 16, "左上");
			 else if(check_parameter.Usart2_check_dir==9)		OLED_ShowCHinese(48, 16, "停止");
			 OLED_ShowCHinese(24, 32, "模式：蓝牙");
			 OLED_ShowString(104, 50, "7/8");
		 }
		 if(check_parameter.Process==18)
		 {
				OLED_ShowCHinese(8, 00, "使用数据线连接");
			  OLED_ShowCHinese(20, 16, "中间的");
			  OLED_ShowString16(68, 16, "USB");
				OLED_ShowCHinese(92,16, "口");
				OLED_ShowCHinese(20, 32, "模式：串口");
			  OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(check_parameter.Process==19)
		 {
			 OLED_ShowCHinese(00, 00, "发送");
			 OLED_ShowString16(36, 00, "#");
			 OLED_ShowCHinese(48, 00, "结尾的数据");
			 
			 OLED_ShowCHinese(8, 16, "波特率：");
			 OLED_ShowString16(72, 16, "115200");
//			 OLED_ShowCHinese(00, 00, "Click to send");
//			 OLED_ShowString16(64, 00, "LUCSYS");
//			 OLED_ShowCHinese (00, 16, "received");
//			 OLED_ShowString16(48, 16, "ASCII:");
//			 OLED_ShowNumber(96, 18, (USART3->DR), 4, 12); Characters are sent on the serial assistant, so ASCII codes are used here for display
			 OLED_ShowCHinese(20, 32, "模式：串口");
			 OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(check_parameter.Process==20)
		 {
				OLED_ShowCHinese(00, 00, "即将退出质检模式");
			  OLED_ShowCHinese(00, 16, "请转动电位器回到");
			  OLED_ShowCHinese(12, 32, "当前车型：");
			  switch(Car_Mode_Show)
			  {
					case Mec_Car:       OLED_ShowString16(92,32,"Mec "); break; 
					case Omni_Car:      OLED_ShowString16(92,32,"Omni"); break; 
					case Akm_Car:       OLED_ShowString16(92,32,"Akm "); break; 
					case Diff_Car:      OLED_ShowString16(92,32,"Diff"); break; 
					case FourWheel_Car: OLED_ShowString16(92,32,"4WD "); break; 
					case Tank_Car:      OLED_ShowString16(92,32,"Tank"); break; 
					case Mec_Car_V550:  OLED_ShowString(92,32,"MecV"); break;
					case FourWheel_Car_V550: OLED_ShowString(92,32,"4WDV"); break;
			  }
				OLED_ShowString(104, 50, "8/8");
		 }
		 OLED_ShowCHinese12(00, 50, "单击继续双击退出");
		 OLED_Refresh_Gram();
	 }
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
Function: Send data to the app
Entry parameters: None
Return value: none
**************************************************************************/
int voltage_to_percentage(float voltage) {
    const float V_MIN = 9.5f;  // Minimum voltage (0%)
    const float V_MAX = 12.2f; // Maximum voltage (100%)
    
    // Ensure the voltage is within the effective range
    if (voltage < V_MIN) return 0;
    if (voltage > V_MAX) return 100;
    
    // Linear interpolation calculates percentages and rounds
    int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // Rounded up
	
	if( percentage > 100 ) percentage = 100;
	if( percentage <= 0 ) percentage = 0;
	
    return percentage;
}

void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //Processing battery voltage as a percentage
	Voltage_Show = voltage_to_percentage(Voltage);
//	 Voltage_Show=(Voltage*1000-10000)/27;
//	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //Wheel speed units are converted to 0.01 m/s, making it easy to display in the app
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //Used to alternately print APP data and display waveform
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //Send parameters to the app, which displays them on the debug interface
		 printf("{C%d:%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI,LineDiffParam);
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0) 
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //Send parameters to the app, which will display them on the homepage
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)imu.gyro.z);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //Send parameters to the app, which displays them on the waveform interface
		 printf("{B%d:%d:%d}$",(int)imu.gyro.x,(int)imu.gyro.y,(int)imu.gyro.z);
	 }
	 if(Debug_Flag)
	 {
		 if(IP_Frame_Valid)//IP request has been closed
			printf("{#IP:%d.%d.%d.%d}$",(u8)Received_IP[0],(u8)Received_IP[1],(u8)Received_IP[2],(u8)Received_IP[3]);//Upload IP
		 else printf("{#IP:unknow..}$");//IP unknown
		 Debug_Flag=0;
	 }
}

float base_vol=11.5f;
#define VOL_COUNT 100
float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};
	
	/*----------- Array Initialization -----------*/
	static u8 once=1;
	if(once)
	{
		once=0;
		for(i=0;i<VOL_COUNT;i++)
			Speed_Buf[i]=base_vol;
	}
	/*-------------------------------*/
	
    for(i = 1 ; i<VOL_COUNT; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[VOL_COUNT - 1] =data;

    for(i = 0 ; i < VOL_COUNT; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
    return Filter_Speed;
}

