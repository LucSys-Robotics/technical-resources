#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int Time_count;
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
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
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
		

		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds // Make sure the buzzer only rings for 0.5 seconds
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds // Make sure the buzzer only rings for 0.5 seconds
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
extern u8 start_clear;
extern u8 clear_state;
void oled_show(void)
{
	int Car_Mode_Show;

    //Collect the tap information of the potentiometer,
    //and display the car model to be fitted when the car starts up in real time
    //Read potentiometer setting and display the chassis type selected for startup in real time
    Car_Mode_Show=(int) (Get_adc_Average(CAR_MODE_ADC,10)/Divisor_Mode);

    Voltage_Show=Voltage*100;

    //The first line of the display displays the content//
    //Display content in line 1 //
    OLED_ShowString(0,0,"TYPE:");
    if (robot_mode_check_flag==0) OLED_ShowNumber(40,0,Car_Mode_Show,2,12);	 //Display robot type // Display robot type
    else if (robot_mode_check_flag==1) OLED_ShowString(40,0,"X");            //Type mismatch displays "X" // Show "X" when type mismatches

    OLED_ShowString(0,10,"A");
    if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
          OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
    else                 	OLED_ShowString(15,10,"+"),
                                OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12);

    if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
        OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
    else                 	OLED_ShowString(60,10,"+"),
                                OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);

    //The third line of the display displays the content//
    //Display content in line 3 //
    //Display the target speed and current speed of motor B
    //Displays the target speed and current speed of motor B
    OLED_ShowString(0,20,"B");
    if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
          OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
    else                 	OLED_ShowString(15,20,"+"),
                                OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12);

    if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
        OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
    else                 	OLED_ShowString(60,20,"+"),
                                OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);

    //The fourth line of the display displays the content//
    //Display content in line 4 //
    //Display the target speed and current speed of motor C
    //Displays the target speed and current speed of motor C
    OLED_ShowString(0,30,"C");
    if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
          OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
    else                 	OLED_ShowString(15,30,"+"),
                                OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12);

    if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
        OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
    else                 	OLED_ShowString(60,30,"+"),
                                OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);

    //Line 5 of the display displays the content//
    //Display content in line 5 //
#if Mec
    //Mecanum chassis displays motor C target and current speeds
    //Wheel trolley displays the target speed and current speed of motor C
    OLED_ShowString(0,40,"D");
    if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
          OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
    else                 	OLED_ShowString(15,40,"+"),
                                OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12);

    if( MOTOR_D.Encoder<0)OLED_ShowString(60,40,"-"),
        OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
    else                 	OLED_ShowString(60,40,"+"),
                                OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);

#elif Omni
    // Omnidirectional wheel shows Z-axis speed (1000 times magnification) in rad/s //
    // Omnidirectional wheel displays Z-axis speed (magnified 1000x), unit: rad/s //
    OLED_ShowString(0,40,"MOVE_Z");
    if( Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
        OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
    else                 	OLED_ShowString(60,40,"+"),
                                OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
#endif

    //Content displayed in line 6 of the display
    //Line 6 of the display displays the contents
    //Displays the current control mode // Display the current control mode

    if      (PS2_ON_Flag==1)
        OLED_ShowString(0,50,"PS2  ");
    else if (APP_ON_Flag==1)
        OLED_ShowString(0,50,"APP  ");
    else if (Remote_ON_Flag==1)
        OLED_ShowString(0,50,"R-C");
    else if (Usart_ON_Flag==1)
        OLED_ShowString(0,50,"USART");
    else if (CAN_ON_Flag==1)
        OLED_ShowString(0,50,"CAN");
    else
        OLED_ShowString(0,50,"ROS");

//    if(A280.kinematic_mode==Inverse_Kinematic)OLED_ShowString(110,0,"-I");
//    else if(A280.kinematic_mode==Forward_Kinematic)OLED_ShowString(110,0,"-F");
//    if      (A280.control_mode==PS2_Mode)
//        OLED_ShowString(70,0,"PS2-ARM");
//    else if (A280.control_mode==APP_Mode)
//        OLED_ShowString(70,0,"APP-A");
//    else if (Arm_Mode_Remote==1)
//        OLED_ShowString(70,0,"RC--A");
//    else if (Arm_Mode_Usart1==1)
//        OLED_ShowString(70,0,"URT-ARM");
//    else if (Arm_Mode_CAN==1)
//        OLED_ShowString(70,0,"CAN-ARM");
//    else if (Arm_Mode_ROS==1)
//        OLED_ShowString(70,0,"ROS-ARM");
//    else
//        OLED_ShowString(70,0,"ARM-NO ");

    //Displays whether controls are allowed in the current car
    //Shows whether the current chassis is allowed to be controlled
    if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");
    else                      OLED_ShowString(45,50,"OFF");

    //Displays the current battery voltage
    //Displays the current battery voltage
    OLED_ShowString(88,50,".");
    OLED_ShowString(110,50,"V");
    OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
    OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
    if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);

    //Refresh the screen // Refresh the screen
    OLED_Refresh_Gram();
	
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
Function: Send data to the app
Entry parameters: None
Return value: none
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //Processing battery voltage as a percentage
	 Voltage_Show=(Voltage*100-2000)*5/26;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
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
		 printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);//Print it onto the app
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0)
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //Send parameters to the app, which will display them on the homepage
	   printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]); 
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //Send parameters to the app, which displays them on the waveform interface
	   printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
	 }

}

float base_vol=22.5f;
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


