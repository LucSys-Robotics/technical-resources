#include "pstwo.h"
#define DELAY_TIME  delay_us(5); 

//Button value reading, zero time storage
//Read button value, temporarily store
u16 Handkey;	
//Start the order. Request data
//Start command. Request data
u8 Comd[2]={0x01,0x42};	
//Data store array
//Data storage array
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	}; //Key value and key name //Key value and key name
/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
Functions: PS2 controller tasks
Entry parameters: None
Return value: none
**************************************************************************/	
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			//The task is run at 100hz
			//This task runs at a frequency of 100Hz
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
			//Read the ps2 data
			//Read data from the PS2
      PS2_Read(); 	
    }
}  
/**************************************************************************
Function: Ps2 handle initializer
Input   : none
Output  : none
Function: Initialize PS2 controller
Entry parameters: None
Return value: none
**************************************************************************/	
void PS2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable the ability port clock
	//Enable port clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//port configuration
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//Normal input mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//Pull down
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //Output
	GPIO_InitStructure.GPIO_OType =GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz;  //2M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Initialize the GPIO according to the set parameters
	GPIO_Init(GPIOE, &GPIO_InitStructure);		

}
/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
Function: Read control from the PS2 controller
Entry parameters: None
Return value: none
**************************************************************************/	
void PS2_Read(void)
{
	//Reading key
  //Read key values
	PS2_KEY=PS2_DataKey(); 
	//Read the analog of the remote sensing x axis on the left
  //Read the analog quantity in the X-axis direction of the left remote sensing
	PS2_LX=PS2_AnologData(PSS_LX); 
	//Read the analog of the directional direction of remote sensing on the left
  //Read the analog quantity in the left remote sensing Y-axis direction
	PS2_LY=PS2_AnologData(PSS_LY);
	//Read the analog of the remote sensing x axis
  //Read the analog quantity in the X-axis direction of the remote sensing on the right
	PS2_RX=PS2_AnologData(PSS_RX);
	//Read the analog of the directional direction of the remote sensing y axis
  //Read the analog quantity in the Y-axis direction of the right remote sensing
	PS2_RY=PS2_AnologData(PSS_RY);  
 
	if(PS2_KEY==4&&Deviation_Count>=CONTROL_DELAY)
	{
		//When the button is pressed, you need to push the right side forward to the formal ps2 control car
		//After pressing the Start button, you need to push down the right forward lever to officially control the car on PS2
		
		PS2_ON_Flag=1;  
	}
	if(PS2_KEY==14&&Deviation_Count>=CONTROL_DELAY) 	
	{
		if(A280.control_mode!=PS2_Mode)
		{
			PS2_ON_Flag=0;
			A280.control_mode=PS2_Mode;
		}
		else
		{
			  A280.control_mode=ROS_Mode;
				PS2_ON_Flag=1;  
		}
		
	}
}
/**************************************************************************
Function: Send commands to the handle
Input   : none
Output  : none
Function: Send commands to the handle
Entry parameters: None
Return value: none
**************************************************************************/	
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;     //Output a control bit
		}
		else DO_L;
		delay_us(16);
		CLK_H;      //Clock lift // Clock lift
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		delay_us(16);
		if(DI)
			Data[1] = ref|Data[1];
		delay_us(16);
	}
	delay_us(16);
}
/**************************************************************************
Function: Whether it is a red light mode, 0x41= analog green light, 0x73= analog red light
Input   : none
Output  : 0: red light mode, other: other modes
Function: Check if it¡¯s red light mode, 0x41 = simulate green light, 0x73 = simulate red light
Entry parameters: None
Return value: 0-red-light mode; otherwise, another mode
**************************************************************************/	
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders // Start command
	PS2_Cmd(Comd[1]);  //Request data // Request data
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/**************************************************************************
Function: Read the handle data
Input   : none
Output  : none
Function: Read handle data
Entry parameters: None
Return value: none
**************************************************************************/	
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders // Start command
	PS2_Cmd(Comd[1]);  //Request data // Request data
	for(byte=2;byte<9;byte++) //Start receiving data // Start receiving data
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}
/**************************************************************************
Function: Handle the data of the read 2 and handle only the key parts
Input   : none
Output  : 0: only one button presses the next time; No press
Function: Processes the data read from the PS2, only handling the button parts
Entry parameters: None
Return value: 0-exactly one button pressed; 1-not pressed
**************************************************************************/	
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3]; //This is 16 buttons, pressed down to 0, and not pressed for 1 // This is 16 buttons, pressed down to 0, and not pressed for 1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;  //No buttons // No buttons pressed
}
/**************************************************************************
Function: Get a simulation of a rocker
Input   : Rocker
Output  : Simulation of rocker, range 0~ 256
Function: Obtain the analog of a joystick
Entry parameter: joystick
Return value: joystick analog value, range 0~256
**************************************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}
/**************************************************************************
Function: Clear data buffer
Input   : none
Output  : none
Function: Clear data buffers
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function: Handle vibration function
Input   : motor1: the right small vibrator, 0x00, other
          motor2: the left big shock motor 0x40~ 0xff motor is open, and the value is greater
Output  : none
Function: handle vibration function
Entry parameters: motor1: small vibration motor on the right side, 0x00 off, others on
	        motor2: Motor with strong vibration on the left 0x40~0xFF Motor on, the higher the value, the greater the vibration
Return value: none
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
  PS2_Cmd(0x01); //Start order // Start command
	PS2_Cmd(0x42); //Request data // Request data
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
/**************************************************************************
Function: Short press
Input   : none
Output  : none
Function: short press
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Enter configuration
Input   : none
Output  : none
Function: Enter configuration
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_EnterConfing(void)
{
  CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Send mode Settings
Input   : none
Output  : none
Function: Send mode settings
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01; digital=0x00 Software Settings send mode The software sets the send mode
	PS2_Cmd(0x03); //0x03 lock storage setup, which cannot be set by the key "mode" set mode. 0x03 latch settings, meaning you cannot set the mode via the "MODE" key.
				         //0xee non-locking software settings can be set by pressing the "MODE" button to set the mode.
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Vibration setting
Input   : none
Output  : none
Function: vibration settings
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Complete and save the configuration
Input   : none
Output  : none
Function: Complete and save the configuration
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Handle configuration initialization
Input   : none
Output  : none
Function: Initialize handle configuration
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		  //Enter configuration mode
	PS2_TurnOnAnalogMode();	//The "traffic light" configuration mode and choose whether to save //"Traffic light" configuration mode, and choose whether to save
	//PS2_VibrationMode(); Open vibration mode // Enable vibration mode
	PS2_ExitConfing();		  //Complete and save the configuration // Complete and save configuration
}
/**************************************************************************
Function: Read the handle information
Input   : none
Output  : none
Function: Read handle information
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_Receive (void)
{
	if(PS2_ON_Flag)
		{
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_RX=PS2_AnologData(PSS_RX);
		PS2_RY=PS2_AnologData(PSS_RY);
		}
		PS2_KEY=PS2_DataKey();
}


