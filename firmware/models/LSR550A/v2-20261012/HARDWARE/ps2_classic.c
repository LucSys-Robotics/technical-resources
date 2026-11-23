#include "pstwo.h"
#include "delay.h"

//Introduced PS2 controller enumeration
#include "WiredPS2_gamepad.h"

#define DELAY_TIME  delay_us(5);

static GamePadKeyStateType_t ClassicPS2_GetKeyState(uint8_t bit);

//Classic PS2 controller
GamePadType_t Classic_PS2Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 0,
	.SetVibration = NULL,
	.getKeyState = ClassicPS2_GetKeyState
};


//
__weak void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	printf("%d,%d\r\n",keyid,event);
}


//Button value reading, zero time storage
//Read button value, temporarily store
u16 Handkey;
//Start the order. Request data
//Start command. Request data
u8 Comd[2]= {0x01,0x42};
//Data store array
//Data storage array
u8 Data[9]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//Analog analog data
#define Rocker_RX 5                
#define Rocker_RY 6
#define Rocker_LX 7
#define Rocker_LY 8

//Internal use function
static void PS2_Cmd(u8 CMD);

//static u8 PS2_RedLight(void);
//static void PS2_Vibration(u8 motor1, u8 motor2);
//static void PS2_VibrationMode(void);

static void PS2_ReadData(void);
static u16 PS2_DataKey(void);
static u8 PS2_AnologData(u8 button);
static void PS2_ClearData(void);
static void PS2_ShortPoll(void);
static void PS2_EnterConfing(void);
static void PS2_TurnOnAnalogMode(void);
static void PS2_ExitConfing(void);
static void PS2_SetInit(void);

//Get the handle key-value status function
static GamePadKeyStateType_t ClassicPS2_GetKeyState(uint8_t bit)
{
	if( (Handkey>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}


//Defines the individual key values for key detection events
static GamePad_CheckEventType_t GamePadKeyCheckEvent[16] = { 0 };

void PS2_Key_Param_Init(void)
{
	gamepad_brand = PS2_USB_Wiredless;
	GamePadInterface = &Classic_PS2Gamepad;
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
	//Pin initialization
    GPIO_InitTypeDef GPIO_InitStructure;
	
	//Corresponding pin clock on
	ENABLE_PS2_DI_PIN_CLOCK;
	ENABLE_PS2_DO_PIN_CLOCK;
	ENABLE_PS2_CS_PIN_CLOCK;
	ENABLE_PS2_CLK_PIN_CLOCK;
	
	//Pin mode settings
    GPIO_InitStructure.GPIO_Pin = PS2_DI_PIN;			//port configuration
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//Normal input mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		//Pulldown (data read pin set to dropdown; when PS2 is pulled out, all signals read are 0; If configured as pull-up, all signals are set to 1)
    GPIO_Init(PS2_DI_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PS2_DO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //Output
    GPIO_InitStructure.GPIO_OType =GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz;  //2M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Initialize the GPIO according to the set parameters
    GPIO_Init(PS2_DO_PORT, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = PS2_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //Output
    GPIO_InitStructure.GPIO_OType =GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz;  //2M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Initialize the GPIO according to the set parameters
    GPIO_Init(PS2_CS_PORT, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = PS2_CLK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   //Output
    GPIO_InitStructure.GPIO_OType =GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed =GPIO_Speed_2MHz;  //2M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//Initialize the GPIO according to the set parameters
    GPIO_Init(PS2_CLK_PORT, &GPIO_InitStructure);
	
	//PS2 internal parameter initialization
	PS2_SetInit();
}


/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
Function: Read control values from the PS2 controller (function used externally)
Entry parameters: None
Return value: none
**************************************************************************/
void PS2_Read(void)
{
    //Reading key
    //Read key values
	PS2_DataKey();
    //Read the analog of the remote sensing x axis on the left
    //Read the analog quantity in the X-axis direction of the left remote sensing
    Classic_PS2Gamepad.LX = PS2_AnologData(Rocker_LX);
    //Read the analog of the directional direction of remote sensing on the left
    //Read the analog quantity in the left remote sensing Y-axis direction
    Classic_PS2Gamepad.LY= 255 - PS2_AnologData(Rocker_LY);
    //Read the analog of the remote sensing x axis
    //Read the analog quantity in the X-axis direction of the remote sensing on the right
    Classic_PS2Gamepad.RX=PS2_AnologData(Rocker_RX);
    //Read the analog of the directional direction of the remote sensing y axis
    //Read the analog quantity in the Y-axis direction of the right remote sensing
    Classic_PS2Gamepad.RY=PS2_AnologData(Rocker_RY);
	
	//LT, RT assignment
	if( ClassicPS2_GetKeyState(PS2KEY_L2)!=0 ) Classic_PS2Gamepad.LT = 255;
	else  Classic_PS2Gamepad.LT = 0;
	
	if( ClassicPS2_GetKeyState(PS2KEY_R2)!=0 ) Classic_PS2Gamepad.RT = 255;
	else  Classic_PS2Gamepad.RT = 0;
	
	//Button callback function triggered
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(Handkey,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//Trigger callback function
		Classic_PS2GamePad_KeyEvent_Callback(key,event);
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
static void PS2_Cmd(u8 CMD)
{
    volatile u16 ref=0x01;
    Data[1] = 0;
    for(ref=0x01; ref<0x0100; ref<<=1)
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
//static u8 PS2_RedLight(void)
//{
//    CS_L;
//    PS2_Cmd(Comd[0]); Start orders // Start command
//    PS2_Cmd(Comd[1]); Request data // Request data
//    CS_H;
//    if( Data[1] == 0X73)   return 0 ;
//    else return 1;

//}
/**************************************************************************
Function: Read the handle data
Input   : none
Output  : none
Function: Read handle data
Entry parameters: None
Return value: none
**************************************************************************/
static void PS2_ReadData(void)
{
    volatile u8 byte=0;
    volatile u16 ref=0x01;
    CS_L;
    PS2_Cmd(Comd[0]);  //Start orders // Start command
    PS2_Cmd(Comd[1]);  //Request data // Request data
    for(byte=2; byte<9; byte++) //Start receiving data // Start receiving data
    {
        for(ref=0x01; ref<0x100; ref<<=1)
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
u8 ps2_default_buf[9] = {0,0x73,0,0xff,0xff,128,128,128,128};

static u16 PS2_DataKey(void)
{
	static u8 last_ps2_data[9] = {0,0,0,0xff,0xff,128,128,128,128};
	static u8 ps2_connect_timeout = 0;
	
	//Read PS2 controller data
    PS2_ClearData();
    PS2_ReadData();
	
	//Wired PS2 Digital Mode (0x41) or Wireless PS2 Controller Digital Mode (0x20) shows hot-plugged in this mode
	//255 is abnormal data; the wireless controller is only partially inserted and not fully inserted to read this data
	if( Data[1]==0x41 || Data[1]==0x20 || Data[1]==255 )
	{
		u8 save = Data[1];
		PS2_SetInit();//Initialize to simulation mode
		memcpy(Data,ps2_default_buf,9);//The first set of data uses mode configuration to prevent loss of control
		Data[1] = save;//Write the current value to prevent initialization failure that would skip the judgment
	}
	
	//If all data is 0, use the previous data (all 0s or all 1s, related to pin configuration initialization pull down or down).
	u8 a=0;
	for(u8 i=0;i<9;i++)
	{
		a+=Data[i];
	}
	if(a==0) //Data set to zero indicates the handle has been removed
	{
		//In this state, the PS2 controller can be considered disconnected. After a certain number of uses, the PS2 control will be cleared
		ps2_connect_timeout++;
		if( ps2_connect_timeout> 10 ) 
		{   //Handle connection timeout, stop chassis control using default variables
			ps2_connect_timeout = 11, memcpy(last_ps2_data,ps2_default_buf,9);
		}			
		
		//Use the previous data when all keys are fully 0 (all keys are pressed simultaneously to count to 0)
		memcpy(Data,last_ps2_data,9);
	}
	else
	{
		ps2_connect_timeout = 0;//Timer reset
	}
	
	//Save the data for this event
	memcpy(last_ps2_data,Data,9); 
	
	//This is 16 buttons, pressed down to 0, and not pressed for 1 
	//These are 16 buttons: 0-pressed, 1-released
    Handkey=(Data[4]<<8)|Data[3]; 
	
	//Converts to 1 when pressed, 0 when not pressed (suitable for USB PS2 data format)
	Handkey = ~Handkey;
	
	//Return key value state
	return Handkey;
}
/**************************************************************************
Function: Get a simulation of a rocker
Input   : Rocker
Output  : Simulation of rocker, range 0~ 256
Function: Obtain the analog of a joystick
Entry parameter: joystick
Return value: joystick analog value, range 0~256
**************************************************************************/
static u8 PS2_AnologData(u8 button)
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
static void PS2_ClearData(void)
{
    u8 a;
    for(a=0; a<9; a++)
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
//static void PS2_Vibration(u8 motor1, u8 motor2)
//{
//    CS_L;
//    delay_us(16);
//    PS2_Cmd(0x01); Start order // Start command
//    PS2_Cmd(0x42); Request data // Request data
//    PS2_Cmd(0X00);
//    PS2_Cmd(motor1);
//    PS2_Cmd(motor2);
//    PS2_Cmd(0X00);
//    PS2_Cmd(0X00);
//    PS2_Cmd(0X00);
//    PS2_Cmd(0X00);
//    CS_H;
//    delay_us(16);
//}
/**************************************************************************
Function: Short press
Input   : none
Output  : none
Function: short press
Entry parameters: None
Return value: none
**************************************************************************/
static void PS2_ShortPoll(void)
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
static void PS2_EnterConfing(void)
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
static void PS2_TurnOnAnalogMode(void)
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
//static void PS2_VibrationMode(void)
//{
//    CS_L;
//    delay_us(16);
//    PS2_Cmd(0x01);
//    PS2_Cmd(0x4D);
//    PS2_Cmd(0X00);
//    PS2_Cmd(0x00);
//    PS2_Cmd(0X01);
//    CS_H;
//    delay_us(16);
//}
/**************************************************************************
Function: Complete and save the configuration
Input   : none
Output  : none
Function: Complete and save the configuration
Entry parameters: None
Return value: none
**************************************************************************/
static void PS2_ExitConfing(void)
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
static void PS2_SetInit(void)
{
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_EnterConfing();		  //Enter configuration mode
    PS2_TurnOnAnalogMode();	//The "traffic light" configuration mode and choose whether to save //"Traffic light" configuration mode, and choose whether to save
    //PS2_VibrationMode(); Open vibration mode // Enable vibration mode
    PS2_ExitConfing();		  //Complete and save the configuration // Complete and save configuration
}


