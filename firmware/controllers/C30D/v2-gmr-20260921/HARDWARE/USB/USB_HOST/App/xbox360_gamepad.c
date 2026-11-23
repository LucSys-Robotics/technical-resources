#include "xbox360_gamepad.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"

//Internal function declaration
static void Xbox360_GamePad_SetVibration(uint8_t m1,uint8_t m2);
static GamePadKeyStateType_t Xbox360_GetKeyState(uint8_t bit);

//Xbox 360 game controller
GamePadType_t Xbox360Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 1,
	.SetVibration = Xbox360_GamePad_SetVibration,
	.getKeyState = Xbox360_GetKeyState
};


//Button callback function
__weak void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
//	printf("keyid:%d\tevent:%d\r\n",keyid,event);
//	
//	if( keyid == Xbox360KEY_SELECT && event == GamePadKeyEvent_LONGCLICK )
//		Xbox360Gamepad.SetVibration(255,0);
}

//Button value
static uint16_t Xbox360_GamePad_KeyOriginalVal = 0;

//Vibration value
volatile static uint8_t VibrationVal[2] = {0,0};

//Vibration duration
volatile static uint32_t VibrationTimes = 0;

//Start the vibration marker
volatile static uint8_t vibrationFlag = 0;

//Get the handle key-value status function
static GamePadKeyStateType_t Xbox360_GetKeyState(uint8_t bit)
{
	if( (Xbox360_GamePad_KeyOriginalVal>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}

//Set the handle vibration interface
static void Xbox360_GamePad_SetVibration(uint8_t m1,uint8_t m2)
{
	// Temporarily shield vibration; When restoring, delete the / * and * / outside the original function below.
	/*
	if( Xbox360Gamepad.Vib_EN == 0 ) return;
		
	vibrationFlag = 1;
	VibrationVal[0] = m1;
	VibrationVal[1] = m2;
	
	//Update vibration duration
	VibrationTimes = xTaskGetTickCount();
	*/
	(void)m1;
	(void)m2;
}


//Joystick value mapping
static uint8_t map_joystick(uint8_t joystick) {
    if (joystick == 0) {
        return 127;
    } else if (joystick <= 127) {
        return 127 + joystick;
    } else {
        return joystick - 128;
    }
}

//16 controller buttons
#define Xbox360_KEY_NUM 16 

//Defines the individual key values for key detection events
static GamePad_CheckEventType_t Xbox360_GamePadKeyCheckEvent[Xbox360_KEY_NUM] = { 0 };

//Provides a vibration interface externally
static void Xbox360GamePad_Vibration(USBH_HandleTypeDef *phost)
{
	if( vibrationFlag==0 ) return;
	
	static uint8_t lastvib[2] = { 0 };
	
	//Generate vibration data
	uint8_t vibration_data[8] = {0x00, 0x08, 0x00, VibrationVal[0], VibrationVal[1], 0x00, 0x00, 0x00};
	
	//Fixed value for stop vibration data
	uint8_t stop_data[8] = {0x00, 0x08, 0x00, 0, 0, 0x00, 0x00, 0x00};
	
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//Activate vibration
	USBH_InterruptSendData(phost, vibration_data, 8, HID_Handle->OutPipe);
	
	//A single vibration lasts 300ms, unless the mission continuously sets vibration
	if( xTaskGetTickCount() - VibrationTimes > 200 )
	{
		vibrationFlag = 0;
		vibration_data[3] = 0,vibration_data[4] = 0;
		for(uint8_t i=0;i<3;i++)
		{
			USBH_InterruptSendData(phost, stop_data, 8, HID_Handle->OutPipe);
			vTaskDelay(20);
		}
		lastvib[0]=0;
		lastvib[1]=0;
	}
	else
	{
		//Refresh only when data changes
		if( lastvib[0]!=VibrationVal[0] || lastvib[1]!=VibrationVal[1]  )
		{
			USBH_InterruptSendData(phost, vibration_data, 8, HID_Handle->OutPipe);
			vTaskDelay(20);
		}
		lastvib[0] = VibrationVal[0];
		lastvib[1] = VibrationVal[1];
	}
		
}

//Data decoding for Xbox 360 game controllers
void Xbox360_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
//	if( datalen!=Xbox360_GamePad_DataLen ) return;
	
	//Send stop vibration data at startup to avoid being reset or other accidents during vibration settings, which could cause the controller to vibrate continuously
	static uint8_t times = 0;
	if( times<2 )
	{
		times++;
		uint8_t vibration_data[8] = {0x00, 0x08, 0x00, 0, 0, 0x00, 0x00, 0x00};
		USBH_InterruptSendData(phost, vibration_data, 8, HID_Handle->OutPipe);
	}
	
	//Analog Travel
	Xbox360Gamepad.LX = map_joystick(buffer[7]);
	Xbox360Gamepad.LY = map_joystick(buffer[9]);
	Xbox360Gamepad.RX = map_joystick(buffer[11]);
	Xbox360Gamepad.RY = map_joystick(buffer[13]);
	Xbox360Gamepad.LT = buffer[4];
	Xbox360Gamepad.RT = buffer[5];

	/* Joystick value range
	      Y
	     254
	      ^
	      |
	0 <---------> 254 X
	      |
	      v
	      0
	*/
	
	//15 key values
	Xbox360_GamePad_KeyOriginalVal = (uint16_t)buffer[3]<<8 | buffer[2];
	
	if( gamepad_brand == PS2_USB_Wiredless )
	{
		if( Xbox360Gamepad.LT==0 ) Xbox360_GamePad_KeyOriginalVal &= ~(1<<Xbox360_PaddingBit);
		else Xbox360_GamePad_KeyOriginalVal |= (1<<Xbox360_PaddingBit);
	}
		
	//Button callback function triggered
	for (uint8_t key = Xbox360KEY_UP; key <= Xbox360KEY_Y; key++) 
	{
		//if (key == Xbox360_PaddingBit) continue;
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(Xbox360_GamePad_KeyOriginalVal,
                                		&Xbox360_GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//Trigger callback function
		Xbox360GamePad_KeyEvent_Callback(key,event);
	}
	
	//Set the vibration function
	Xbox360GamePad_Vibration(phost);
}

