#include "xbox360_gamepad.h"
#include "FreeRTOS.h"
#include "task.h"

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

//Nonblocking vibration state
#define XBOX360_VIBRATION_CMD_NONE   0U
#define XBOX360_VIBRATION_CMD_START  1U
#define XBOX360_VIBRATION_CMD_STOP   2U
#define XBOX360_VIBRATION_RETRY_MS   4U
#define XBOX360_VIBRATION_MAX_RETRY  3U
#define XBOX360_VIBRATION_UPDATE_MS  50U

//Asynchronous USB OUT must use static buffers
static uint8_t VibrationOutBuffer[8] = {0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00};

static uint8_t VibrationOutBusy = 0;
static uint8_t VibrationOutCommand = XBOX360_VIBRATION_CMD_NONE;
static uint8_t VibrationMotorRunning = 0;
static uint8_t VibrationRetryCount = 0;
static uint8_t VibrationLastVal[2] = {0,0};
static uint32_t VibrationRetryTime = 0;
static uint32_t VibrationLastSubmitTime = 0;

//Get the handle key-value status function
static GamePadKeyStateType_t Xbox360_GetKeyState(uint8_t bit)
{
	if( (Xbox360_GamePad_KeyOriginalVal>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}

//Set the handle vibration interface
//Only update target values here; do not transmit USB data or wait directly in the caller's task
static void Xbox360_GamePad_SetVibration(uint8_t m1,uint8_t m2)
{
	if( Xbox360Gamepad.Vib_EN == 0 ) return;

	VibrationVal[0] = m1;
	VibrationVal[1] = m2;
	VibrationTimes = xTaskGetTickCount();

	if( m1 == 0 && m2 == 0 )
		vibrationFlag = 0;
	else
		vibrationFlag = 1;
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

//When the handle is reconnected, it resets to a non-blocking vibration state
void Xbox360_GamePad_VibrationReset(void)
{
	VibrationVal[0] = 0;
	VibrationVal[1] = 0;
	VibrationTimes = 0;
	vibrationFlag = 0;

	VibrationOutBusy = 0;
	VibrationOutCommand = XBOX360_VIBRATION_CMD_NONE;
	VibrationMotorRunning = 0;
	VibrationRetryCount = 0;
	VibrationLastVal[0] = 0;
	VibrationLastVal[1] = 0;
	VibrationRetryTime = 0;
	VibrationLastSubmitTime = 0;

	VibrationOutBuffer[0] = 0x00;
	VibrationOutBuffer[1] = 0x08;
	VibrationOutBuffer[2] = 0x00;
	VibrationOutBuffer[3] = 0x00;
	VibrationOutBuffer[4] = 0x00;
	VibrationOutBuffer[5] = 0x00;
	VibrationOutBuffer[6] = 0x00;
	VibrationOutBuffer[7] = 0x00;
}

//Nonblocking background vibration processing
void Xbox360_GamePad_VibrationProcess(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle;
	USBH_URBStateTypeDef urb_state;

	if( phost == NULL || phost->pActiveClass == NULL || phost->pActiveClass->pData == NULL ) return;

	HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	if( HID_Handle->OutPipe == 0U ) return;

	//Each vibration lasts 200ms; external re-requests will update the time
	if( vibrationFlag != 0 &&
		(xTaskGetTickCount() - VibrationTimes) > 200U )
	{
		vibrationFlag = 0;
		VibrationVal[0] = 0;
		VibrationVal[1] = 0;
	}

	//When the previous OUT has already been submitted, it must wait for completion and cannot overwrite the same Pipe
	if( VibrationOutBusy != 0 )
	{
		urb_state = USBH_LL_GetURBState(phost, HID_Handle->OutPipe);

		if( urb_state == USBH_URB_DONE )
		{
			VibrationOutBusy = 0;
			VibrationRetryCount = 0;

			if( VibrationOutCommand == XBOX360_VIBRATION_CMD_START )
			{
				VibrationLastVal[0] = VibrationOutBuffer[3];
				VibrationLastVal[1] = VibrationOutBuffer[4];
				VibrationMotorRunning = 1;
			}
			else if( VibrationOutCommand == XBOX360_VIBRATION_CMD_STOP )
			{
				VibrationLastVal[0] = 0;
				VibrationLastVal[1] = 0;
				VibrationMotorRunning = 0;
			}

			VibrationOutCommand = XBOX360_VIBRATION_CMD_NONE;
		}
		else if( urb_state == USBH_URB_NOTREADY ||
				 urb_state == USBH_URB_STALL ||
				 urb_state == USBH_URB_ERROR )
		{
			VibrationOutBusy = 0;
			VibrationRetryCount++;
			VibrationRetryTime = phost->Timer;

			if( VibrationRetryCount >= XBOX360_VIBRATION_MAX_RETRY )
			{
				VibrationOutCommand = XBOX360_VIBRATION_CMD_NONE;
				VibrationMotorRunning = 0;
				vibrationFlag = 0;
				VibrationRetryCount = 0;
				return;
			}
		}
		else
		{
			return;
		}
	}

	//Command retention after failure is retained and retried after a 4ms interval
	if( VibrationOutCommand != XBOX360_VIBRATION_CMD_NONE )
	{
		if( (phost->Timer - VibrationRetryTime) < XBOX360_VIBRATION_RETRY_MS )
			return;
	}
	else
	{
		if( vibrationFlag != 0 &&
			(VibrationMotorRunning == 0 ||
			 VibrationLastVal[0] != VibrationVal[0] ||
			 VibrationLastVal[1] != VibrationVal[1]) )
		{
			//During motor operation, the speed is updated as fast as every 50ms
			if( VibrationMotorRunning != 0 &&
				(phost->Timer - VibrationLastSubmitTime) < XBOX360_VIBRATION_UPDATE_MS )
				return;

			VibrationOutCommand = XBOX360_VIBRATION_CMD_START;
			VibrationOutBuffer[3] = VibrationVal[0];
			VibrationOutBuffer[4] = VibrationVal[1];
		}
		else if( vibrationFlag == 0 && VibrationMotorRunning != 0 )
		{
			VibrationOutCommand = XBOX360_VIBRATION_CMD_STOP;
			VibrationOutBuffer[3] = 0;
			VibrationOutBuffer[4] = 0;
		}
		else
		{
			return;
		}

		VibrationRetryCount = 0;
	}

	USBH_InterruptSendData(phost,
						  VibrationOutBuffer,
						  8,
						  HID_Handle->OutPipe);

	VibrationOutBusy = 1;
	VibrationRetryTime = phost->Timer;
	VibrationLastSubmitTime = phost->Timer;
}

//Data decoding for Xbox 360 game controllers
void Xbox360_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	(void)phost;
	
//	if( datalen!=Xbox360_GamePad_DataLen ) return;
	
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
	
	//USB Host calls Xbox360_GamePad_VibrationProcess in the background to process vibration
}

