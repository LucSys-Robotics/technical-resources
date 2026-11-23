#include "xbox360_gamepad.h"
#include "beitong_gamepad.h"
#include "FreeRTOS.h"
#include "task.h"


static void BeiTong_GamePad_SetVibration(uint8_t m1,uint8_t m2);
static GamePadKeyStateType_t BeiTong_GetKeyState(uint8_t bit);
static void BeiTongGamePad_Vibration(USBH_HandleTypeDef *phost);
static uint8_t BeiTong_MapJoystick(uint8_t joystick,uint8_t reverse);


GamePadType_t BeiTongGamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 1,
	.SetVibration = BeiTong_GamePad_SetVibration,
	.getKeyState = BeiTong_GetKeyState
};



__weak void BeiTongGamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
//	printf("keyid:%d\tevent:%d\r\n",keyid,event);
//	
//	if( keyid == Xbox360KEY_SELECT && event == GamePadKeyEvent_LONGCLICK )
//		BeiTongGamepad.SetVibration(255,0);
}


static uint16_t BeiTong_GamePad_KeyOriginalVal = 0;


volatile static uint8_t VibrationVal[2] = {0,0};


volatile static uint32_t VibrationTimes = 0;


volatile static uint8_t vibrationFlag = 0;


static GamePadKeyStateType_t BeiTong_GetKeyState(uint8_t bit)
{
	if( (BeiTong_GamePad_KeyOriginalVal>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}


static void BeiTong_GamePad_SetVibration(uint8_t m1,uint8_t m2)
{
	if( BeiTongGamepad.Vib_EN == 0 ) return;
		
	vibrationFlag = 1;
	VibrationVal[0] = m1;
	VibrationVal[1] = m2;
	
	//������ʱ��
	VibrationTimes = xTaskGetTickCount();
}



// Convert BeiTong raw axis to the same final axis value used by Xbox360.
static uint8_t BeiTong_MapJoystick(uint8_t joystick,uint8_t reverse)
{
	uint8_t value = reverse ? (uint8_t)(255 - joystick) : joystick;
	
	if( value > 127 ) value--;
	
	return value;
}

#define BeiTong_KEY_NUM 16 
#define BEITONG_PARSE_DEBUG 0

#if BEITONG_PARSE_DEBUG
static const char *BeiTong_KeyName[BeiTong_KEY_NUM] = {
	"UP", "DOWN", "LEFT", "RIGHT",
	"MENU", "SELECT", "LJOY", "RJOY",
	"LB", "RB", "HOME", "PAD",
	"A", "B", "X", "Y"
};
#endif

static GamePad_CheckEventType_t BeiTong_GamePadKeyCheckEvent[BeiTong_KEY_NUM] = { 0 };


static void BeiTongGamePad_Vibration(USBH_HandleTypeDef *phost)
{
	if( vibrationFlag==0 ) return;
	
	static uint8_t lastvib[2] = { 0 };
	
	uint8_t vibration_data[8] = {0x00, 0x08, 0x00, VibrationVal[0], VibrationVal[1], 0x00, 0x00, 0x00};
	
	uint8_t stop_data[8] = {0x00, 0x08, 0x00, 0, 0, 0x00, 0x00, 0x00};
	
    HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	USBH_InterruptSendData(phost, vibration_data, 8, HID_Handle->OutPipe);
	
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
		if( lastvib[0]!=VibrationVal[0] || lastvib[1]!=VibrationVal[1]  )
		{
			USBH_InterruptSendData(phost, vibration_data, 8, HID_Handle->OutPipe);
			vTaskDelay(20);
		}
		lastvib[0] = VibrationVal[0];
		lastvib[1] = VibrationVal[1];
	}
		
}

void BeiTong_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	uint16_t keyOriginalVal = 0;
	
//	for(uint8_t i=0;i<datalen;i++)
//		printf("%d ",buffer[i]);
//	printf("\r\n");

	if( datalen <= 10 ) return;

	static uint8_t times = 0;
	if( times<2 )
	{
		times++;
		uint8_t vibration_data[8] = {0x00, 0x08, 0x00, 0, 0, 0x00, 0x00, 0x00};
		USBH_InterruptSendData(phost, vibration_data, 8, HID_Handle->OutPipe);
	}
	
	BeiTongGamepad.LX = BeiTong_MapJoystick(buffer[5],0);
	BeiTongGamepad.LY = BeiTong_MapJoystick(buffer[6],1);
	BeiTongGamepad.RX = BeiTong_MapJoystick(buffer[7],0);
	BeiTongGamepad.RY = BeiTong_MapJoystick(buffer[8],1);
	BeiTongGamepad.LT = buffer[9];
	BeiTongGamepad.RT = buffer[10];

	/* 
	      Y
	     254
	      ^
	      |
	0 <---------> 254 X
	      |
	      v
	      0
	*/
	
	switch( buffer[2]&0x0F )
	{
		case 0x00:
			keyOriginalVal |= (1U<<Xbox360KEY_UP);
			break;
		case 0x01:
			keyOriginalVal |= (1U<<Xbox360KEY_UP);
			keyOriginalVal |= (1U<<Xbox360KEY_RIGHT);
			break;
		case 0x02:
			keyOriginalVal |= (1U<<Xbox360KEY_RIGHT);
			break;
		case 0x03:
			keyOriginalVal |= (1U<<Xbox360KEY_RIGHT);
			keyOriginalVal |= (1U<<Xbox360KEY_DOWN);
			break;
		case 0x04:
			keyOriginalVal |= (1U<<Xbox360KEY_DOWN);
			break;
		case 0x05:
			keyOriginalVal |= (1U<<Xbox360KEY_DOWN);
			keyOriginalVal |= (1U<<Xbox360KEY_LEFT);
			break;
		case 0x06:
			keyOriginalVal |= (1U<<Xbox360KEY_LEFT);
			break;
		case 0x07:
			keyOriginalVal |= (1U<<Xbox360KEY_LEFT);
			keyOriginalVal |= (1U<<Xbox360KEY_UP);
			break;
		default:
			break;
	}
	
	if( (buffer[3]>>0)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_A);
	if( (buffer[3]>>1)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_B);
	if( (buffer[3]>>3)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_X);
	if( (buffer[3]>>4)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_Y);
	if( (buffer[3]>>6)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_LB);
	if( (buffer[3]>>7)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_RB);
	
	if( (buffer[4]>>2)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_SELECT);
	if( (buffer[4]>>3)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_Menu);
	if( (buffer[4]>>4)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_HOME);
	if( (buffer[4]>>5)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_LJoy);
	if( (buffer[4]>>6)&0x01 ) keyOriginalVal |= (1U<<Xbox360KEY_RJoy);
	
	BeiTong_GamePad_KeyOriginalVal = keyOriginalVal;

#if BEITONG_PARSE_DEBUG
	printf("BT_PARSE LX:%u LY:%u RX:%u RY:%u LT:%u RT:%u HAT:%u KEY:0x%04X PRESS:",
		(unsigned int)BeiTongGamepad.LX, (unsigned int)BeiTongGamepad.LY,
		(unsigned int)BeiTongGamepad.RX, (unsigned int)BeiTongGamepad.RY,
		(unsigned int)BeiTongGamepad.LT, (unsigned int)BeiTongGamepad.RT,
		(unsigned int)(buffer[2] & 0x0FU), (unsigned int)BeiTong_GamePad_KeyOriginalVal);

	uint8_t pressCount = 0;
	for (uint8_t key = Xbox360KEY_UP; key <= Xbox360KEY_Y; key++)
	{
		if (key == Xbox360_PaddingBit) continue;
		if (((BeiTong_GamePad_KeyOriginalVal >> key) & 0x01U) != 0U)
		{
			printf(" %s", BeiTong_KeyName[key]);
			pressCount++;
		}
	}
	if (pressCount == 0U) printf(" NONE");
	printf("\r\n");
#endif	
	for (uint8_t key = Xbox360KEY_UP; key <= Xbox360KEY_Y; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(BeiTong_GamePad_KeyOriginalVal,
									&BeiTong_GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		

		BeiTongGamePad_KeyEvent_Callback(key,event);
	}
	
	BeiTongGamePad_Vibration(phost);
}


