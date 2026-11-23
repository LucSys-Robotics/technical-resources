#include "WiredPS2_gamepad.h"
#include "FreeRTOS.h"
#include "task.h"

__weak void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	printf("%d,%d\r\n",keyid,event);
}

static GamePadKeyStateType_t WiredPS2_GetKeyState(uint8_t bit);

//Wired PS2 game controller
GamePadType_t Wired_USB_PS2Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 0,
	.SetVibration = NULL,
	.getKeyState = WiredPS2_GetKeyState
};

//Button value
static uint16_t GamePad_KeyOriginalVal = 0;

//Get the handle key-value status function
static GamePadKeyStateType_t WiredPS2_GetKeyState(uint8_t bit)
{
	if( (GamePad_KeyOriginalVal>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}


//Defines the individual key values for key detection events
static GamePad_CheckEventType_t GamePadKeyCheckEvent[16] = { 0 };


//Flag setting function to assist PS2 controller decoding
static void ps2_set_bit(uint16_t* state,uint8_t state_bit,uint8_t bit)
{
	if(state_bit==1) //Set the specified bit to 1, leaving all other bits unchanged
	{
		*state |= (1U<<bit);
	}
	else //Set the specified bit to 0, leaving all other bits unchanged
	{
		*state &= ~(1U<<bit);
	}
}

//Data decoding
void Wired_USB_PS2gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	(void)phost;
	
	uint8_t tmp_bool = 0 ;
	
//	static uint8_t times = 0;
//	if( times<2 )
//	{
//		times++;
//		uint8_t vibration_data[2] = {1,1};
//		USBH_InterruptSendData(phost, vibration_data, 2, HID_Handle->OutPipe);
//	}
	
	Wired_USB_PS2Gamepad.LX = buffer[3];
	Wired_USB_PS2Gamepad.LY = 255-buffer[4];
	Wired_USB_PS2Gamepad.RX = buffer[1];
	Wired_USB_PS2Gamepad.RY = buffer[2];
	
	tmp_bool = (buffer[6]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,0); //Seltec Key Select the key
	
	tmp_bool = (buffer[6]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,1); //Left analog stick button
	
	tmp_bool = (buffer[6]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,2); //Right analog stick button
	
	tmp_bool = (buffer[6]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,3); //start
	
	tmp_bool = buffer[5]&0x0F;//Remove the lower 4 bits
	if(tmp_bool==0x0F)//No buttons pressed
	{
		ps2_set_bit(&GamePad_KeyOriginalVal,0,4); //¡ü
		ps2_set_bit(&GamePad_KeyOriginalVal,0,5); //¡ú
		ps2_set_bit(&GamePad_KeyOriginalVal,0,6); //¡ý
		ps2_set_bit(&GamePad_KeyOriginalVal,0,7); //¡û
	}
	else if( (tmp_bool&0x01)==0 )
	{	
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://¡ü
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //¡ü
				break;
			case 0x01://¡ú
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //¡ú
				break;
			case 0x02://¡ý
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //¡ý
				break;
			case 0x03://¡û
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //¡û
				break;
			default:
				break;
		}
	}
	else if( (tmp_bool&0x01)==1 ) //First bit is 1 when two buttons on the left pad are pressed
	{
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://¡ü¡ú
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4);//¡ü
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //¡ú
				break;
			case 0x01://¡ý¡ú
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //¡ý
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //¡ú
				break;
			case 0x02://¡ý¡û
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //¡ý
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //¡û
				break;
			case 0x03://¡ü¡û
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //¡ü
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //¡û
				break;
			default:
				break;
		}
	}
	
	tmp_bool = (buffer[6]>>2)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,8); //Left trigger No. 2
	if( tmp_bool ) Wired_USB_PS2Gamepad.LT = 255;
	else Wired_USB_PS2Gamepad.LT = 0;
	
	tmp_bool = (buffer[6]>>3)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,9); //Right trigger No. 2
	if( tmp_bool ) Wired_USB_PS2Gamepad.RT = 255;
	else Wired_USB_PS2Gamepad.RT = 0;
	
	tmp_bool = (buffer[6]>>0)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,10); //Left trigger No. 1
	
	tmp_bool = (buffer[6]>>1)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,11); //Right trigger No. 1

	tmp_bool = (buffer[5]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,12); //Number One, Green GREEN
	
	tmp_bool = (buffer[5]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,13); //Number two, red RED

	tmp_bool = (buffer[5]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,14); //No. 3, Bluetooth BLUE
	
	tmp_bool = (buffer[5]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,15); //Number four, pink PINK
	
	//Button callback function triggered
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(GamePad_KeyOriginalVal,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//Trigger callback function
		Wired_USB_PS2GamePad_KeyEvent_Callback(key,event);
	}

}

//Data decoding
void Wired_USB_V2_PS2gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	(void)phost;
	
	uint8_t tmp_bool = 0 ;
	
//	static uint8_t times = 0;
//	if( times<2 )
//	{
//		times++;
//		uint8_t vibration_data[2] = {1,1};
//		USBH_InterruptSendData(phost, vibration_data, 2, HID_Handle->OutPipe);
//	}
	
	Wired_USB_PS2Gamepad.LX = buffer[0];
	Wired_USB_PS2Gamepad.LY = 255-buffer[1];
	Wired_USB_PS2Gamepad.RX = buffer[3];
	Wired_USB_PS2Gamepad.RY = buffer[4];
	
	tmp_bool = (buffer[6]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,0); //Seltec Key Select the key
	
	tmp_bool = (buffer[6]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,1); //Left analog stick button
	
	tmp_bool = (buffer[6]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,2); //Right analog stick button
	
	tmp_bool = (buffer[6]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,3); //start
	
	tmp_bool = buffer[5]&0x0F;//Remove the lower 4 bits
	if(tmp_bool==0x0F)//No buttons pressed
	{
		ps2_set_bit(&GamePad_KeyOriginalVal,0,4); //¡ü
		ps2_set_bit(&GamePad_KeyOriginalVal,0,5); //¡ú
		ps2_set_bit(&GamePad_KeyOriginalVal,0,6); //¡ý
		ps2_set_bit(&GamePad_KeyOriginalVal,0,7); //¡û
	}
	else if( (tmp_bool&0x01)==0 )
	{	
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://¡ü
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //¡ü
				break;
			case 0x01://¡ú
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //¡ú
				break;
			case 0x02://¡ý
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //¡ý
				break;
			case 0x03://¡û
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //¡û
				break;
			default:
				break;
		}
	}
	else if( (tmp_bool&0x01)==1 ) //First bit is 1 when two buttons on the left pad are pressed
	{
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://¡ü¡ú
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4);//¡ü
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //¡ú
				break;
			case 0x01://¡ý¡ú
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //¡ý
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //¡ú
				break;
			case 0x02://¡ý¡û
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //¡ý
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //¡û
				break;
			case 0x03://¡ü¡û
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //¡ü
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //¡û
				break;
			default:
				break;
		}
	}
	
	tmp_bool = (buffer[6]>>2)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,8); //Left trigger No. 2
	if( tmp_bool ) Wired_USB_PS2Gamepad.LT = 255;
	else Wired_USB_PS2Gamepad.LT = 0;
	
	tmp_bool = (buffer[6]>>3)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,9); //Right trigger No. 2
	if( tmp_bool ) Wired_USB_PS2Gamepad.RT = 255;
	else Wired_USB_PS2Gamepad.RT = 0;
	
	tmp_bool = (buffer[6]>>0)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,10); //Left trigger No. 1
	
	tmp_bool = (buffer[6]>>1)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,11); //Right trigger No. 1

	tmp_bool = (buffer[5]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,12); //Number One, Green GREEN
	
	tmp_bool = (buffer[5]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,13); //Number two, red RED

	tmp_bool = (buffer[5]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,14); //No. 3, Bluetooth BLUE
	
	tmp_bool = (buffer[5]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,15); //Number four, pink PINK
	
	//Button callback function triggered
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(GamePad_KeyOriginalVal,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//Trigger callback function
		Wired_USB_PS2GamePad_KeyEvent_Callback(key,event);
	}

}





