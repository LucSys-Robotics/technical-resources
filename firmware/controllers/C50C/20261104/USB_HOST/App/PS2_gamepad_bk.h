#ifndef __PS2_GAMEPAD_H
#define __PS2_GAMEPAD_H

#include <stdint.h>

#include "bsp_gamepad.h"

//PS2 key position enumeration (bit0~bit15 correspond to 0~15 below)
enum 
{
	PS2KEY_SELECT	   = 0, //Select button
	PS2KEY_LROCKER      , //Press the key value with the left or right joysticks
	PS2KEY_RROCKER      ,
	PS2KEY_START        , //Press the Start button
	PS2KEY_UP           , //Left key area
	PS2KEY_RIGHT        ,
	PS2KEY_DOWN         ,
	PS2KEY_LEFT         ,
	PS2KEY_L2           ,	//Left and right trigger key values
	PS2KEY_R2           ,
	PS2KEY_L1           ,  
	PS2KEY_R1           ,
	PS2KEY_1GREEN       , //Right key area
	PS2KEY_2RED         , 
	PS2KEY_3BLUE        , 
	PS2KEY_4PINK           
};

//Data decoding function
void PS2_Wired_Decode(const uint8_t *data,uint8_t datalen);
void PS2_Wiredless_Android_Decode(const uint8_t *data,uint8_t datalen);
void PS2_Wiredless_PC_Decode(const uint8_t *data,uint8_t datalen);

//Interfaces provided externally
extern GamePadType_t ps2_gamepad;

#endif /* __PS2_GAMEPAD_H */

