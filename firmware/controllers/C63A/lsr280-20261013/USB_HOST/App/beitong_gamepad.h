#ifndef __BeiTong_GAMEPAD_H
#define __BeiTong_GAMEPAD_H

#include <stdint.h>

#include "usbh_hid.h"
#include "usbh_hid_GamePad.h"
#include "bsp_gamepad.h"


//Handle products PID and VID
#define BeiTong_GamePad_VID 0x20BC
#define BeiTong_GamePad_PID 0x516A

#define BEITONG_KP20_XINPUT_VID 0x20BCU
#define BEITONG_KP20_XINPUT_PID 0x5128U

//Data length information for the handle
#define BeiTong_GamePad_DataLen 64

//15 key combinations
//Button multiplexing using Xbox 360 style
//enum{
//	Xbox360KEY_UP = 0, // Axis cross - up
//	Xbox360KEY_DOWN, // Pivot cross - down
//	Xbox360KEY_LEFT, // Pivot cross - left
//	Xbox360KEY_RIGHT, // Pivot cross - right
//	Xbox360KEY_Menu, // Menu/star key
//	Xbox360KEY_SELECT, // Window/select key
//	Xbox360KEY_LJoy, // Left stick button
//	Xbox360KEY_RJoy, // Right stick button
//	Xbox360KEY_LB , // Upper left key LB
//	Xbox360KEY_RB , // Upper right key RB
//	Xbox360KEY_HOME , // logo key Home
//	Xbox360_PaddingBit, // fills bits, used to align key functions, but actually this bit is meaningless
//	Xbox360KEY_A , // Press key A
//	Xbox360KEY_B , // Press key B
//	Xbox360KEY_X , // Press X key
//	Xbox360KEY_Y , // Press the Y key
//	//Reserve trigger buttons
//};

//Handle decoder function
void BeiTong_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);

//Provide handle objects externally
extern GamePadType_t BeiTongGamepad;

#endif /* __Xbox360_GAMEPAD_H */
