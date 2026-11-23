#ifndef __XBOX360_GAMEPAD_H
#define __XBOX360_GAMEPAD_H

#include <stdint.h>

#include "usbh_hid.h"
#include "usbh_hid_GamePad.h"
#include "bsp_gamepad.h"

//Handle product ID and serial number
#define Xbox360_Manufacturer "Xbox360"
#define Xbox360_SerialNum "5E904C0C"

//Handle products PID and VID
#define Xbox360_GamePad_VID 0x045E
#define Xbox360_GamePad_PID 0x028E

//Data length information for the handle
#define Xbox360_GamePad_DataLen 64

//15 key combinations
enum{
	Xbox360KEY_UP = 0, //D-pad axis: up
	Xbox360KEY_DOWN,   //D-pad axis: down
	Xbox360KEY_LEFT,   //D-pad axis: left
	Xbox360KEY_RIGHT,  //D-pad axis: right
	Xbox360KEY_Menu,   // Menu/star button
	Xbox360KEY_SELECT, // Window/select key
	Xbox360KEY_LJoy,   //Left analog stick button
	Xbox360KEY_RJoy,   //Right analog stick button
	Xbox360KEY_LB ,    //Upper left button LB
	Xbox360KEY_RB ,    //Press the RB button at the top right
	Xbox360KEY_HOME ,  //logo button Home
	Xbox360_PaddingBit,//Fill bits, used to align key functions; in practice, this bit is meaningless
	Xbox360KEY_A ,     //Button A
	Xbox360KEY_B ,     //Button B
	Xbox360KEY_X ,     //Button X
	Xbox360KEY_Y ,     //Button Y
	//Reserve trigger buttons
};

//Handle decoder function
void Xbox360_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);

//Provide handle objects externally
extern GamePadType_t Xbox360Gamepad;

#endif /* __Xbox360_GAMEPAD_H */
