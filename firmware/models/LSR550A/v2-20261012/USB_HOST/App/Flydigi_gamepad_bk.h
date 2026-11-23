#ifndef __FLYDIGI_GAMEPAD_H
#define __FLYDIGI_GAMEPAD_H

#include <stdint.h>

#include "usbh_hid.h"
#include "usbh_hid_GamePad.h"
#include "bsp_gamepad.h"

//Handle product ID and serial number
#define FLYDIGI_Manufacturer "Flydigi"
#define FLYDIGI_SerialNum "5E904C0C"

//Handle products PID and VID
#define FLYDIGI_GamePad_VID 0x045E
#define FLYDIGI_GamePad_PID 0x028E

//Data length information for the handle
#define FLYDIGI_GamePad_DataLen 64

//15 key combinations
enum{
	FlydigiKEY_UP = 0, //D-pad axis: up
	FlydigiKEY_DOWN,   //D-pad axis: down
	FlydigiKEY_LEFT,   //D-pad axis: left
	FlydigiKEY_RIGHT,  //D-pad axis: right
	FlydigiKEY_Menu,   // Menu/star button
	FlydigiKEY_SELECT, // Window/select key
	FlydigiKEY_LJoy,   //Left analog stick button
	FlydigiKEY_RJoy,   //Right analog stick button
	FlydigiKEY_LB ,    //Upper left button LB
	FlydigiKEY_RB ,    //Press the RB button at the top right
	FlydigiKEY_HOME ,  //logo button Home
	Flydigi_PaddingBit,//Fill bits, used to align key functions; in practice, this bit is meaningless
	FlydigiKEY_A ,     //Button A
	FlydigiKEY_B ,     //Button B
	FlydigiKEY_X ,     //Button X
	FlydigiKEY_Y ,     //Button Y
	//Reserve trigger buttons
};

//Handle decoder function
void Flydigi_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);

//Provide handle objects externally
extern GamePadType_t flydigiGamepad;

#endif /* __FLYDIGI_GAMEPAD_H */
