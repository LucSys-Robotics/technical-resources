#ifndef __BSP_GAMEPAD_H
#define __BSP_GAMEPAD_H

#include <stdint.h>

//Gamepad button general events
typedef enum{
    GamePadKeyEvent_NONE,         //No key press events
    GamePadKeyEvent_SINGLECLICK,  //Click the event
    GamePadKeyEvent_DOUBLECLICK,  //Double-click event
    GamePadKeyEvent_LONGCLICK     //Long-press event
}GamePadKeyEventType_t;

//Gamepad universal key status
typedef enum{
	GamePadKeyState_Release = 0,  //Release
	GamePadKeyState_Press = 1     //Press
}GamePadKeyStateType_t;

//Generic gamepad button-detection state machine
typedef enum{
    WaitToPress = 0,
    WaitToRelease ,
    KEYPress    ,
    KEYUp       ,
    LONG_CLICK  ,
}GamePad_StateMachineType_t;

//Generic gamepad individual-button attributes
typedef struct 
{
    uint8_t keystate;
    uint32_t timebase;
    uint32_t statetime;
    GamePad_StateMachineType_t statemachine;
}GamePad_CheckEventType_t;

//Universal detection time for game controller buttons, unit: milliseconds
#define GamePad_LONGPRESS_TIEM 1000 //Long-press detection time
#define GamePad_CLICK_TIME     400  //Single or double tap to check the time
#define GamePad_KEYFILTER_TIME 50   //Button debounce time

//Generic gamepad object
typedef struct{
	uint8_t LX;  //Four directional joystick values, set to 0~255
	uint8_t LY;
	uint8_t RX;
	uint8_t RY;
	uint8_t LT;  //Two accelerator triggers on the left and right sides
	uint8_t RT;
	void (*SetVibration)(uint8_t m1,uint8_t m2);//Vibration interface
	GamePadKeyStateType_t (*getKeyState)(uint8_t keybit);
	uint8_t StartFlag;
	uint8_t Vib_EN;
}GamePadType_t;


//Game controller brand
typedef enum{
	UnKnown_Dev,          //Unknown device
	PS2_Classic,           //Classic PS2 controller
	PS2_USB_Wired,         //Wired PS2 controller
	PS2_USB_WiredV2,       //Second-generation wired PS2
	PS2_USB_Wiredless,    //Wireless PS2 controller
	Xbox360,    //Xbox 360 identifier
	SwitchPro,
	BeiTong,
}GamePadBrandType_t;

//Handle debugging structure
enum{
	EnumNULL = 0,
	EnumWait ,
	EnumDone ,
};
typedef struct{
	uint8_t ready;
	uint8_t enmu_state;
	GamePadBrandType_t type;
}GamePadDebugValType_t;

//Interface functions that provide key detection for gamepads
GamePadKeyEventType_t GamePadKey_CheckEvent(uint16_t keysource,GamePad_CheckEventType_t* key,uint8_t bit);

//Provides external specifications for game controller brands
extern GamePadBrandType_t gamepad_brand;

//Handle device insertion and pullback callback functions
void USB_GamePad_InsertCallback(void);
void USB_GamePad_PullOutCallback(void);

//Generic gamepad interface
extern GamePadType_t* GamePadInterface;
extern GamePadType_t GamePadDefalut;

//Generic gamepad debug interface
extern GamePadDebugValType_t GamePadDebug;

#endif /* __BSP_GAMEPAD_H */
