#include "bsp_gamepad.h"

#include "FreeRTOS.h"
#include "task.h"


GamePadDebugValType_t GamePadDebug = { 0 };

//Define the game controller brand
GamePadBrandType_t gamepad_brand = UnKnown_Dev;

extern void OLED_ShowGamePadState(void);

//Handle device insert function_task environment
void USB_GamePad_InsertCallback(void)
{
//	OLED_ShowGamePadState();
	GamePadDebug.enmu_state = EnumWait;
//	vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1000ms for OLED display
	GamePadDebug.enmu_state = EnumDone;
	GamePadDebug.type = gamepad_brand;
}

//Handle device ejected _task environment
void USB_GamePad_PullOutCallback(void)
{
	//Designated as an unknown device
	gamepad_brand = UnKnown_Dev;
	
	//Reset the current controller joystick value to prevent it from being recognized next time
	GamePadInterface->LX = 127;
	GamePadInterface->RX = 127;
	GamePadInterface->LY = 127;
	GamePadInterface->RY = 127;
	GamePadInterface->LT = 0;
	GamePadInterface->RT = 0;
	
	//Switch back to the default interface to prevent control signals
	GamePadInterface = &GamePadDefalut;
	
	//Debugging related matters
	GamePadDebug.enmu_state = EnumNULL;
	GamePadDebug.type = gamepad_brand;
	GamePadDebug.ready = 0;
}

// The neutral gamepad reports every key released when no USB pad is connected.
static GamePadKeyStateType_t DefaultGamePad_GetKeyState(uint8_t keybit)
{
	(void)keybit;
	return GamePadKeyState_Release;
}

GamePadType_t GamePadDefalut = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 1,
	.SetVibration = 0,
	.getKeyState = DefaultGamePad_GetKeyState
};

//Generic gamepad object
GamePadType_t* GamePadInterface = &GamePadDefalut;

//Entry parameters: original key value, key attributes, and the keys to be detected
GamePadKeyEventType_t GamePadKey_CheckEvent(uint16_t keysource,GamePad_CheckEventType_t* key,uint8_t bit)
{
    //Read the corresponding key-value status
    key->keystate = (keysource>>bit)&0x01;

    switch (key->statemachine)
    {
        case WaitToPress:
            if( GamePadKeyState_Press == key->keystate )
            {
                key->timebase = xTaskGetTickCount();
                key->statemachine = KEYPress;
            } 
            break;
        case KEYPress:
            //Counts the time after the first key press (natural looping when no symbol overflows. No need to check for overflow)
            key->statetime = xTaskGetTickCount() - key->timebase;

            //Check if the keys are loose
            if( GamePadKeyState_Release == key->keystate )
            {
                //If the time spent pressing a key is too short, ignore it. This serves as a filtering effect
                if( key->statetime < GamePad_KEYFILTER_TIME ) key->statemachine = WaitToPress;
                else
                {
                   key->timebase = xTaskGetTickCount();//Update time base for detection of the next state
                   key->statemachine = KEYUp;    //Button pressed for a certain time and released, enter the next detection state
                }
            }
            else if( key->statetime > GamePad_LONGPRESS_TIEM ) 
            {   //Button not released and held for a certain time counts as a long press
                key->statemachine = LONG_CLICK;
            }

            break;
        case KEYUp:
            //Counts the time after the first key press (natural looping when no symbol overflows. No need to check for overflow)
            key->statetime = xTaskGetTickCount() - key->timebase;

            if( GamePadKeyState_Press == key->keystate && key->statetime < GamePad_CLICK_TIME && key->statetime > GamePad_KEYFILTER_TIME )
            {
                key->statemachine = WaitToRelease;
                return GamePadKeyEvent_DOUBLECLICK;
            }
            else if( key->statetime >= GamePad_CLICK_TIME )
            {
                key->statemachine = WaitToRelease;
                return GamePadKeyEvent_SINGLECLICK;
            }
            break;
        case LONG_CLICK:
            key->statemachine = WaitToRelease;
            return GamePadKeyEvent_LONGCLICK;
        case WaitToRelease:
            //After button detection is complete, wait for user to release the button to restore the state machine status
            if( GamePadKeyState_Release == key->keystate ) key->statemachine = WaitToPress;
            break;
        default:
            break;
    }

    return GamePadKeyEvent_NONE;
}

