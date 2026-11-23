#include "ps2_task.h"

/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
Functions: PS2 controller tasks
Entry parameters: None
Return value: none
**************************************************************************/
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(PS2_TASK_RATE));
		
		if( SysVal.HardWare_Ver == V1_0 ) // V1.0 hardware version uses a standard PS2 controller
		{
			//Read the ps2 data
			//Read data from the PS2
			PS2_Read();		
		}
		else if( SysVal.HardWare_Ver >= V1_1 ) //V1.1 hardware version uses a USB controller
		{
			//V1.1 hardware uses USB handles; read tasks are not included in this function
		}
		
		//Game controller mode starts
		if( GamePadInterface->StartFlag == 1 && Get_Control_Mode(_PS2_Control)==0 && SysVal.Time_count>=CONTROL_DELAY && GamePadInterface->LY > 150 )
			Set_Control_Mode(_PS2_Control);
		
    }
}

