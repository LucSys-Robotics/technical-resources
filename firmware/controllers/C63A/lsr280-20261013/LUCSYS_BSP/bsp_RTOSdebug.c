#include "bsp_RTOSdebug.h"
#include <math.h>
#include "gpio.h"


//Timer frequency, unit in Hz. Modified according to configuration
#define TIM_FREQ 100000

static uint16_t get_TickCount(void)
{
    //Method to obtain count values. Note that this value depends on the timer configuration; the current setting is 0.01ms count 1
    return TIM6->CNT;
}

//Methods to get debug timing
static void get_StartCount(RTOSDebugPrivateVar* priv_var)
{
    //0 replaced by the method of reading the count
    priv_var->TickLast = get_TickCount();
}

//Method to obtain frequency. Note: Too slow frequency cannot be detected; only one timer data overflow case is handled here
static uint16_t get_Freq(RTOSDebugPrivateVar* priv_var)
{
    priv_var->countState = !priv_var->countState;

    if( 1 == priv_var->countState )
        priv_var->TickLast = get_TickCount(),//Save the count once
        priv_var->TaskFreq = priv_var->LastFreq;
    else
    {
        //Enter the second time and start calculating frequency
        priv_var->TickNow = get_TickCount();
        if( priv_var->TickNow < priv_var->TickLast )
        {   //Numeric overflow
            priv_var->TaskFreq = priv_var->TickNow + 0xFFFF - priv_var->TickLast;
        }
        else
            priv_var->TaskFreq = priv_var->TickNow - priv_var->TickLast;

        priv_var->TaskFreq = round((float)TIM_FREQ/(float)priv_var->TaskFreq);
        priv_var->LastFreq = priv_var->TaskFreq;
    }

    return priv_var->TaskFreq;
}

//Method to obtain the function runtime; note that the timing start side uses the TickStart method to save the count value once
static float get_UsedTime(RTOSDebugPrivateVar* priv_var)
{
    //Get the count
    priv_var->TickNow = get_TickCount();

    //If the previous data is larger than the value obtained this time, it indicates count overflow
    if( priv_var->TickLast > priv_var->TickNow )  
    {
        priv_var->UseTime = priv_var->TickNow + 0xFFFF - priv_var->TickLast;
    }
    else
        priv_var->UseTime = priv_var->TickNow - priv_var->TickLast;
    
    priv_var->UseTime = ((float)priv_var->UseTime/(float)TIM_FREQ)*1000;//Convert output to ms

    return priv_var->UseTime;
}

//Attach driver
RtosDebugInterface_t RTOSDebugTimer = {
    .TickStart = get_StartCount,
    .UpdateFreq = get_Freq,
    .UpdateUsedTime = get_UsedTime,
};

