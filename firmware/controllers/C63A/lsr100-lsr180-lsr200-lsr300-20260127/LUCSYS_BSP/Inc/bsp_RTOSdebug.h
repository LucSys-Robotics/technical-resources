#ifndef __BSP_RTOSDEBUG_H
#define __BSP_RTOSDEBUG_H
#include <stdint.h>

/*

Instructions for Use:
1. Before using this feature, please provide the variable RtosDebugPrivateVar task1debugVar; Variable names are optional
2. Using the interface: pRtosDebugInterface_t task1debug = &RTOSTaskDebug;
3. Calculating the task frequency for a task: task1freq = task1debug->UpdateFreq(&task1debugVar); The returned value obtained is the frequency of that task

*/

typedef struct {
    uint16_t TickNow;
    uint16_t TickLast;
    float UseTime;
    uint16_t TaskFreq;
    uint16_t LastFreq;
    uint8_t countState;
}RTOSDebugPrivateVar;

typedef struct {
    void (*TickStart)(RTOSDebugPrivateVar* priv_var);
    uint16_t (*UpdateFreq)(RTOSDebugPrivateVar* priv_var);
    float (*UpdateUsedTime)(RTOSDebugPrivateVar* priv_var);
}RtosDebugInterface_t,*pRTOSDebugInterface_t;

extern RtosDebugInterface_t RTOSDebugTimer;

#endif /* __BSP_RTOSDEBUG_H */

