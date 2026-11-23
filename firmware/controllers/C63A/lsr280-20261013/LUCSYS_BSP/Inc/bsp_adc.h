#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

//The index value stored in the buffer by the ADC
#define userconfigADC_VOL_CHANNEL       0
#define userconfigADC_CARMODE_CHANNEL   1

//External interface
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel);
void ADC_Userconfig_Init(void);


#endif /* __BSP_ADC_H */
