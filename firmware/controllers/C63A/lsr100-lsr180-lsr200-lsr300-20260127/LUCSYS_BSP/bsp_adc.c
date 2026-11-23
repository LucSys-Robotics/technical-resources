#include "bsp_adc.h"
#include "adc.h"

#define userconfig_ADCDMA_BUF_LEN 80 //Set the number of channels that DMA can transport

static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 };

void ADC_Userconfig_Init(void)
{
	//Start ADC acquisition and set up DMA transmission
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)g_Adc1Buf,sizeof(g_Adc1Buf)/sizeof(g_Adc1Buf[0][0]));
}

//Obtain the ADC value after DMA transfer
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
	uint32_t tmp = 0;
	
	//Index checks prevent array overflow
	if( channel > 1 ) return 0;
	
	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc1Buf[i][channel];
	}
	
	return tmp/userconfig_ADCDMA_BUF_LEN;
}

