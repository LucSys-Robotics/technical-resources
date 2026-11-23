#include "adc.h"

float Voltage,Voltage_Count,Voltage_All; //Variables related to battery voltage sampling // Variables related to battery voltage sampling
const float Revise=0.99;

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
Function: ADC initializes battery voltage detection
Entry parameters: None
Return value: none
**************************************************************************/
void  Adc_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//Enable GPIOA clock
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //Enable ADC1 clock


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//Analog input
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//No pull-up or down
		GPIO_Init(GPIOB, &GPIO_InitStructure);//Initialize

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1 reset
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//Reset complete

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//Independent mode
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//A 5-hour delay between two sampling stages
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //Disable DMA
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//Prescaler divide-by-4: ADCCLK=PCLK2/4=84/4=21MHz; ADC clock should not exceed 36MHz
		ADC_CommonInit(&ADC_CommonInitStructure);//Initialize

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12-bit mode
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//Non-scan mode
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//Turn off continuous conversion
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//Disable trigger detection; use software to trigger detection
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//Align to the right
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1 transformation in a regular sequence, meaning only transform regular sequence 1
		ADC_Init(ADC1, &ADC_InitStructure);//ADC initialization
		ADC_Cmd(ADC1, ENABLE);//Turn on the AD converter
}		


void  Adc_POWER_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//Enable GPIOA clock
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); //Enable ADC1 clock


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0 channel 8
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//Analog input
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//No pull-up or down
		GPIO_Init(GPIOB, &GPIO_InitStructure);//Initialize

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	  //ADC2 reset
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);	//Reset complete

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//Independent mode
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//A 5-hour delay between two sampling stages
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //Disable DMA
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//Prescaler divide-by-4: ADCCLK=PCLK2/4=84/4=21MHz; ADC clock should not exceed 36MHz
		ADC_CommonInit(&ADC_CommonInitStructure);//Initialize

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12-bit mode
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//Non-scan mode
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//Turn off continuous conversion
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//Disable trigger detection; use software to trigger detection
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//Align to the right
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1 transformation in a regular sequence, meaning only transform regular sequence 1
		ADC_Init(ADC2, &ADC_InitStructure);//ADC initialization
		ADC_Cmd(ADC2, ENABLE);//Turn on the AD converter
}		
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
Function: AD sampling
Entry parameters: ADC channels
Return value: AD conversion result
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//Set the rule group channel, sequence, and sampling time for the specified ADC
	
	//ADC1, ADC channel, sampling time of 480 cycles
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC1 software transformation startup function	
  //Enable the software conversion startup function for the specified ADC1
	ADC_SoftwareStartConv(ADC1);
	//Wait for the conversion to finish
  //Wait for the conversion to finish
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//Return the latest ADC1 regular-group conversion result
	return ADC_GetConversionValue(ADC1);	
}

u16 Get_Adc2(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//Set the rule group channel, sequence, and sampling time for the specified ADC
	
	//ADC2, ADC channel, sampling time of 480 cycles
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC2 software transformation startup function	
  //Enable the software conversion startup function for the specified ADC1
	ADC_SoftwareStartConv(ADC2);
	//Wait for the conversion to finish
  //Wait for the conversion to finish
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
	//Returns the result of the last ADC2 rule group conversion
	//Return the latest ADC1 regular-group conversion result
	return ADC_GetConversionValue(ADC2);	
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
Function: Collect ADC values multiple times and calculate the average
Entry parameters: ADC channels and number of acquisitions
Return value: AD conversion result
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
Function: Read battery voltage
Entry parameters: None
Return value: battery voltage, unit: mV
**************************************************************************/
float Get_battery_volt(void)   
{  
	float Volt;
	
	//The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
	//Resistor voltage division, which can be obtained based on a simple analysis based on the schematic
	Volt=Get_Adc2(Battery_Ch)*3.3*11.0*Revise/1.0/4096;	
	return Volt;
}




