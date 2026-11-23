#include "adc.h"
#include "motor.h"

//Ackermann steering slide-rail data
AKM_SERVO_ADC Akm_Servo;

//The amount of data collected using DMA
#define akm_servo_bufsize 80 //The total number of ADC data collected per group
static uint16_t adc_OriBuf[akm_servo_bufsize][2]; //Original values (ADC values) of rail position and servo adjustment potentiometer

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
Function: ADC initializes battery voltage detection
Entry parameters: None
Return value: none
**************************************************************************/
static void ADC1_Pin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	ENABLE_BATTERY_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = Battery_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//Analog input
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//No pull-up or down
    GPIO_Init(Battery_PORT, &GPIO_InitStructure);
	
	ENABLE_CarMode_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = CarMode_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//Analog input
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//No pull-up or down
    GPIO_Init(CarMode_PORT, &GPIO_InitStructure);
}


static void ADC2_Pin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ENABLE_SLIDE_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = SLIDE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//Analog input
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//No pull-up or down
    GPIO_Init(SLIDE_PORT, &GPIO_InitStructure);
	
	ENABLE_ServoAdj_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = ServoAdj_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//Analog input
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//No pull-up or down
    GPIO_Init(ServoAdj_PORT, &GPIO_InitStructure);
}

static void ADCx_DMAConfig_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    ENABLE_DMAx_CLOCK;

    DMA_DeInit(DMAx_Streamx);
    while(DMA_GetCmdStatus(DMAx_Streamx)!=DISABLE); //Wait for DMA to be configured
    DMA_InitStructure.DMA_Channel = DMA_ChannelX; //Select DMA channel
    DMA_InitStructure.DMA_PeripheralBaseAddr = DMA_REG_ADDR; //Address of the peripheral
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)adc_OriBuf; //Address of the storage area
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //Peripherals to Storage mode
    DMA_InitStructure.DMA_BufferSize = akm_servo_bufsize*2; //Number of bytes to be transferred
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //Peripheral addresses do not increment automatically
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //Storage address increments
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //Peripheral Transmission Half-Word (16-bit)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //Storage area saves half a word
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //Loop mode
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //High priority

    //////////// No need to configure parameters after disabling FIFO ////////////
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//Memory burst single transfer
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//Peripheral burst single transmission
    //////////// *** ////////////
    DMA_Init(DMAx_Streamx, &DMA_InitStructure);//Initialize DMA

    DMA_Cmd(DMAx_Streamx, ENABLE);  //Enable DMA transfer
}

void ADC1_Init(void)
{
	ADC1_Pin_Init();//Configured with ADC1 pin
	
    ADC_CommonInitTypeDef  ADC_CommonInitStructure;
    ADC_InitTypeDef        ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //Enable ADC1 clock
	
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1 reset
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);  //Reset complete

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                     //Independent mode
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //A 5-hour delay between two sampling stages
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;      //Disable DMA
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                  //Prescaler divide-by-4: ADCCLK=PCLK2/4=84/4=21MHz; ADC clock should not exceed 36MHz
    ADC_CommonInit(&ADC_CommonInitStructure);                                    //Initialize

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                       //12-bit mode
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                                //Non-scan mode
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                          //Turn off continuous conversion
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  //Disable trigger detection; use software to trigger detection
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                       //Align to the right
    ADC_InitStructure.ADC_NbrOfConversion = 1;                                   //1 transformation in a regular sequence, meaning only transform regular sequence 1

    ADC_Init(ADC1, &ADC_InitStructure);//ADC1 initialization
    ADC_Cmd(ADC1, ENABLE);//Turn on the AD converter
}

void ADC2_Init(void)
{
	ADC2_Pin_Init();//ADC2 user pin configuration
	
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); //Enable the ADC2 clock

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	   	 //ADC2 reset
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);	 //Reset complete

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                    //Independent mode
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//A 5-hour delay between two sampling stages
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;     //Disable DMA
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                 //Prescaler divide-by-4: ADCCLK=PCLK2/4=84/4=21MHz; ADC clock should not exceed 36MHz
    ADC_CommonInit(&ADC_CommonInitStructure);                                   //Initialize

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                      //12-bit mode
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                //For multi-channel, enable scan mode
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                          //Continuous conversion
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //Disable trigger detection; use software to trigger detection
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      //Align to the right
    ADC_InitStructure.ADC_NbrOfConversion = 2;                                  //1 transformation in a regular sequence, meaning only transform regular sequence 1
    ADC_Init(ADC2, &ADC_InitStructure);                                         //ADC initialization

    //Configure regular channels and DMA requests
    ADC_RegularChannelConfig(ADC2,SLIDE_Ch,1,ADC_SampleTime_480Cycles);    //First collection of slide track data, stored in Akm_Servo.adc_OriBuf[x][0]
	ADC_RegularChannelConfig(ADC2,ServoAdj_Ch,2,ADC_SampleTime_480Cycles); //Second, collect potentiometer data in the upper right corner, stored in Akm_Servo.adc_OriBuf[x][1]
	
	//After data conversion is completed, a DMA request is automatically generated
    ADC_DMARequestAfterLastTransferCmd(ADC2,ENABLE);
	
	//ADC2 starts
    ADC_Cmd(ADC2, ENABLE);
	
	//DMA configuration
	ADCx_DMAConfig_Init();
	
	//ADC2 DMA handling request enabled
    ADC_DMACmd(ADC2,ENABLE);
	
    //Trigger the conversion, with the DMA handling the data transfer
    ADC_SoftwareStartConv(ADC2);
}

//Return average slide-rail data acquired by DMA
int get_DMA_SlideRes(void)
{
	int tmp = 0;
	for(u8 i=0;i<akm_servo_bufsize;i++)
	{
		tmp += adc_OriBuf[i][0];
	}
	tmp = ( tmp/(float)akm_servo_bufsize ) - 2048;
	return tmp;
}

//Return average top-right potentiometer data
int get_DMA_ServoBias(void)
{
	int tmp = 0;
	for(u8 i=0;i<akm_servo_bufsize;i++)
	{
		tmp += adc_OriBuf[i][1];
	}
	tmp = ( tmp/(float)akm_servo_bufsize ) - 2048;
	return tmp;
}

//Initialize Ackermann servo parameters
void Akm_ServoParam_Init(AKM_SERVO_ADC* p)
{
	p->Max = 2000;
	p->Min = 1000;
	p->Mid = SERVO_INIT;
}

/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
Function: AD sampling
Entry parameters: ADC channels
Return value: AD conversion result
**************************************************************************/
u16 Get_ADC1(volatile u8 ch)
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

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
Function: Collect ADC values multiple times and calculate the average
Entry parameters: ADC channels and number of acquisitions
Return value: AD conversion result
**************************************************************************/
u16 Get_ADC1_Average(u8 chn, u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0; t<times; t++)
    {
        temp_val+=Get_ADC1(chn);
    }
    return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in V
Function: Read battery voltage
Entry parameters: None
Return value: battery voltage, unit: V
**************************************************************************/
float Get_battery_volt(void)
{
    float Volt;
    //The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
    //Resistor voltage division, which can be obtained based on a simple analysis based on the schematic
    Volt=Get_ADC1(Battery_Ch)*3.3*11.0/1.0/4096;
    return Volt;
}


