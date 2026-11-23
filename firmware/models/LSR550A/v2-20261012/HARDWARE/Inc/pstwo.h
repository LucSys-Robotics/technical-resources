#ifndef __PSTWO_H
#define __PSTWO_H
#include "sys.h"

/*--------PS2 config--------*/
//PS2 controller pins
#define ENABLE_PS2_DI_PIN_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define ENABLE_PS2_DO_PIN_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define ENABLE_PS2_CS_PIN_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)
#define ENABLE_PS2_CLK_PIN_CLOCK RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE)

#define PS2_DI_PORT  GPIOE
#define PS2_DO_PORT  GPIOE
#define PS2_CS_PORT  GPIOE
#define PS2_CLK_PORT GPIOE

#define PS2_DI_PIN  GPIO_Pin_0
#define PS2_DO_PIN  GPIO_Pin_1
#define PS2_CS_PIN  GPIO_Pin_2
#define PS2_CLK_PIN GPIO_Pin_3

#define DI   PEin(0)     //Input pin

#define DO_H PEout(1)=1   //Command height // Command high
#define DO_L PEout(1)=0   //Command low // Command low

#define CS_H PEout(2)=1  //Cs pull up // CS pull up
#define CS_L PEout(2)=0  //Cs drawdown // CS pulls down

#define CLK_H PEout(3)=1 //Clock lift // Clock lift
#define CLK_L PEout(3)=0 //Clock down // Clock pull down
/*----------------------------------*/

//External interface
void PS2_Init(void);
void PS2_Read(void);
void PS2_Key_Param_Init(void);

#endif





