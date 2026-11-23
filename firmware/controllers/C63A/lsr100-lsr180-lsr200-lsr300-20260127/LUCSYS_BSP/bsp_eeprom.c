/**
 * @file    bsp_eeprom.c
 * @brief BL24C02F-RRRC device driver (256 bytes EEPROM space)
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-08-05
 * @version 1.0.0
 *
 * @details
 * - C63X - This device is only available in version V2.0; version V1.0 does not. The software distinguishes the current hardware runtime environment by this feature
 * BL24C02F has a total of 256 bytes of storage space, with addresses 0-255, supporting single-byte and multi-byte writes (up to one page written at once).
 * One page is 16 bytes.
 * @note
 * 
 * 
 */
 
#include "bsp_eeprom.h"
#include "bsp_siic.h"

//IIC Driver
static pIICInterface_t eeprom = &UserVer_sIICDev;
	
//EEPROM write data
//Write up to 16 bytes at a time
uint8_t eeprom_writebytes(uint8_t addr,uint8_t *data,uint8_t bufferLen)
{
	if( bufferLen > 16 ) return 1;
	
	return eeprom->write_reg(&eeprom->iic_io,0x50<<1,addr,data,bufferLen,0xFFFF);
}
	
//EEPROM read data
uint8_t eeprom_readbytes(uint8_t addr,uint8_t* data,uint8_t readLen)
{
	uint8_t state = 0;
	for(uint8_t r=addr;r<addr+readLen;r++)
	{
		state+=eeprom->read_reg(&eeprom->iic_io,0x50<<1,r,data++,1,0xFFFF);
		eeprom->delay_ms(5);
	}
	return state;
}

//Example of version confirmation
//C63X hardware version determined (hardware version confirmed via EEPROM data operations)
// HAL_Delay(500);
// uint8_t write_ver = 1;
// uint8_t iic_state = 0;
// iic_state += eeprom_writebytes(0,&write_ver,1);

// HAL_Delay(5);
// uint8_t read_ver = 0xff;
// iic_state += eeprom_readbytes(0,&read_ver,1);

// //IIC communication is normal, the version number can be correctly written and read, then it's hardware version V2.0
// if( iic_state==0 && read_ver == 1 )
// {
// 	HardWareVersion = HW_1_1;
// }
// else
// {
// 	HardWareVersion = HW_1_0;
	
// 	//Deinitialize V2.0 version CAN pins
// 	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
	
// 	//Initialize the V1.0 version CAN pin
// 	GPIO_InitTypeDef GPIO_InitStruct = {0};
// 	__HAL_RCC_GPIOA_CLK_ENABLE();
// 	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
// 	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
// 	GPIO_InitStruct.Pull = GPIO_NOPULL;
// 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
// 	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
// 	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
// 	//V1.0 version emergency stop switch
// 	HAL_GPIO_DeInit(ENKey_GPIO_Port,ENKey_Pin);
// 	__HAL_RCC_GPIOD_CLK_ENABLE();
// 	GPIO_InitStruct.Pin = ENKey_V1_0_Pin;
// 	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
// 	GPIO_InitStruct.Pull = GPIO_PULLUP;
// 	HAL_GPIO_Init(ENKey_V1_0_GPIO_Port, &GPIO_InitStruct);
// }
	