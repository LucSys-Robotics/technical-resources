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

	