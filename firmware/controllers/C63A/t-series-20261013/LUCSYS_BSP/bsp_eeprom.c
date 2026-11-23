/**
 * @file    bsp_eeprom.c
 * @brief   BL24C02F-RRRC器件驱动(256bytes空间的EEPROM)
 * @note LucSys Robotics distribution maintenance; original authorship not claimed
 * @date    2025-08-05
 * @version 1.0.0
 *
 * @details
 * - C63X - V2.0版本才有此器件,V1.0版本没有此器件.软件通过此特征来区分当前的硬件运行环境
 * BL24C02F共有256bytes存储空间,分别是地址 0-255,支持单字节写入、多字节写入(最多一次性写入1页)
 * 一页为16bytes.
 * @note
 * 
 * 
 */
 
#include "bsp_eeprom.h"
#include "bsp_siic.h"

//IIC驱动
static pIICInterface_t eeprom = &UserVer_sIICDev;
	
//EEPROM写入数据
//1次最多写入16字节
uint8_t eeprom_writebytes(uint8_t addr,uint8_t *data,uint8_t bufferLen)
{
	if( bufferLen > 16 ) return 1;
	
	return eeprom->write_reg(&eeprom->iic_io,0x50<<1,addr,data,bufferLen,0xFFFF);
}
	
//EEPROM读数据
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

	