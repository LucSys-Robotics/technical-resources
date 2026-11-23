#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h" 
#include "system.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////
//User sets according to their needs
#define STM32_FLASH_SIZE 	64 	 			//Selected STM32 FLASH capacity size (in K)
#define STM32_FLASH_WREN 	1              		//Enable FLASH writing (0, not enable; 1, enable)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH start address
#define STM32_FLASH_BASE 0x08000000 			//Start address of STM32 FLASH
//FLASH unlock key value
#define FLASH_KEY1               0X45670123
#define FLASH_KEY2               0XCDEF89AB
void STMFLASH_Unlock(void);					  	//FLASH unlock
void STMFLASH_Lock(void);					  	//FLASH lock
u8 STMFLASH_GetStatus(void);				  	//Gain status
u8 STMFLASH_WaitDone(u16 time);				  	//Wait for the operation to finish
u8 STMFLASH_ErasePage(u32 paddr);			  	//Erase page
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);	//Write half-word
u16 STMFLASH_ReadHalfWord(u32 faddr);		  	//Read half a character
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//Specify the address to start writing data of the specified length
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//Specify the address to start reading data of the specified length
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//Write data of the specified length starting from the specified address
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//Read data of the specified length starting from the specified address

//Test write
void Test_Write(u32 WriteAddr,u16 WriteData);			

void Flash_Read(void);
void Flash_Write(void);
#endif

















