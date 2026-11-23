#include "bsp_flash.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_flash.h"

//F407VE has 512kb Flash. Save data in the last sector 7. 0x8000000 + (512-128)*1024 = 0x806000
#define FLASH_SAVE_ADDR 0x08060000

/* FLASH sector start address */
#define ADDR_FLASH_SECTOR_0     ((uint32_t )0x08000000)     /* Sector 0 starting address, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t )0x08004000)     /* Sector 1 starting address, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t )0x08008000)     /* Sector 2 starting address, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t )0x0800C000)     /* Sector 3 starting address, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t )0x08010000)     /* Sector 4 starting address, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t )0x08020000)     /* Sector 5 starting address, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t )0x08040000)     /* Sector 6 starting address, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t )0x08060000)     /* Sector 7 starting address, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t )0x08080000)     /* Sector 8 starting address, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t )0x080A0000)     /* Sector 9 starting address, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t )0x080C0000)     /* Sector 10 starting address, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t )0x080E0000)     /* Sector 11 starting address, 128 Kbytes */

/**
Get the Flash sector where a certain address is located
faddr: flash address
0~11, that is, the sector where the addr is located
*/
static uint8_t stmflash_get_flash_sector(uint32_t addr)
{
    if (addr < ADDR_FLASH_SECTOR_1)return 0;
    else if (addr < ADDR_FLASH_SECTOR_2)return 1;
    else if (addr < ADDR_FLASH_SECTOR_3)return 2;
    else if (addr < ADDR_FLASH_SECTOR_4)return 3;
    else if (addr < ADDR_FLASH_SECTOR_5)return 4;
    else if (addr < ADDR_FLASH_SECTOR_6)return 5;
    else if (addr < ADDR_FLASH_SECTOR_7)return 6;
    else if (addr < ADDR_FLASH_SECTOR_8)return 7;
    else if (addr < ADDR_FLASH_SECTOR_9)return 8;
    else if (addr < ADDR_FLASH_SECTOR_10)return 9;
    else if (addr < ADDR_FLASH_SECTOR_11)return 10;

    return 11;
}

static uint32_t read_flash(uint32_t addr)
{
	return  *(volatile uint32_t *)addr;
}

//External interface, save parameters to Flash, occupying space in the 7th sector
int8_t User_Flash_SaveParam(uint32_t* data,uint16_t datalen)
{
	HAL_FLASH_Unlock();//Unlock

    // Erase the target sector
    uint32_t SectorError;
    FLASH_EraseInitTypeDef EraseInitStruct;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//Partial erase
    EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;     //Voltage range: 2.7~3.6V
    EraseInitStruct.Sector = stmflash_get_flash_sector(FLASH_SAVE_ADDR); //Erased sectors
    EraseInitStruct.NbSectors = 1;    //Number of sectors to erase
    HAL_StatusTypeDef eraseStatus = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError); //Start erasing
    if (eraseStatus != HAL_OK) /* Flash error debugging method */
    {
        //printf("Flash erase failed at sector: %lu, error code: %d\r\n", SectorError, HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return -1;
    }
	
	
	uint32_t writeaddr = FLASH_SAVE_ADDR;
	for(uint16_t i=0;i<datalen;i++)
	{
		HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,writeaddr,data[i]);	
        if (status != HAL_OK) /* Flash error debugging method */
        {
            //printf("Flash write failed at address: 0x%08X, error: %d\r\n", writeaddr, HAL_FLASH_GetError());
            HAL_FLASH_Lock();
            return -2;
        }
		writeaddr += 4;
	}
	
	HAL_FLASH_Lock();//Lock
	
	return 0;
}

//External interface, reads parameters to Flash, occupying the 7th sector space
void User_Flash_ReadParam(uint32_t* p,uint16_t datalen)
{
	uint16_t i=0;
	uint32_t addr = FLASH_SAVE_ADDR;
	for(i=0;i<datalen;i++)
	{
		p[i] = read_flash(addr);
		addr += 4;
	}
}


