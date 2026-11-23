#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "sys.h"

//#define use_usart_debug 

#ifdef use_usart_debug

/*
__FILE__ // Current program source file, type char*
__FUNCCTIONN__ // Currently running function, type char*
__LINE__ // Current function line, type int
__VA_ARGS__ // Represents the variable parameters in the macro, used together with... Used together

#  : Convert the content following # into a string
## : Concatenation operator; test##i joins test with the supplied i
      In variadic macros, ## also removes the preceding comma
args... : represents a list of variable parameters

*/

#define DEBUG_INFO(fmt,...) \
	any_printf(USART1,"[File:%s][Function:%s][Line:%d]" fmt,__FILE__,__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define DEBUG_INFO(fmt,...) __nop()
#endif


#define OTP_ADDR_TEST 0x1FFF7800 //Starting address of OTP area block 0
#define OTP_LOCK_ADDR 0x1FFF7A00 //Starting address of OTP area lock block configuration

//Flash programming error enumeration type
enum{
	FLASH_EOP    = ( 1 << 0 ) , //Flash programming/erase operation completed (this bit only takes effect when the configuration operation ends and the enable is interrupted (EOPIE=1))
	FLASH_OPERR  = ( 1 << 1 ) , //Flash operation error: There is a Flash operation request, but there is a parallel bit error, alignment error, or write-saving error (this bit only takes effect if the enable operation is interrupted (ERRIE=1))
	FLASH_WRPERR = ( 1 << 4 ) , //The address to be erased/programmed belongs to a write-protected region in Flash
	FLASH_PGAERR = ( 1 << 5 ) , //Programming alignment error: Flash is 128-bit wide, and the data written spans beyond this width
	FLASH_PGPERR = ( 1 << 6 ) , //Programming parallel bit error, PSIZE setting does not match the width of data written to Flash
	FLASH_PGSERR = ( 1 << 7 ) , //Programming order error: the sequence of steps when controlling registers is incorrect
	FLASH_BSY    = ( 1 << 16) , //Flash operation is currently in progress
};
#define Get_FlashState(res,mask) ( res & mask) //Get the Flash enumeration error type

//Flash error enumeration
typedef enum
{
	flash_no_error = 0 , //Flash has no errors
	flash_busy ,     //Flash is busy with operations
	flash_more_error,//Multiple errors exist during the Flash operation
	flash_eop,       //Flash operation completed
	flash_operr,     //Flash operation error
	flash_wrperr,    //Flash write protection error
	flash_pgperr,    //Flash parallel error
	flash_pgserr,    //Flash alignment error
	flash_overtime,   //Flash waits for timeout
	flash_AddrErr,   //Invalid Flash address for the requested operation
	flash_AddrWillout,//Flash operations will exceed capacity limits by the end
	flash_sector_out, //The sector to be set exceeds the sector already present on the chip
}FlashState;


//Flash base address
#define STM32_FLASH_BASE    0x08000000

//Flash capacity of the chip used, unit kB
#define STM32_FLASH_SIZE 512

//Flash end address
#define FLASH_END_ADDR  STM32_FLASH_BASE + STM32_FLASH_SIZE*1024 - 1

//RAM base address
#define STM32_RAM_BASE 0x20000000
#define STM32_RAM_SIZE 192
#define RAM_END_ADDR  STM32_RAM_BASE + STM32_RAM_SIZE*1024 - 1

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


//External interface
uint8_t Write_Flash(uint32_t* data,uint16_t datalen);
int Read_Flash(uint16_t index);

#endif

