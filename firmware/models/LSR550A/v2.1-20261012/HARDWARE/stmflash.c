#include "stmflash.h"
#include "delay.h"

//F407VE has 512kb Flash. Save data in the last sector 7. 0x8000000 + (512-128)*1024 = 0x806000
#define FLASH_SAVE_ADDR 0x08060000

//Unlock key for main memory
#define STM32_FLASH_KEY1    ((uint32_t)0x45670123)
#define STM32_FLASH_KEY2    ((uint32_t)0xCDEF89AB)

//Unlock key value for OPT user option bytes
#define STM32_FLASH_OPTKEY1 ((uint32_t)0x08192A3B)
#define STM32_FLASH_OPTKEY2 ((uint32_t)0x4C5D6E7F)

//Unlock the primary storage Flash
void stmflash_unlock(void)
{
	//Note: If you write the key again while the LOCK bit is 0, it will trigger a hardware error and cause the microcontroller to freeze
	if( (FLASH->CR & (uint32_t)1<<31) != 0)
	{
		FLASH->KEYR = STM32_FLASH_KEY1;
		FLASH->KEYR = STM32_FLASH_KEY2;
	}
	else
	{
		DEBUG_INFO("LOCK位当前是0，无需解锁\r\n");
	}
}

//Unlock OPT user option byte area Flash
static void stmflash_opt_unlock(void)
{
	if( (FLASH->OPTCR & (uint32_t)1<<0)!=0 )
	{
		/* FLASH write unlock sequence */
		FLASH->OPTKEYR = STM32_FLASH_OPTKEY1;     
		FLASH->OPTKEYR = STM32_FLASH_OPTKEY2;
	}
	else
	{
		DEBUG_INFO("OPT LOCK位当前是0，无需解锁");
	}
}

//Main storage area Flash locked
static void stmflash_lock(void)
{
	/* FLASH lock */
    FLASH->CR |= (uint32_t)1 << 31;     
}

//Lock Flash option-byte OPT region
static void stmflash_opt_lock(void)
{
	/* FLASH lock */
    FLASH->OPTCR |= (uint32_t)1 << 0;     
}

//Basic Flash settings, which should be adjusted according to your clock frequency and hardware conditions
void stmflash_init(void)
{	
	DEBUG_INFO("1.写入key解锁\r\n");
	stmflash_unlock();
	
	delay_us(100);
	
	//Set Flash's LATENCY
	//Closely related to chip supply voltage and CPU clock frequency configuration
	//The chip supply voltage is 3.3V, CPU frequency is 168M, and it needs to be set to 5 WS
	DEBUG_INFO("2.ACR设置，将LATENCY设置为5ws\r\n");
	FLASH->ACR &= ~(7 << 0); //Clear the original configuration
	FLASH->ACR |= 5 << 0;    //Set LATENCY to 5 ws
	
	delay_us(100);
	//Set the Flash operating unit (PSIZE).
	//Related to the chip supply voltage; when the voltage is 2.7~3.6V, the number of parallel bits should be set to 32
	
	DEBUG_INFO("3.CR设置，设置PSIZE宽度为32位\r\n");
	FLASH->CR &= ~(3 << 8); //Clear the original configuration
	FLASH->CR |= 2<<8;      //Set the PSIZE width to 32 bits
	
	DEBUG_INFO("4.初始化完毕，上锁\r\n");
	stmflash_lock();
	
	/*
	 * Access the ACR register default value: F4 ART accelerator, (1) prefetch enabled,
	 * (2) Instruction cache enable, (3) Data cache enabled by default
	*/
}

//Read Flash data at a specified address
uint32_t stmflash_read_word(uint32_t faddr)
{
	return  *(volatile uint32_t *)faddr;
}

//Specify the address and length to read Flash data
void read_flash(uint32_t addr,uint32_t* p,u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
		p[i] = stmflash_read_word(addr);
		addr += 4;
	}
}

//Retrieve the read/write status of Flash
static FlashState stmflash_get_state(void)
{
    uint32_t res = 0;
	uint8_t error_count = 0;//Error count
	
	//Flash status, first mark as no errors
	FlashState state = flash_no_error ;

	res = FLASH->SR; //Read the SR register to retrieve status
	
	//Retrieve and tag the status of Flash
	if(Get_FlashState(res,FLASH_WRPERR)) 
	{
		error_count++,state = flash_wrperr;
		DEBUG_INFO("要写入的地址处于写保护状态\r\n");
	}
	if(Get_FlashState(res,FLASH_PGPERR)) 
	{
		error_count++,state = flash_pgperr;
		DEBUG_INFO("发生Flash编程并行位错误\r\n");
	}
	if(Get_FlashState(res,FLASH_PGSERR)) 
	{
		error_count++,state = flash_pgserr;
		DEBUG_INFO("发生Flash编程顺序错误\r\n");
	}
	if(Get_FlashState(res,FLASH_BSY)   )
	{
		error_count++,state = flash_busy;
		DEBUG_INFO("Flash总线等待中...\r\n");
	}
	
	/* EOP and OPERR only work when interrupts are enabled, not used by default here */
	//if(Get_FlashState(res,FLASH_EOP)   ) error_count++,state = flash_eop;
	//if(Get_FlashState(res,FLASH_OPERR) ) error_count++,state = flash_operr;
	//tag
	if(error_count>1)
	{
		state  = flash_more_error;
		DEBUG_INFO("Flash总线上存在多个错误...\r\n");
	}
    
    return state;
}

//Wait for Flash to complete the operation
static FlashState stmflash_wait_done(uint32_t time)
{
    FlashState state;
    do
    {
        state = stmflash_get_state();//Get Flash status

        if (state != flash_busy)
        {
            break;      /* Not busy; no need to wait, return immediately */
        }
        
        time--;
    } while (time);

    if (time == 0) 
	{
		DEBUG_INFO("Flash总线等待超时\r\n");
		state = flash_overtime;   /* Timeout */
	}
	
	//Return status
    return state;
}

//Erase sector operation
//Only the primary storage area needs to be erased; the OTP area cannot be erased, and the option byte area will be erased automatically
FlashState stmflash_erase_sector(uint8_t saddr)
{
    FlashState res;

    res = stmflash_wait_done(0XFFFFFFFF);   /* Wait for the last operation to finish */
	
    if (res == flash_no_error)
    {
		stmflash_unlock();
        FLASH->CR &= ~(0X1F << 3);           /* Clear the original sector settings */
        FLASH->CR |= saddr << 3;             /* Set the sectors to erase */
		
		FLASH->CR &= ~(7 << 0);              /* Clear the original configuration of selecting programming, sector erase, and slice erase */
        FLASH->CR |= 1 << 1;                 /* Configure sector erase */
		
        FLASH->CR |= 1 << 16;                /* Start erasing */
        res = stmflash_wait_done(0XFFFFFFFF);/* Wait for the operation to finish */
		stmflash_lock();
		//Note: The STRT bit will automatically clear 0 (bit 16), and the SER bit will not clear (bit 1), so just clear the previous configuration before operating the SER bit
    }
    return res;
}


//Write a word at the specified location in FLASH
//Entry parameter: address, the data to be written
//The address must be a multiple of 4, because PSIZE must be configured based on voltage. The hardware voltage is 3.3V, so PSIZE is configured to 32 bits. \
  Flash是128位宽的，地址不是4的倍数时，将会发生编程对齐错误
static FlashState stmflash_write_word(uint32_t faddr, uint32_t data)
{
    FlashState res;

    res = stmflash_wait_done(0XFFFFF);

    if (res == flash_no_error)
    {
		FLASH->CR &= ~(7 << 0);             /* Clear the original configuration of selecting programming, sector erase, and slice erase */
        FLASH->CR |= 1 << 0;                /* Programming */
        *(volatile uint32_t *)faddr = data; /* Write data */
        res = stmflash_wait_done(0XFFFFF);  /* Wait for the operation to complete, one-word programming */
		//Note: Programming is unrelated to the STRT bit; this bit is used for erasing
    }

    return res;
}


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


//Specify the location and write 32-bit data of the specified length
//Entry parameters: address, data to write, total amount of data to write
//Allowed Write Areas: Main Memory Area, OTP Area (One-Time Programming Area)
FlashState stmflash_write(uint32_t waddr, uint32_t *pbuf, uint32_t length)
{

    FlashState status = flash_no_error;
    uint32_t endaddr = 0;
	
	/* Invalid address:
	 * (1) Address greater than the chip's maximum Flash capacity
	 * (2) Less than the first address of the storage area
	 * (3) The address is not a multiple of 4
	 * (4) Address checks are strict and exclude OTP addresses; to write OTP, the address-check code can be commented out
	*/
    if (waddr < STM32_FLASH_BASE ||  waddr > FLASH_END_ADDR ||  waddr % 4 ) 
    {
		status = flash_AddrErr;//Invalid address
		DEBUG_INFO("写入的地址非法\r\n");
        return status;
    }
	
    endaddr = waddr + length*4;
	if(endaddr>FLASH_END_ADDR) //Writing content will exceed the total memory capacity and is invalid
	{
		DEBUG_INFO("写入的内容未来会超出flash总容量\r\n");
		status = flash_AddrWillout;
		return status;
	}
    
    
	stmflash_unlock();       //Flash unlock
    FLASH->ACR &= ~(1 << 9); //Disable instruction caching
	FLASH->ACR &= ~(1 << 10);//Disable data caching
	FLASH->ACR |= 1 << 11;   //Reset (clear) instruction cache
	FLASH->ACR |= 1 << 12;   //Reset (clear) the data cache
	__disable_irq();         //Close all interrupts to ensure Flash writes are correct
	
	//Wait for the last operation to complete
	status = stmflash_wait_done(0xffff);
	
	//Flash status shows no errors, data writing begins
	if(status == flash_no_error)
	{
		DEBUG_INFO("开始从 0x%x 处写入数据\r\n",waddr);
		while(waddr < endaddr)
		{
			status = stmflash_write_word(waddr , *pbuf);
			if(status!=flash_no_error) break;
			
			waddr += 4;
			pbuf ++ ;
		}	
		
	}

	FLASH->ACR |= 1<<9;    //Enable instruction caching
	FLASH->ACR |= 1<<10;   //Enable data caching
	FLASH->ACR &= ~(1<<11);//Clear the instruction cache reset flag
	FLASH->ACR &= ~(1<<12);//Clear the data cache reset flag
	stmflash_lock();       //Flash locked
	__enable_irq();        //Flash writes complete and resumes interrupt
	
	return status;
}


//Set/Disable write protection for the specified sector
//Entry parameters: sector, set, or cancel (1: set 0: cancel)
FlashState set_sector_WriteProtect(uint8_t sector,uint8_t enable)
{
	FlashState state = flash_no_error;
	uint8_t set_sector;
	
	
	if(sector > stmflash_get_flash_sector(FLASH_END_ADDR))
	{
		state = flash_sector_out;
		return state;
	}
	state = stmflash_wait_done(0xFFFF);//Wait for the last operation to finish
	
	//There are no issues on the Flash bus
	if(state==flash_no_error)
	{
		stmflash_opt_unlock(); //Unlock option byte register protection
		
		if(sector<=11) //F40x series chips have a maximum of 11 sectors
		{
			set_sector = sector + 16 ; //16-bit register sets the offset of position
			
			//Set write protection
			if(enable) FLASH->OPTCR &= ~(1<<set_sector);
			
			//Disable write protection
			else FLASH->OPTCR |= 1<<set_sector;
				
		}
		else // F42x, F43x, sectors can go up to 23
		{
			set_sector = sector-12 + 16 ; //16-bit register sets the offset of position; -12 is the offset relative to the previous 11 sectors
			
			//Set write protection
			if(enable) FLASH->OPTCR1 &= ~(1<<set_sector);
			
			//Disable write protection
			else FLASH->OPTCR1 |= 1<<set_sector;
		}
		
		//Start writing option bytes
		FLASH->OPTCR |= 1<<1;
		
		//Wait for the operation to finish
		state = stmflash_wait_done(0xFFFF);
		
		//Clear the write option byte flag
		//Note: This step is actually not necessary. Regardless of whether the Flash operation succeeds, this bit will automatically be set to 0
		//FLASH->OPTCR &= ~(1<<1);
		
		//Lock option byte areas
		stmflash_opt_lock();
	}
	
	return state;
	
}


u8 get_sector_WriteProtect(u8 sector)
{
	
	u16 temp;
	
	if(sector<=11)
	{
		temp = stmflash_read_word(0x1fffc008);
		
	}
	else
	{
		temp = stmflash_read_word(0x1ffec008);
		
		sector = sector-12; //Subtract the deviation
	}
	
	return !(temp>>sector&0x01);

}

//External functions to write data to Flash.
//Entry parameters: 32-bit data address, length of data to be written.
uint8_t Write_Flash(uint32_t* data,uint16_t datalen)
{
	FlashState res;
	
	static u8 init=0;
	if(init==0) 
	{
		init =1;
		stmflash_init();//Initialize Flash
	}
	
	//Data must be erased before writing
	res = stmflash_erase_sector( stmflash_get_flash_sector(FLASH_SAVE_ADDR) );//Erase the FLash segment to be written
	
	if( res==flash_no_error )
	{
		res = stmflash_write(FLASH_SAVE_ADDR,data,datalen);//Write the value
	}

	return res;
}

//External functions to read Flash content.
//Entry parameter: Index number of the read data
int Read_Flash(uint16_t index)
{
	int tmp;
	tmp = stmflash_read_word( FLASH_SAVE_ADDR+(4*index) );
	return tmp;
}

