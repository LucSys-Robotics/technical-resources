#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdint.h>

//FLASH parameter save and read function
int8_t User_Flash_SaveParam(uint32_t* data,uint16_t datalen);
void User_Flash_ReadParam(uint32_t* p,uint16_t datalen);

#endif /* __BSP_FLASH_H */
