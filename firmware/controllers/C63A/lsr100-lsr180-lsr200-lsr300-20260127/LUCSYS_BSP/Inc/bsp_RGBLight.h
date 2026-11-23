#ifndef __BSP_RGBLIGHT_H
#define __BSP_RGBLIGHT_H

#include <stdint.h>

typedef struct {
	void (*init)(void);                                            //Initialize the light strip
	void (*turnoff)(void);                                         //Turn off the light strip
    void (*SetColor)(uint8_t r,uint8_t g,uint8_t b);               //Set the light strip color directly
    void (*SetColorFade)(uint8_t r,uint8_t g,uint8_t b);           //Gradient sets the color of the light strip
	void (*SetBreath)(uint8_t r,uint8_t g,uint8_t b);              //Specify a color to let the light strip breathe
	void (*ShowError)(uint8_t times);                              //Make the light strip flash with an error; the entry parameter is the number of flashes
	void (*RainbowMode)(void);                                     //Rainbow LED strip
	void (*SetBlink)(uint8_t r,uint8_t g,uint8_t b,uint32_t times);//Set the light strip to flash, where times is the flashing delay time, measured in milliseconds
}RGBLightInterface_t,*pRGBLightInterface_t;

extern RGBLightInterface_t UserRGBLight;

#endif

