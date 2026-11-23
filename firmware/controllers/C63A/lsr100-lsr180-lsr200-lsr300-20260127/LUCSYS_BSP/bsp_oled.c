/*
 * oled.c
 *
 *  Created on: Jun 11, 2022
 *      Author: lf
 */
#include "bsp_oled.h"
#include "oledfont.h"
#include "gpio.h"

//Oled port macro definition
//Definition of OLED port macros
#define OLED_RST_Clr() HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_RESET);   //RST
#define OLED_RST_Set() HAL_GPIO_WritePin(OLED_RES_GPIO_Port,OLED_RES_Pin,GPIO_PIN_SET);   //RST

#define OLED_RS_Clr() HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_RESET);    //DC
#define OLED_RS_Set() HAL_GPIO_WritePin(OLED_DC_GPIO_Port,OLED_DC_Pin,GPIO_PIN_SET);     //DC

#define OLED_SCLK_Clr()  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_RESET); //SCL
#define OLED_SCLK_Set()  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port,OLED_SCL_Pin,GPIO_PIN_SET);   //SCL

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_RESET);   //SDA
#define OLED_SDIN_Set()  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port,OLED_SDA_Pin,GPIO_PIN_SET);    //SDA
#define OLED_CMD  0	//Command // Write command
#define OLED_DATA 1	//Data // Write data


static void OLED_WR_Byte(uint8_t dat,uint8_t cmd);

uint8_t OLED_GRAM[128][8];
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
Function: Refresh OLED screen
Entry parameters: None
Return value: none
**************************************************************************/
static void OLED_Refresh_Gram(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Set page address (0~7) // Set page address (0~7)
		OLED_WR_Byte (0x00,OLED_CMD);      //Set the display location - column low address // Set the display location - column low address
		OLED_WR_Byte (0x10,OLED_CMD);      //Set the display location - column height address // Set the display location - column height address
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
Function: Write one byte to the OLED
Entry parameters: dat: data/command to write, cmd: data/command flag 0, indicates command; 1, represents data
Return value: none
**************************************************************************/
static void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
	uint8_t i;
	if(cmd)
	{
		OLED_RS_Set();
	}

	else
	{
		OLED_RS_Clr();
	}

	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();
		if(dat&0x80)
		{
			 OLED_SDIN_Set();
		}

		else
		{
			 OLED_SDIN_Clr();
		}

		OLED_SCLK_Set();
		dat<<=1;
	}
	OLED_RS_Set();
}
/**************************************************************************
Function: Turn on the OLED display
Input   : none
Output  : none
Function: Enable OLED display
Entry parameters: None
Return value: none
**************************************************************************/
//static void OLED_Display_On(void)
//{
//	OLED_WR_Byte(0X8D,OLED_CMD); SET DCDC command //SET DCDC command
//	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
//	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
//}
///**************************************************************************
//Function: Turn off the OLED display
//Input   : none
//Output  : none
//Function: Turn off OLED display
//Entry parameters: None
//Return value: none
//**************************************************************************/
//static void OLED_Display_Off(void)
//{
//	OLED_WR_Byte(0X8D,OLED_CMD); SET DCDC command //SET DCDC command
//	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
//	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
//}
/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
Function: Screen clear function. After clearing the screen, the entire screen turns black, as if it is not lit
Entry parameters: None
Return value: none
**************************************************************************/
static void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;
	OLED_Refresh_Gram(); //Update the display // Update the display
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
Function: Draw points
Entry parameters: x, y: starting coordinates; t: 1, fill, 0, clear
Return value: none
**************************************************************************/
static void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//Out of range.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;
}
/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
Function: Displays a character at a specified position, including some characters
Entry parameters: x, y: starting coordinates; len: number of digits; size: font size; mode: 0, highlighted; 1, normal display
Return value: none
**************************************************************************/
static void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' '; //Get the offset value // Get the offset value
    for(t=0;t<size;t++)
    {
		if(size==12)temp=oled_asc2_1206[chr][t];  //Invoke 1206 font //Invoke 1206 font
		else temp=oled_asc2_1608[chr][t];		      //Invoke the 1608 font //Invoke 1608 font
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}
    }
}
/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
Function: Find the function of m to the nth power
Entry parameters: m: base, n: power
Return value: none
**************************************************************************/
static uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}

/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
Function: Displays 2 numbers
Entry parameters: x, y: starting coordinates; len: number digits; size: font size; mode: mode, 0, fill mode, 1, overlay mode; num: value (0~4294967295);
Return value: none
**************************************************************************/
static void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
	}
}


/**************************************************************************
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
Function: Display strings
Entry parameters: x, y: starting coordinates; *p: starting address of the string
Return value: none
**************************************************************************/
static void OLED_ShowString(uint8_t x,uint8_t y,const char *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);
        x+=8;
        p++;
    }
}

//short type display
static void OLED_ShowShortNum(uint8_t x,uint8_t y,short num,uint8_t len,uint8_t size)
{
	if( num < 0 ) num=-num,OLED_ShowString(x,y,"-");
	else if( num>=0 ) OLED_ShowString(x,y,"+");

	OLED_ShowNumber(x+10,y,num,len,size);
}

/**************************************************************************
Function: Initialize the OLED
Input   : none
Output  : none
Function: Initialize OLED
Entry parameters: None
Return value: none
**************************************************************************/
static void OLED_Init(void)
{
	OLED_RST_Clr();
	HAL_Delay(100);
	OLED_RST_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); //Close display // Close display
	OLED_WR_Byte(0xD5,OLED_CMD); //The frequency frequency factor, the frequency of the shock // Set the clock division factor, the oscillation frequency
	OLED_WR_Byte(80,OLED_CMD);   //[3:0], the frequency dividing factor; [7:4], oscillation frequency // [3:0], frequency division factor; [7:4], oscillation frequency
	OLED_WR_Byte(0xA8,OLED_CMD); //Set the number of driver paths // Set the number of driver paths
	OLED_WR_Byte(0X3F,OLED_CMD); //Default 0x3f(1/64) // Default 0X3F(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //Setting display deviation // Set display offset
	OLED_WR_Byte(0X00,OLED_CMD); //Default is 0 // Default is 0

	OLED_WR_Byte(0x40,OLED_CMD); //Sets the number of rows to display starting line [5:0] // Set display start row [5:0], number of rows

	OLED_WR_Byte(0x8D,OLED_CMD); //Charge pump setup // Charge pump setup
	OLED_WR_Byte(0x14,OLED_CMD); //Bit2, on/off //Bit2, on/off
	OLED_WR_Byte(0x20,OLED_CMD); //Set up the memory address mode // Set memory address mode
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00, column address mode; 01, line address mode; 10. Page address mode; The default 10; [1:0], 00, column address mode; 01, row address mode; 10, page address mode; Default 10;
	OLED_WR_Byte(0xA1,OLED_CMD); //Segment redefine setting, bit0:0, 0->0;1, 0->127; //Segment redefine setting, bit0:0, 0->0;1, 0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //Set the COM scan direction; Bit3:0, normal mode; 1, Re-define schema COM[n-1]- >; COM0; N: Number of driving paths // Set COM scan direction; bit3:0, normal mode; 1, redefine mode COM[N-1]->COM0; N: Number of driving paths
	OLED_WR_Byte(0xDA,OLED_CMD); //Set the COM hardware pin configuration // Set the COM hardware pin configuration
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]configuration // [5:4]configuration

	OLED_WR_Byte(0x81,OLED_CMD); //Contrast Settings // Contrast settings
	OLED_WR_Byte(0xEF,OLED_CMD); //1~ 255; Default 0x7f (brightness Settings, the bigger the brighter) //1~255; Default 0X7F (brightness setting, the higher the brighter)
	OLED_WR_Byte(0xD9,OLED_CMD); //Set the pre-charging cycle // Set the pre-charging cycle
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //Setting vcomh voltage multiplier // Set VCOMH voltage multiplier
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //Global display; Bit0:1, open; 0, close; (white screen/black screen) // Global display on; bit0:1, on; 0, off; (white screen/black screen)
	OLED_WR_Byte(0xA6,OLED_CMD); //Settings display mode; Bit0:1, anti-phase display; 0, normal display // Set display mode; bit0:1, inverse display; 0, normal display
	OLED_WR_Byte(0xAF,OLED_CMD); //Open display
	OLED_Clear();
}

/**************************************************************************
Function: Set the coordinates (position) displayed on the screen.
Input   : x, y: starting point coordinates
Output  : none
Function: Set the coordinates (positions) of Chinese characters displayed on the screen
Entry parameters: x, y: starting coordinates
Return value: none
**************************************************************************/
//static void OLED_Set_Pos(unsigned char x, unsigned char y)
//{
//	 OLED_WR_Byte(0xb0+y,OLED_CMD);
//	 OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
//	 OLED_WR_Byte((x&0x0f),OLED_CMD);
//}


//Floating-point factorization
static int* FenJie_float(const float fudian)
{
	static int tmp[3];
	float temp;
	temp = fudian;
	if(temp<0) temp = -temp,tmp[2]=-1;
	else tmp[2]=1;
	
	tmp[0] = (int)temp; 			//Get the integer part
	tmp[1] = (temp - tmp[0])*1000;  //Fractional part
	return tmp;
}

//Function: OLED displays floating-point numbers
//Entry parameters: Number of floating-point values to display, x coordinates, y coordinates, number of integers displayed, number of decimal numbers to display
//Return value: none
static void oled_showfloat(uint8_t show_x,uint8_t show_y,const float needtoshow,uint8_t zs_num,uint8_t xs_num)
{
	static int* p;
	p = FenJie_float(needtoshow);
	//Symbol bit display
	if(p[2]>0) OLED_ShowChar(show_x,show_y,'+',12,1);	 
	else OLED_ShowChar(show_x,show_y,'-',12,1); 		 
	//The integer part
	OLED_ShowNumber(show_x+8,show_y,p[0],zs_num,12); 
	//Decimal point
	OLED_ShowChar(show_x+6+8*zs_num,show_y,'.',12,1);
	//Fractional part
	if(p[1]<100)//Zero padding required
	{	
		if(xs_num==3)
		{
			OLED_ShowNumber(show_x+12+8*zs_num,show_y,0,1,12);	 //Fill in the first zero
			if(p[1]>=10)		
				OLED_ShowNumber(show_x+18+8*zs_num,show_y,p[1],2,12);	 //The last two bits do not require zeros
			else
				OLED_ShowNumber(show_x+18+8*zs_num,show_y,0,1,12), //Supplement the second zero
				OLED_ShowNumber(show_x+24+8*zs_num,show_y,p[1],1,12);
		}
		else
		{
			if(p[1]>=0&&p[1]<100)
				OLED_ShowNumber(show_x+12+8*zs_num,show_y,0,1,12);	 //Fill in the first zero
				OLED_ShowNumber(show_x+18+8*zs_num,show_y,p[1]/10,1,12);
		}
	}
	else
	{
		if(xs_num==3)
		{
			OLED_ShowNumber(show_x+12+8*zs_num,show_y,p[1],3,12);
		}	
		else
			OLED_ShowNumber(show_x+12+8*zs_num,show_y,p[1]/10,2,12);
	}
}


//Attach driver
OLEDInterface_t UserOLED = {
	.init = OLED_Init,
	.ShowChar = OLED_ShowChar,
	.ShowNumber = OLED_ShowNumber,
	.ShowShort = OLED_ShowShortNum,
	.ShowString = OLED_ShowString,
	.ShowFloat = oled_showfloat,
	.RefreshGram = OLED_Refresh_Gram,
	.Clear = OLED_Clear
};

