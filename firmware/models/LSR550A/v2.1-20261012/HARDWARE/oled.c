#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"

static u8 OLED_GRAM[128][8];

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} OLED_Pin_t;

typedef struct
{
    OLED_Pin_t scl;
    OLED_Pin_t sda;
    OLED_Pin_t rst;
    OLED_Pin_t dc;
} OLED_PinMap_t;

static const OLED_PinMap_t oled_pins_old = {
    {GPIOD, GPIO_Pin_7},
    {GPIOD, GPIO_Pin_6},
    {GPIOD, GPIO_Pin_5},
    {GPIOD, GPIO_Pin_4},
};

static const OLED_PinMap_t oled_pins_v1_2 = {
    {GPIOD, GPIO_Pin_7},
    {GPIOD, GPIO_Pin_4},
    {GPIOE, GPIO_Pin_1},
    {GPIOE, GPIO_Pin_0},
};

static const OLED_PinMap_t *oled_pins = &oled_pins_old;

static void OLED_SelectHardware(void)
{
    if(SysVal.HardWare_Ver == V1_2)
        oled_pins = &oled_pins_v1_2;
    else
        oled_pins = &oled_pins_old;
}

static void OLED_PinClockEnable(GPIO_TypeDef *port)
{
    if(port == GPIOA)      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    else if(port == GPIOB) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    else if(port == GPIOC) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    else if(port == GPIOD) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    else if(port == GPIOE) RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}

static void OLED_GPIO_Output_Init(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
}

static void OLED_PinWrite(const OLED_Pin_t *pin, BitAction value)
{
    GPIO_WriteBit(pin->port, pin->pin, value);
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
Function: Write one byte to the OLED
Entry parameters: dat: data/command to write, cmd: data/command flag 0, indicates command; 1, represents data
Return value: none
**************************************************************************/
static void OLED_WR_Byte(u8 dat,u8 cmd)
{
    u8 i;
    if(cmd)
        OLED_RS_Set();
    else
        OLED_RS_Clr();
    for(i=0; i<8; i++)
    {
        OLED_SCLK_Clr();
        if(dat&0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat<<=1;
    }
    OLED_RS_Set();
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
Function: Refresh OLED screen
Entry parameters: None
Return value: none
**************************************************************************/
void OLED_Refresh_Gram(void)
{
    u8 i,n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //Set page address (0~7) // Set page address (0~7)
        OLED_WR_Byte (0x00,OLED_CMD);      //Set the display location - column low address // Set the display location - column low address
        OLED_WR_Byte (0x10,OLED_CMD);      //Set the display location - column height address // Set the display location - column height address
        for(n=0; n<128; n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
    }
}

/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
Function: Screen clear function. After clearing the screen, the entire screen turns black, as if it is not lit
Entry parameters: None
Return value: none
**************************************************************************/
void OLED_Clear(void)
{
    u8 i,n;
    for(i=0; i<8; i++)for(n=0; n<128; n++)OLED_GRAM[n][i]=0X00;
    OLED_Refresh_Gram(); //Update the display // Update the display
}

void OLED_ClearBuf(void)
{
    u8 i,n;
    for(i=0; i<8; i++)for(n=0; n<128; n++)OLED_GRAM[n][i]=0X00;
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
Function: Draw points
Entry parameters: x, y: starting coordinates; t: 1, fill, 0, clear
Return value: none
**************************************************************************/
static void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
    u8 pos,bx,temp=0;
    if(x>127||y>63)return;//Out of range.
    pos=7-y/8;
    bx=y%8;
    temp=1<<(7-bx);
    if(t)OLED_GRAM[x][pos]|=temp;
    else OLED_GRAM[x][pos]&=~temp;
}

/**************************************************************************
Function: Set the coordinates (position) displayed on the screen.
Input   : x, y: starting point coordinates
Output  : none
Function: Set the coordinates (positions) of Chinese characters displayed on the screen
Entry parameters: x, y: starting coordinates
Return value: none
**************************************************************************/
static void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD);
}

/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
Function: Find the function of m to the nth power
Entry parameters: m: base, n: power
Return value: none
**************************************************************************/
static u32 oled_pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}


/**************************************************************************
Function: Initialize the OLED
Input   : none
Output  : none
Function: Initialize OLED
Entry parameters: None
Return value: none
**************************************************************************/
void OLED_Init(void)
{
    OLED_SelectHardware();
    OLED_PinClockEnable(oled_pins->scl.port);
    OLED_PinClockEnable(oled_pins->sda.port);
    OLED_PinClockEnable(oled_pins->rst.port);
    OLED_PinClockEnable(oled_pins->dc.port);

    OLED_GPIO_Output_Init(oled_pins->scl.port, oled_pins->scl.pin);
    OLED_GPIO_Output_Init(oled_pins->sda.port, oled_pins->sda.pin);
    OLED_GPIO_Output_Init(oled_pins->rst.port, oled_pins->rst.pin);
    OLED_GPIO_Output_Init(oled_pins->dc.port, oled_pins->dc.pin);
	
//	//Special IO: handling PC13, 14, and 15
//	//Turn off the external low speed external clock signal function, PC13, PC14, PC15 can be used as normal IO
//    //After turning off the external low-speed external clock signal function, PC13, PC14, and PC15 can be used as regular IO
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
//    PWR_BackupAccessCmd(ENABLE);
//    RCC_LSEConfig(RCC_LSE_OFF);
//    PWR_BackupAccessCmd(DISABLE);

    OLED_RST_Clr();
    delay_ms(100);
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
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
Function: Displays a character at a specified position, including some characters
Entry parameters: x, y: starting coordinates; len: number of digits; size: font size; mode: 0, highlighted; 1, normal display
Return value: none
**************************************************************************/
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{
    u8 temp,t,t1;
    u8 y0=y;
    chr=chr-' '; //Get the offset value // Get the offset value
    for(t=0; t<size; t++)
    {
        if(size==12)temp=oled_asc2_1206[chr][t];  //Invoke 1206 font //Invoke 1206 font
        else temp=oled_asc2_1608[chr][t];		      //Invoke the 1608 font //Invoke 1608 font
        for(t1=0; t1<8; t1++)
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
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
Function: Displays 2 numbers
Entry parameters: x, y: starting coordinates; len: number digits; size: font size; mode: mode, 0, fill mode, 1, overlay mode; num: value (0~4294967295);
Return value: none
**************************************************************************/
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{
    u8 t,temp;
    u8 enshow=0;
    for(t=0; t<len; t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
                continue;
            } else enshow=1;

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
void OLED_ShowString(u8 x,u8 y,const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX) {
            x=0;
            y+=16;
        }
        if(y>MAX_CHAR_POSY) {
            y=x=0;
            OLED_Clear();
        }
        OLED_ShowChar(x,y,*p,12,1);
        x+=8;
        p++;
    }
}

/**************************************************************************
Function: Display character
Input   : x: indicates the horizontal coordinates displayed; Y: the vertical coordinates that show the display;
          no: the line number in the array of the Chinese character (module) in the hzk-and "array", which is determined by the line number to determine the characters shown in the array,
          The value of the width of the font here must be consistent with the size of the dot matrix value of the use of the word mold.
          font_height: the font is high for the use of the word mold, because my screen pixels are 32hours, 128----0~ 7, and four bits per page
Output  : none
Note: this method is used to show that the Chinese character must satisfy the size of the word that the word model generates the software to generate the same size as the dot matrix
Function: Displays Chinese characters
Entry parameters: x: represents the displayed horizontal coordinates; y: represents the displayed vertical coordinates;
          no: Indicates the line number of the Chinese character (module) to be displayed in the hzk[][] array. The line number determines which character to display in the array.
              The font_width value must match the dot-matrix size used when generating the font bitmap;
          font_height: When generating font templates with font creation software, the font height is high. Since my screen resolution is 32*128-----0~7, totaling 8 pages, each page has 4 digits
Return value: none
Note: To display Chinese characters using this method, the font width generated by the font generation software must match the bitmap size; otherwise, garbled text is likely to occur
**************************************************************************/
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 font_width,u8 font_height)
{
    u8 t, i;
    for(i=0; i<(font_height/8); i++)	//The maximum height of font_height is 32. this screen is only 8 pages (line), four digits per page
        //font_height maximum value is 32, and this screen has only 8 pages (rows), with 4 bits per page
    {
        OLED_Set_Pos(x,y+i);
        for(t=0; t<font_width; t++)		//The maximum value of font_width is 128. the screen is only that large
            //font_width maximum is 128, and the screen is only that large
        {
            OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
        }
    }
}

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
void oled_showfloat(const float needtoshow,u8 show_x,u8 show_y,u8 zs_num,u8 xs_num)
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

//Paint function
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,const unsigned char BMP[])
{ 	
 	unsigned int j=0; //Define variables
 	unsigned char x,y; //Define variables
  
 	if(y1%8==0) y=y1/8;   //Determine whether the terminating page is an integer multiple of 8
 	 else y=y1/8+1;

	for(y=y0;y<y1;y++) //Draw from the start page to the end page
	{
		OLED_Set_Pos(x0,y); //Start drawing in the first column of the page
		for(x=x0;x<x1;x++) //Draw columns x1 - x0
			{
				OLED_WR_Byte(BMP[j++],OLED_DATA);	//Draw the dots of the picture
			}
	}
}

void OLED_Refresh_Line(void)
{
	u8 i,n;		    
	for(i=0;i<2;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Set page address (0~7) // Set page address (0~7)
		OLED_WR_Byte (0x00,OLED_CMD);      //Set the display location - column low address // Set the display location - column low address
		OLED_WR_Byte (0x10,OLED_CMD);      //Set the display location - column height address // Set the display location - column height address
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//usb bmp
const unsigned char gImage_usb_bmp[] = { /* 0X72,0X01,0X00,0X40,0X00,0X3F, */
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0XC0,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,
0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0XE0,0X80,
0X80,0X80,0X80,0X80,0X80,0X80,0X80,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XE0,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0X60,0XF8,0X08,
0X0C,0X0C,0X0C,0XFF,0X00,0X00,0XC0,0XC0,0X80,0X80,0X80,0X80,0XC0,0XE0,0XB0,0X98,
0X88,0X88,0X88,0X9C,0X9C,0X9C,0X80,0X80,0XC0,0XC0,0X80,0X80,0X00,0X00,0XFF,0X01,
0X01,0X3D,0X25,0X25,0X3D,0X01,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X07,0X08,
0X08,0X08,0X08,0XFF,0X00,0X00,0X01,0X01,0X00,0X00,0X00,0X01,0X03,0X06,0X0C,0X0C,
0X0C,0X0C,0X0C,0X0C,0X00,0X00,0X00,0X02,0X01,0X01,0X00,0X00,0X00,0X00,0XFF,0X40,
0X40,0X5E,0X52,0X52,0X5E,0X40,0X7F,0X7F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X01,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,
0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};
