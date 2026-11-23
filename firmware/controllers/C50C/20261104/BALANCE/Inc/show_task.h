#ifndef __SHOW_TASK_H
#define __SHOW_TASK_H
#include "system.h"

#define SHOW_TASK_PRIO		2
#define SHOW_STK_SIZE 		512  
#define SHOW_TASK_RATE      RATE_10_HZ
extern TaskHandle_t show_TaskHandle;

#define OLED_MAX_PAGE 6 //Number of OLED pages

typedef struct{
	u8 page;       //Current page (page numbers start from 1)
	u8 last_page;  //Previous page
	u8 refrsh;     //Refresh the flag
	u8 MAX_PAGE;   //Maximum OLED pages
}OLED_t;

extern OLED_t oled;

//External interface
void show_task(void *pvParameters);
void OLED_Param_Init(OLED_t* p);

//Internal use function
static float VolMean_Filter(float data);
static void APP_ShowTask(void);
static void OLED_ShowTask(void);

static void oled_akm_show(void);
static void oled_diff_show(void);
static void oled_mec_4wd_omni_show(void);


#endif
