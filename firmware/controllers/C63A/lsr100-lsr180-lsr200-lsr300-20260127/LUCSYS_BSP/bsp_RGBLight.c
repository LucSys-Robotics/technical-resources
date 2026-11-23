#include "bsp_RGBLight.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"

#include <math.h>
#include <stdio.h>

#include "tim.h"

// Definition of light strip effect modes
enum{
	RGB_MODE_STATIC = 0,
	RGB_MODE_FADE,
	RGB_MODE_RAINBOW,
	RGB_MODE_BREATH,
	RGB_MODE_BLINK,
	RGB_MODE_ERRORTIPS
};

// Configure target parameters for the strip (passed to the task)
typedef struct {
    uint8_t r;          // Target red value (for gradients)
    uint8_t g;          // Target green value
    uint8_t b;          // Target blue value
    uint32_t steps;     // Gradient step count
    uint32_t delay_ms;  // Delay per step
} RGB_EffectParams_t;

// Real-time status of the light strip
typedef struct {
    uint8_t r;          // Current red value
    uint8_t g;          // Current green value
    uint8_t b;          // Current blue value
    uint8_t is_on;      // Light strip switch status (0: off, 1: on)
    uint8_t mode;       // Operating mode
	RGB_EffectParams_t setTarget;//Set the target value for the light strip
	TaskHandle_t TaskHandle;//Lightstrip task handle
} RGB_State_t;


// Light strip status management
static RGB_State_t rgb_state = {0, 0, 0, 0, RGB_MODE_STATIC,{0},NULL};
static SemaphoreHandle_t rgb_mutex = NULL;

static void RGBLight_Init(void)
{
	rgb_mutex = xSemaphoreCreateMutex();
    if (rgb_mutex == NULL)
	{
        while (1); // Error handling
    }
    rgb_state.is_on = 0;
	
	//Timer configuration
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0);
}

//Light strip color setting function
static void RGB_Set(uint8_t r_value,uint8_t g_value,uint8_t b_value)
{
    if (xSemaphoreTake(rgb_mutex, portMAX_DELAY) == pdTRUE)
	{
        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, r_value);
        __HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, g_value);
        __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, b_value);
        rgb_state.r = r_value;
        rgb_state.g = g_value;
        rgb_state.b = b_value;
        xSemaphoreGive(rgb_mutex);
    }
}

// Stop the current effect task
static void RGB_StopEffect(void) 
{
    if ( rgb_state.TaskHandle != NULL)
	{
        vTaskDelete(rgb_state.TaskHandle);
        rgb_state.TaskHandle = NULL;
    }
}

// Linear interpolation
static uint8_t lerp(uint8_t start, uint8_t end, float t) {
    return (uint8_t)(start + (end - start) * t);
}

// Gradient method setup implementation task
static void RGB_FadeTask(void *pvParameters) {
    for (uint32_t i = 0; i <= rgb_state.setTarget.steps; i++) {
        float t = (float)i / rgb_state.setTarget.steps;
        uint8_t r = lerp(rgb_state.r, rgb_state.setTarget.r, t);
        uint8_t g = lerp(rgb_state.g, rgb_state.setTarget.g, t);
        uint8_t b = lerp(rgb_state.b, rgb_state.setTarget.b, t);
        RGB_Set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(rgb_state.setTarget.delay_ms));
    }
    rgb_state.mode = RGB_MODE_STATIC; // Switch to static mode after gradients are complete
	rgb_state.TaskHandle = NULL;
    vTaskDelete(NULL); //Set up and delete the task
}

// HSV to RGB
static void HSVtoRGB(float h, uint8_t *r, uint8_t *g, uint8_t *b) {
    h = fmod(h, 360.0f);
    float s = 1.0f, v = 1.0f;
    int i = (int)(h / 60.0f) % 6;
    float f = (h / 60.0f) - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);
    float r_f, g_f, b_f;
    switch (i) {
        case 0: r_f = v; g_f = t; b_f = p; break;
        case 1: r_f = q; g_f = v; b_f = p; break;
        case 2: r_f = p; g_f = v; b_f = t; break;
        case 3: r_f = p; g_f = q; b_f = v; break;
        case 4: r_f = t; g_f = p; b_f = v; break;
        case 5: r_f = v; g_f = p; b_f = q; break;
        default: r_f = g_f = b_f = 0; break;
    }
    *r = (uint8_t)(r_f * 255);
    *g = (uint8_t)(g_f * 255);
    *b = (uint8_t)(b_f * 255);
}

// Rainbow effect implementation task
static void RGB_RainbowTask(void *pvParameters)
{
    float hue = 0.0f;
	
	//Gradually to the starting position of the rainbow first
    for (uint32_t i = 0; i <= 100; i++) {
        float t = (float)i / 200;
        uint8_t r = lerp(rgb_state.r, 255, t);
        uint8_t g = lerp(rgb_state.g, 0, t);
        uint8_t b = lerp(rgb_state.b, 0, t);
        RGB_Set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
	
    while (1) {
        uint8_t r, g, b;
        HSVtoRGB(hue, &r, &g, &b);
        RGB_Set(r, g, b);
        hue = fmod(hue + 0.5f, 360.0f);//Loop through hues to create rainbow effect
        vTaskDelay(pdMS_TO_TICKS(rgb_state.setTarget.delay_ms));
    }
}

//Error prompt
static void RGB_ErrorTipsTask(void* pvParameters)
{
	uint8_t tips = 0;
	uint8_t r = 0;
	
    for (uint32_t i = 0; i <= 100; i++) {
        float t = (float)i / 200;
        uint8_t r = lerp(rgb_state.r, 0, t);
        uint8_t g = lerp(rgb_state.g, 0, t);
        uint8_t b = lerp(rgb_state.b, 0, t);
        RGB_Set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
	
	while(1)
	{
		for(uint32_t i=0;i<rgb_state.setTarget.steps*2;i++)
		{
			tips=!tips;
			r = (tips == 0)?0:255;
			RGB_Set(r,0,0);
			vTaskDelay(350);
		}
		
		vTaskDelay(2000);
	}
}

// Breathing light effect to complete tasks
static void RGB_BreathTask(void *pvParameters)
{
	//Transition to the light strip to turn off the breathing light
    for (uint32_t i = 0; i <= 100; i++) {
        float t = (float)i / 200;
        uint8_t r = lerp(rgb_state.r, 0, t);
        uint8_t g = lerp(rgb_state.g, 0, t);
        uint8_t b = lerp(rgb_state.b, 0, t);
        RGB_Set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
	
    while (1) 
	{
        // Goes from 0 to 255 and back to 0, simulating breathing effects
        for (uint32_t i = 0; i <= rgb_state.setTarget.steps; i++) {
            float t = (float)i / rgb_state.setTarget.steps;
            uint8_t brightness = (uint8_t)(255 * sin(t * 3.14159f));
            RGB_Set(rgb_state.setTarget.r * brightness / 255, rgb_state.setTarget.g * brightness / 255, rgb_state.setTarget.b * brightness / 255);
            vTaskDelay(pdMS_TO_TICKS(rgb_state.setTarget.delay_ms));
        }
    }
}

//User-specified blinking
static void RGB_BlinkTask(void *pvParameters)
{
	//Transition to Close
    for (uint32_t i = 0; i <= 100; i++) {
        float t = (float)i / 200;
        uint8_t r = lerp(rgb_state.r, 0, t);
        uint8_t g = lerp(rgb_state.g, 0, t);
        uint8_t b = lerp(rgb_state.b, 0, t);
        RGB_Set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
	
	uint8_t blinkflag = 0;
	while(1)
	{
		blinkflag=!blinkflag;
		
		if( blinkflag ) RGB_Set(rgb_state.setTarget.r,rgb_state.setTarget.g,rgb_state.setTarget.b);
		else RGB_Set(0,0,0);
		
		vTaskDelay(rgb_state.setTarget.delay_ms);
	}
}


// Set the color method immediately
static void RGB_SetColorImmediate(uint8_t r, uint8_t g, uint8_t b)
{
    RGB_StopEffect();
    rgb_state.mode = RGB_MODE_STATIC;
    rgb_state.is_on = 1;
	rgb_state.setTarget.r=r;
	rgb_state.setTarget.g=g;
	rgb_state.setTarget.b=b;
    RGB_Set(r, g, b);
}

// Methods for setting gradient colors
static void RGB_SetColorFade(uint8_t r, uint8_t g, uint8_t b)
{
	//Prevent repeated parameter calls in the same mode
	if( (rgb_state.mode==RGB_MODE_FADE || rgb_state.mode==RGB_MODE_STATIC) &&
        (r==rgb_state.setTarget.r && g==rgb_state.setTarget.g && b==rgb_state.setTarget.b)	)
	return;
	
	RGB_StopEffect();
    rgb_state.setTarget.r = r;
    rgb_state.setTarget.g = g;
    rgb_state.setTarget.b = b;
    rgb_state.setTarget.steps = 255;
    rgb_state.setTarget.delay_ms = 35;
    rgb_state.mode = RGB_MODE_FADE;
    rgb_state.is_on = 1;
    xTaskCreate(RGB_FadeTask, "FadeTask", 128*4, NULL, osPriorityBelowNormal6, &rgb_state.TaskHandle);
}

// Turn off the light strip
static void RGB_TurnOff(void)
{
	RGB_SetColorFade(0,0,0);
}

// How to set the rainbow effect
static void RGB_StartRainbow(void)
{
	//Rainbow mode has no parameters to prevent repeated calls
	if( rgb_state.mode==RGB_MODE_RAINBOW ) return;
	
    RGB_StopEffect();
    rgb_state.mode = RGB_MODE_RAINBOW;
    rgb_state.is_on = 1;

    rgb_state.setTarget.delay_ms = 20;
    xTaskCreate(RGB_RainbowTask, "RainbowTask", 128*2, NULL, osPriorityBelowNormal6, &rgb_state.TaskHandle);
}

// How to set breathing light effects
static void RGB_StartBreath(uint8_t r, uint8_t g, uint8_t b)
{
	//Prevent repeated parameter calls in the same mode
	if( rgb_state.mode==RGB_MODE_BREATH && r==rgb_state.setTarget.r && g==rgb_state.setTarget.g && b==rgb_state.setTarget.b ) return;
	
	RGB_StopEffect();
	
	rgb_state.setTarget.r = r;
	rgb_state.setTarget.g = g;
	rgb_state.setTarget.b = b;
	rgb_state.setTarget.steps = 100;
	rgb_state.setTarget.delay_ms = 30;
    rgb_state.mode = RGB_MODE_BREATH;
    rgb_state.is_on = 1;
	
    xTaskCreate(RGB_BreathTask, "BreathTask", 128*2, NULL, osPriorityBelowNormal6, &rgb_state.TaskHandle);
}

static void RGB_ShowError(uint8_t times)
{
	if( rgb_state.mode==RGB_MODE_ERRORTIPS && times==rgb_state.setTarget.steps ) return;
	RGB_StopEffect();

	rgb_state.mode = RGB_MODE_ERRORTIPS;
	rgb_state.setTarget.steps=times;
	xTaskCreate(RGB_ErrorTipsTask, "ErrorTipsTask", 128*2, NULL, osPriorityBelowNormal6, &rgb_state.TaskHandle);
}

//Set the flashing effect
static void RGB_SetBlink(uint8_t r, uint8_t g, uint8_t b,uint32_t blink_times)
{
	//Prevent repeated parameter calls in the same mode
	if( rgb_state.mode==RGB_MODE_BLINK && r==rgb_state.setTarget.r && g==rgb_state.setTarget.g &&
 		b==rgb_state.setTarget.b && blink_times==rgb_state.setTarget.delay_ms ) return;
	
	RGB_StopEffect();
	
	rgb_state.setTarget.r = r;
	rgb_state.setTarget.g = g;
	rgb_state.setTarget.b = b;
	rgb_state.setTarget.steps = 0;
	rgb_state.setTarget.delay_ms = blink_times;
    rgb_state.mode = RGB_MODE_BLINK;
    rgb_state.is_on = 1;
	
    xTaskCreate(RGB_BlinkTask, "BlinkTask", 128*2, NULL, osPriorityBelowNormal6, &rgb_state.TaskHandle);
}

RGBLightInterface_t UserRGBLight = {
	.init = RGBLight_Init,
	.turnoff = RGB_TurnOff,
	.SetColor = RGB_SetColorImmediate,
	.SetColorFade = RGB_SetColorFade,
	.SetBreath = RGB_StartBreath,
	.RainbowMode = RGB_StartRainbow,
	.ShowError = RGB_ShowError,
	.SetBlink = RGB_SetBlink
};
