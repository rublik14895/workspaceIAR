///////// This code was written for the STM32F429-Discovery board
#include "stm32f4xx_hal.h"
#include "stm32f429xx.h"
#include "BSP/stm32f429i_discovery_lcd.h"
#include "stdio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_adc.h"

#define y0 150
#define h  75

static void SystemClock_Config(void);
static void onLCD();
static void initADC();
static int millis = 0;
static inline int sgn(int x);

int v;
int last_v = 0;
int diff;
int last_diff;
int t;
int last_t = 0;

void TIM1_UP_TIM10_IRQHandler(){
  LL_TIM_ClearFlag_UPDATE(TIM1);
  ++millis;
  last_diff = diff;
  diff = v - last_v;
  if (!(millis % 240))
     BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_DrawPixel(millis % 240, y0 - (int)((float)v / 1024 * h), LCD_COLOR_BLACK);
  LL_ADC_REG_StartConversionSWStart(ADC1);
}

void ADC_IRQHandler(){
  LL_ADC_ClearFlag_EOCS(ADC1);
  last_v = v;
  v=LL_ADC_REG_ReadConversionData10(ADC1);
}

int main()
{
 //HAL Library initialization
 HAL_Init();
 //Appropriate clock configuration
 SystemClock_Config();
 onLCD();
 initADC();

 while(1)
 {
   char str[32];
   sprintf(str, "time: %d", millis / 1000);
   BSP_LCD_DisplayStringAtLine(2, str);
   //HAL_Delay(100);
   if ( (sgn(diff) * sgn(last_diff) < 0) && ( abs(diff) > 10 )) {
     last_t = t;
     t = millis;
   }
   int per = t - last_t;
   per *= 2;
   float f = 1000.0 / per;
   sprintf(str, "f: %f", f);
   BSP_LCD_DisplayStringAtLine(12, str);
   HAL_Delay(100);
   BSP_LCD_ClearStringLine(12);
   BSP_LCD_ClearStringLine(2);
 }
  //sprintf(str, "Line %d", i);
  //BSP_LCD_DisplayStringAtLine(i, str);
 return 0;
}
//Appropriate clock configuration
static void SystemClock_Config(void)
{
 RCC_ClkInitTypeDef RCC_ClkInitStruct;
 RCC_OscInitTypeDef RCC_OscInitStruct;
 /* Enable Power Control clock */
 __HAL_RCC_PWR_CLK_ENABLE(); 
 /* The voltage scaling allows optimizing the power consumption when the device is
 clocked below the maximum system frequency, to update the voltage scaling value
 regarding system frequency refer to product datasheet. */
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

 /* Enable HSE Oscillator and activate PLL with HSE as source */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLM = 8;
 RCC_OscInitStruct.PLL.PLLN = 360;
 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
 RCC_OscInitStruct.PLL.PLLQ = 7;
 HAL_RCC_OscConfig(&RCC_OscInitStruct);
 /* Activate the Over-Drive mode */
 HAL_PWREx_EnableOverDrive();

 /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
 clocks dividers */
 RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void onLCD() {
  //Display initialization
 BSP_LCD_Init();
 BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
 BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
 BSP_LCD_DisplayOn();
 BSP_LCD_Clear(LCD_COLOR_WHITE);//Clear display
 BSP_LCD_SetBackColor(LCD_COLOR_WHITE);//Choose background color
 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);//Set work color, not only for text
}

static void initADC() {
  //ADC na APB2
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);  //taktirovanie ADC
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);  // on GPIO
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);  // on TIm
LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ANALOG);   // analog regim na PA3
LL_RCC_ClocksTypeDef RCCClocks;
LL_RCC_GetSystemClocksFreq(&RCCClocks);
int freq = RCCClocks.PCLK2_Frequency;
//SystemCoreClock = freq;
LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
//auto reload register
LL_TIM_SetAutoReload(TIM1, freq / 1000 / 1000 - 1);
//Prescaler
LL_TIM_SetPrescaler(TIM1, 1000 / 3 * 2);
LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);         //napravlenie schota vverh
LL_TIM_EnableIT_UPDATE(TIM1);  
LL_TIM_EnableCounter(TIM1);
LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_10B);      // razreshenie ADC
LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);    //vyravnivanie
LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);  // scan off
LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);  //
LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
LL_ADC_EnableIT_EOCS(ADC1);
LL_ADC_Enable(ADC1);
NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
NVIC_EnableIRQ(ADC_IRQn);
}

static inline int sgn(int x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}