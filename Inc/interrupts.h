#ifndef __interrupts_H
#define __interrupts_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "adc.h"

/* USER CODE BEGIN */
void HAL_ADC_ConvCheck(ADC_HandleTypeDef* hadc);
/* USER CODE END */

#ifdef __cplusplus
}
#endif
#endif /*__interrupts_H */
