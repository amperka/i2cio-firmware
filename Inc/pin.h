/**
  ******************************************************************************
  * File Name          : pin.h
  * Description        : This file provides code for the pin logic.
  ******************************************************************************
**/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __pin_H
#define __pin_H
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
typedef enum {
  InputMode
  , PullUpMode
  , PullDownMode
  , OutputMode
  , PwmMode
  , AnalogMode
} PinMode;

void setPwmFreq(uint16_t freq);
uint16_t analogRead(uint8_t adcChNum);
uint16_t digitalReadPort();
void digitalWritePort(uint16_t Port, bool Value);
bool digitalRead(uint8_t Pin);
void digitalWrite(uint8_t Pin, bool Value);
void analogWrite(uint8_t Pin, uint16_t Value);
void portMode(uint16_t Port, PinMode Mode);

/* USER CODE END */

#ifdef __cplusplus
}
#endif
#endif /*__ pin_H */
