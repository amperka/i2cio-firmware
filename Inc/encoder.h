/**
  ******************************************************************************
  * File Name          : encoder.h
  * Description        : This file provides code for the encoder logic.
  ******************************************************************************
**/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __encoder_H
#define __encoder_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f0xx_hal.h"
//#include "main.h"
//#include "gpio.h"
//#include "tim.h"
#include "pin.h"

/* USER CODE BEGIN */

#define MAX_ENCODER_COUNT 4

void initEncoders();
void setEncoderPins(uint8_t encoder, uint8_t pinA, uint8_t pinB);
int8_t getValueEncoder(uint8_t encoder);
void encoderCapture();
/* USER CODE END */

#ifdef __cplusplus
}
#endif
#endif /*__encoder_H */
