/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

GPIO_Type GPIO[GPIO_COUNT];

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  for (uint8_t i = 0; i < GPIO_COUNT; ++i) {
    GPIO[i].Cfg.Mode = GPIO_MODE_ANALOG;
    GPIO[i].Cfg.Pull = GPIO_NOPULL;
    GPIO[i].Cfg.Speed = GPIO_SPEED_FREQ_LOW;
  }

  GPIO[0].Cfg.Pin = P0_T3C4_H_Pin;
  GPIO[1].Cfg.Pin = P1_T17C1_H_Pin;
  GPIO[2].Cfg.Pin = P2_T16C1_H_Pin;
  GPIO[3].Cfg.Pin = P3_T1C1_Pin;
  GPIO[4].Cfg.Pin = P4_T1C2_Pin;
  GPIO[5].Cfg.Pin = P5_T1C4_Pin;
  GPIO[6].Cfg.Pin = P6_T3C1_Pin;
  GPIO[7].Cfg.Pin = P7_T14C1_H_Pin;
  GPIO[8].Cfg.Pin = P8_T3C2_Pin;
  GPIO[9].Cfg.Pin = P9_T3C3_Pin;

  GPIO[0].Port = P0_T3C4_H_GPIO_Port;
  GPIO[1].Port = P1_T17C1_H_GPIO_Port;
  GPIO[2].Port = P2_T16C1_H_GPIO_Port;
  GPIO[3].Port = P3_T1C1_GPIO_Port;
  GPIO[4].Port = P4_T1C2_GPIO_Port;
  GPIO[5].Port = P5_T1C4_GPIO_Port;
  GPIO[6].Port = P6_T3C1_GPIO_Port;
  GPIO[7].Port = P7_T14C1_H_GPIO_Port;
  GPIO[8].Port = P8_T3C2_GPIO_Port;
  GPIO[9].Port = P9_T3C3_GPIO_Port;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO[9].Cfg.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_WritePin(GPIO[9].Port, P9_T3C3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_Init(GPIO[9].Port, &GPIO[9].Cfg);

/*
  !!!!!!!!!
  HAL_GPIO_WritePin(P3_T1C1_GPIO_Port, P3_T1C1_Pin, GPIO_PIN_RESET);
!!!!!!!!!!!
  GPIO.Cfg.Pin = P6_T3C1_Pin;
  HAL_GPIO_Init(P6_T3C1_GPIO_Port, &GPIO.Cfg);

  GPIO.Cfg.Pin = P8_T3C2_Pin;
  HAL_GPIO_Init(P8_T3C2_GPIO_Port, &GPIO.Cfg);

  GPIO.Cfg.Pin = P3_T1C1_Pin;
  HAL_GPIO_Init(P3_T1C1_GPIO_Port, &GPIO.Cfg);
*/
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
