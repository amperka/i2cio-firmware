/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define GPIO_COUNT 10

#define P0_T3C4_H_Pin GPIO_PIN_1
#define P1_T17C1_H_Pin GPIO_PIN_7
#define P2_T16C1_H_Pin GPIO_PIN_6
#define P3_T1C1_Pin GPIO_PIN_5
#define P4_T1C2_Pin GPIO_PIN_0
#define P5_T1C4_Pin GPIO_PIN_1
#define P6_T3C1_Pin GPIO_PIN_2
#define P7_T14C1_H_Pin GPIO_PIN_4
#define P8_T3C2_Pin GPIO_PIN_3
/* LED - P9_T3C3 */
#define LED1_Pin GPIO_PIN_1
#define P9_T3C3_Pin LED1_Pin

#define P0_T3C4_H_GPIO_Port GPIOB
#define P1_T17C1_H_GPIO_Port GPIOA
#define P2_T16C1_H_GPIO_Port GPIOA
#define P3_T1C1_GPIO_Port GPIOA
#define P4_T1C2_GPIO_Port GPIOA
#define P5_T1C4_GPIO_Port GPIOA
#define P6_T3C1_GPIO_Port GPIOA
#define P7_T14C1_H_GPIO_Port GPIOA
#define P8_T3C2_GPIO_Port GPIOA
#define LED1_GPIO_Port GPIOF
#define P9_T3C3_GPIO_Port LED1_GPIO_Port
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
