/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"
#include "main.h"


/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC interrupt.
*/
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
* @brief This function handles TIM1 break, update, trigger and commutation interrupts.
*/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /*
  uint32_t statusReg = htim1.Instance->SR;
  uint32_t interruptEnableReg = htim1.Instance->DIER;

  if ((interruptEnableReg & TIM_IT_UPDATE) && (statusReg & TIM_FLAG_UPDATE))
  {
*/
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    setPullups(pullBotom);
    TimCh[3].Htim->Instance->CCR1 = pwmValue;
//  }
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /*
  uint32_t interruptEnableReg = htim1.Instance->DIER;

  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) != RESET)
  {
    if (interruptEnableReg & TIM_IT_CC1)
    {
      */
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);  
      //P3_T1C1_Pin
      setPullups(pullTop);
//    }
//  }
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{

  uint32_t statusReg = htim3.Instance->SR;
  uint32_t interruptEnableReg = htim3.Instance->DIER;

  if ((interruptEnableReg & TIM_IT_UPDATE) && (statusReg & TIM_FLAG_UPDATE))
  {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    GPIO[9].Port->BSRR = TimCh[9].SwPwmPinMask;
    GPIO[6].Port->BSRR = TimCh[8].SwPwmPinMask
                       | TimCh[6].SwPwmPinMask;
  }

  if ((interruptEnableReg & TIM_IT_CC3) && (statusReg & TIM_FLAG_CC3))
  {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);  
    //P9_T3C3_Pin
    GPIO[9].Port->BRR = TimCh[9].SwPwmPinMask;
    htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  }
  if ((interruptEnableReg & TIM_IT_CC2) && (statusReg & TIM_FLAG_CC2))
  {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);  
    //P8_T3C2_Pin
    GPIO[8].Port->BRR = TimCh[8].SwPwmPinMask;
    htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  }
  if ((interruptEnableReg & TIM_IT_CC1) && (statusReg & TIM_FLAG_CC1))
  {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);  
    //P6_T3C1_Pin
    GPIO[6].Port->BRR = TimCh[6].SwPwmPinMask;
    htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
  }
}

/**
* @brief This function handles I2C1 global interrupt.
*/
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c1);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c1);
  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
