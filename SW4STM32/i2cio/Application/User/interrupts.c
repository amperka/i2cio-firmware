#include "interrupts.h"

uint8_t adcToPin[] = 
{
    4
  , 5
  , 6
  , 8
  , 7
  , 3
  , 2
  , 1
  , 0
  , 9
  , 10
};

uint8_t adcIndex=0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
    {
      adcValues[adcToPin[adcIndex]] = HAL_ADC_GetValue(hadc);
//    adcValues[adcToPin[adcIndex]] += HAL_ADC_GetValue(hadc);
//    adcValues[adcToPin[adcIndex]] >>= 1;
    adcIndex++;
    }
 
  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
    {
    adcIndex=0;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
      //P9_T3C3_Pin
      GPIO[9].Port->BRR = TimCh[9].SwPwmPinMask;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      //P8_T3C2_Pin
      GPIO[8].Port->BRR = TimCh[8].SwPwmPinMask;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      //P6_T3C1_Pin
      GPIO[6].Port->BRR = TimCh[6].SwPwmPinMask;
    }
  }
  if (htim->Instance == TIM1)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      //P5_T1C4_Pin
      GPIO[5].Port->BRR = TimCh[5].SwPwmPinMask;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      //P4_T1C2_Pin
      GPIO[4].Port->BRR = TimCh[4].SwPwmPinMask;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      //P3_T1C1_Pin
      GPIO[3].Port->BRR = TimCh[3].SwPwmPinMask;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    //P9_T3C3_Pin__GPIOF
    //P8_T3C2_Pin__GPIOA
    //P6_T3C1_Pin__GPIOA
    GPIO[9].Port->BSRR = TimCh[9].SwPwmPinMask;
    GPIO[6].Port->BSRR = TimCh[8].SwPwmPinMask
                       | TimCh[6].SwPwmPinMask;
  }
  if (htim->Instance == TIM1)
  {
    //P5_T1C4_Pin__GPIOA
    //P4_T1C2_Pin__GPIOA
    //P3_T1C1_Pin__GPIOA
    GPIO[3].Port->BSRR = TimCh[5].SwPwmPinMask
                       | TimCh[4].SwPwmPinMask
                       | TimCh[3].SwPwmPinMask;
  }
}