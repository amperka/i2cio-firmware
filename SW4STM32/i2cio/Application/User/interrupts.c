#include "interrupts.h"
/*
#define P0_T3C4_H_Pin GPIO_PIN_1
#define P1_T17C1_H_Pin GPIO_PIN_7
#define P2_T16C1_H_Pin GPIO_PIN_6
#define P3_T1C1_Pin GPIO_PIN_5
#define P4_T1C2_Pin GPIO_PIN_0
#define P5_T1C4_Pin GPIO_PIN_1
#define P6_T3C1_Pin GPIO_PIN_2
#define P7_T14C1_H_Pin GPIO_PIN_4
#define P8_T3C2_Pin GPIO_PIN_3
*/
#define ADCh0 ADC_CHSELR_CHANNEL(ADC_CHANNEL_9)
#define ADCh1 ADC_CHSELR_CHANNEL(ADC_CHANNEL_7)
#define ADCh2 ADC_CHSELR_CHANNEL(ADC_CHANNEL_6)
#define ADCh3 ADC_CHSELR_CHANNEL(ADC_CHANNEL_5)
#define ADCh4 ADC_CHSELR_CHANNEL(ADC_CHANNEL_0)
#define ADCh5 ADC_CHSELR_CHANNEL(ADC_CHANNEL_1)
#define ADCh6 ADC_CHSELR_CHANNEL(ADC_CHANNEL_2)
#define ADCh7 ADC_CHSELR_CHANNEL(ADC_CHANNEL_4)
#define ADCh8 ADC_CHSELR_CHANNEL(ADC_CHANNEL_3)


uint32_t adcChannel[] = 
{
    ADCh0
  , ADCh1
  , ADCh2
  , ADCh3
  , ADCh4
  , ADCh5
  , ADCh6
  , ADCh7
  , ADCh8
};

uint8_t adcConversionCount = 0; // we have wery high resistive load
uint8_t adcIndex = 0; // as init in adc.c = ADCh0
uint16_t accum=0;


// try without interrupts. Regular check
void HAL_ADC_ConvCheck(ADC_HandleTypeDef* hadc)
{
  if (HAL_IS_BIT_SET(hadc->Instance->ISR, (ADC_FLAG_EOC | ADC_FLAG_EOS)))//__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
    {
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS));

      uint8_t nextChennelEn = 0;

      if (adcConversionCount < 3)
      {
//        static uint16_t val;
        accum = HAL_ADC_GetValue(hadc);
      }
      else if (adcConversionCount < 9)
      {
        accum += HAL_ADC_GetValue(hadc);
      } else 
      {
        accum += HAL_ADC_GetValue(hadc);
        nextChennelEn = 1;
      }

      ++adcConversionCount;

      if (nextChennelEn)
      {
        adcValues[adcIndex] = accum>>3;
        accum = 0;

        adcConversionCount = 0;

        ++adcIndex;
        if (adcIndex >= ADC_COUNT)
        {
          adcIndex = 0;
        }

        HAL_ADC_Stop(hadc);
        HAL_ADCEx_Calibration_Start(hadc);
        hadc->Instance->CHSELR = adcChannel[adcIndex];
        HAL_ADC_Start(hadc);
      }
      
    }
 /*
  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))
    {
    adcIndex=0;
    }
    */
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
