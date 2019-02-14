#include "adcLoop.h"
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

#define ADC_FILTER_SH 2

uint8_t speed = 0;
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
volatile uint32_t accum = 0;


// try without interrupts. Regular check
bool HAL_ADC_ConvCheck(ADC_HandleTypeDef* hadc)
{
  bool fullCycle = false;

  if (HAL_IS_BIT_SET(hadc->Instance->ISR, ADC_FLAG_EOC))//__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC))
  {

//    accum = HAL_ADC_GetValue(hadc);

 //   bool nextChannelEn = false;

      adcValues[adcIndex] = HAL_ADC_GetValue(hadc);
//      nextChannelEn = true;

/*    if (nextChannelEn)
    {
      ++adcIndex;

      HAL_ADC_Stop(hadc);
*/
/*
      if (adcIndex >= ADC_COUNT)
      {
        adcIndex = 0;
        HAL_ADCEx_Calibration_Start(hadc);
        fullCycle = true;
      }
*/
            /*select next channel*/
//      hadc->Instance->CHSELR = adcChannel[adcIndex];
            /* Clear the old sample time */
//      hadc->Instance->SMPR &= ~(ADC_SMPR_SMP);
            /* Set the new sample time */
/*      if (currentAdcSpeed != speed)
      {
        speed = currentAdcSpeed;
        hadc->Instance->SMPR |= ADC_SMPR_SET(adcSpeed[(speed)]);
      }
      HAL_ADC_Start(hadc);
    }
    */
  }
  return fullCycle;
}