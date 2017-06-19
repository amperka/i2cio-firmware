/**
  ******************************************************************************
  * File Name          : pin.c
  * Description        : This file provides code for the pin logic.
  ******************************************************************************
**/

#include "pin.h"


//#define VREFINT_CAL_ADDR                   ((uint16_t*) ((uint32_t)0x1FFFF7BAU)) /* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV). */
//#define VREFINT_CAL_VREF                   ((uint32_t) 3300U)                    /* Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */
/* Temperature sensor */
//#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) ((uint32_t)0x1FFFF7B8U)) /* Internal temperature sensor, address of parameter TS_CAL1: On STM32F0, temperature sensor ADC raw data acquired at temperature  30 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV). */
//#define TEMPSENSOR_CAL1_TEMP               (( int32_t)   30)                     /* Internal temperature sensor, temperature at which temperature sensor has been calibrated in production for data into TEMPSENSOR_CAL1_ADDR (tolerance: +-5 DegC) (unit: DegC). */
//#define TEMPSENSOR_CAL_VREFANALOG          ((uint32_t) 3300U)                    /* Analog voltage reference (Vref+) voltage with which temperature sensor has been calibrated in production (+-10 mV) (unit: mV). */


/**
Start of predeclaration
**/
void setPwmFreq(uint16_t freq);
uint16_t analogRead(uint8_t adcChNum);
void portMode(uint16_t Port, PinMode Mode);
uint16_t digitalReadPort();
void digitalWritePort(uint16_t Port, bool Value);
bool digitalRead(uint8_t Pin);
void digitalWrite(uint8_t Pin, bool Value);
void analogWrite(uint8_t Pin, uint16_t Value);
void setPwmMode(uint8_t Pin);
void resetPwmMode(uint8_t Pin);
void setPinMode(uint8_t Pin, uint8_t Mode);
PinMode getPinMode(uint8_t Pin);
bool isTimChIntEnable(TIM_HandleTypeDef *htim, uint32_t Channel);
bool isTimUpdIntEnable(TIM_HandleTypeDef *htim);
bool isOnlyTimUpdIntEnable(TIM_HandleTypeDef *htim);

/**
End of predeclaration
**/

void setPwmFreq(uint16_t freq)
{
	//TODO
}

uint16_t analogRead(uint8_t adcChNum)
{
	uint16_t result = 0xffff;

		if (adcChNum < ADC_COUNT) // Led not used as input!
		{
		  setPinMode(adcChNum, AnalogMode);
			result = adcValues[adcChNum];

		} 
	return result;
}

void portMode(uint16_t Port, PinMode Mode)
{
	for (uint8_t i = 0; i < GPIO_COUNT; ++i)
	{
		if (Port & 1)
		{
			setPinMode(i, Mode);
		}
		Port >>= 1;
	}
}

uint16_t digitalReadPort()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < GPIO_COUNT; ++i)
	{
		result <<= 1;
		if (digitalRead(i))
		{
			result |= 1;
		}
	}
	return result;
}

void digitalWritePort(uint16_t Port, bool Value)
{
	for (uint8_t i = 0; i < GPIO_COUNT; ++i)
	{
		if (Port & 1)
		{
			digitalWrite(i, Value);
		}
		Port >>= 1;
	}
}

bool digitalRead(uint8_t Pin)
{
	return (bool)(GPIO[Pin].Port->IDR & GPIO[Pin].Cfg.Pin);
}

void digitalWrite(uint8_t Pin, bool Value)
{
	GPIO_PinState state;
	if (Value)
	{
		GPIO[Pin].Port->BSRR = GPIO[Pin].Cfg.Pin;
		state = GPIO_PIN_SET;
	}
	else
	{
		GPIO[Pin].Port->BRR = GPIO[Pin].Cfg.Pin;
		state = GPIO_PIN_RESET;
	}

	setPinMode(Pin, OutputMode);
	// It's double set for good luck.
	// What if SoftPwm IRQ before setPinMode() happens?
  HAL_GPIO_WritePin(GPIO[Pin].Port, GPIO[Pin].Cfg.Pin, state);
}

void analogWrite(uint8_t Pin, uint16_t Value)
{
	if (Pin < GPIO_COUNT)
	{
		if (Value == 0)
		{
			digitalWrite(Pin, GPIO_PIN_RESET);
		}
		else if (Value >= TimCh[Pin].Htim->Init.Period)
		{
			digitalWrite(Pin, GPIO_PIN_SET);
		}
		else
		{
		  switch (TimCh[Pin].Channel)
		  {
		    case TIM_CHANNEL_1:
		    {
		    	TimCh[Pin].Htim->Instance->CCR1 = Value;
		    }
		    break;
		    case TIM_CHANNEL_2:
		    {
		    	TimCh[Pin].Htim->Instance->CCR2 = Value;
		    }
		    break;
		    case TIM_CHANNEL_3:
		    {
		    	TimCh[Pin].Htim->Instance->CCR3 = Value;
		    }
		    break;
		    case TIM_CHANNEL_4:
		    {
		    	TimCh[Pin].Htim->Instance->CCR4 = Value;
		    }
		    break;
		  }
			setPinMode(Pin, PwmMode);
		}
	}
}

void setPwmMode(uint8_t Pin)
{
	if (TimCh[Pin].IsHwPwm)
	{
		HAL_TIM_PWM_Start(TimCh[Pin].Htim
			                 , TimCh[Pin].Channel);
    GPIO[Pin].Cfg.Mode = GPIO_MODE_AF_PP;
	}
	else
	{
		if (!isTimUpdIntEnable(TimCh[Pin].Htim))
		{
			HAL_TIM_Base_Start_IT(TimCh[Pin].Htim);
		}
		HAL_TIM_PWM_Start_IT(TimCh[Pin].Htim
			                 , TimCh[Pin].Channel);
    GPIO[Pin].Cfg.Mode = GPIO_MODE_OUTPUT_PP;
    TimCh[Pin].SwPwmPinMask = GPIO[Pin].Cfg.Pin;
	}
  GPIO[Pin].Cfg.Pull = GPIO_NOPULL;
}

void resetPwmMode(uint8_t Pin)
{
	if (TimCh[Pin].IsHwPwm)
	{
		HAL_TIM_PWM_Stop(TimCh[Pin].Htim
                       , TimCh[Pin].Channel);
	}
	else
	{
		HAL_TIM_PWM_Stop_IT(TimCh[Pin].Htim
                          , TimCh[Pin].Channel);
  	TimCh[Pin].SwPwmPinMask = 0;
  	if (isOnlyTimUpdIntEnable(TimCh[Pin].Htim))
  	{
  		HAL_TIM_Base_Stop_IT(TimCh[Pin].Htim);
  	}
  }
}

// at first we need set value, then set mode
void setPinMode(uint8_t Pin, uint8_t Mode)
{
	PinMode currentMode = getPinMode(Pin);
	if (currentMode != Mode)
	{
		if (currentMode == PwmMode)
		{
			resetPwmMode(Pin);
		}
		switch (Mode)
	  {
	    case InputMode:
	    {
	      GPIO[Pin].Cfg.Mode = GPIO_MODE_INPUT;
        GPIO[Pin].Cfg.Pull = GPIO_NOPULL;
	    }
	    break;
	    case PullUpMode:
	    {
	      GPIO[Pin].Cfg.Mode = GPIO_MODE_INPUT;
	      GPIO[Pin].Cfg.Pull = GPIO_PULLUP;
	    }
	    break;
	    case PullDownMode:
	    {
	      GPIO[Pin].Cfg.Mode = GPIO_MODE_INPUT;
	      GPIO[Pin].Cfg.Pull = GPIO_PULLDOWN;
	    }
	    break;
	    case OutputMode:
	    {
	      GPIO[Pin].Cfg.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO[Pin].Cfg.Pull = GPIO_NOPULL;
	    }
	    break;
 	    case PwmMode:
	    {
	    	setPwmMode(Pin);
	    }
	    break;
	    case AnalogMode:
	    {
	      GPIO[Pin].Cfg.Mode = GPIO_MODE_ANALOG;
          GPIO[Pin].Cfg.Pull = GPIO_NOPULL;
	    }
	    break;
	  }
	  HAL_GPIO_Init(GPIO[Pin].Port, &GPIO[Pin].Cfg);	
	}
}

PinMode getPinMode(uint8_t Pin)
{
  PinMode result = AnalogMode;
  uint32_t gpioMode = GPIO[Pin].Cfg.Mode;
  uint32_t pullMode = GPIO[Pin].Cfg.Pull;
  switch (gpioMode)
  {
    case GPIO_MODE_INPUT:
    {
      if (pullMode == GPIO_NOPULL)
      {
  	  	result = InputMode;
  	  }
  	  else if (pullMode == GPIO_PULLUP)
  	  {
  	  	result = PullUpMode;
  	  }
  	  else if (pullMode == GPIO_PULLDOWN)
  	  {
  	  	result = PullDownMode;
  	  }
    }
    break;
    case GPIO_MODE_OUTPUT_PP:
    {
    	if (isTimChIntEnable(TimCh[Pin].Htim, TimCh[Pin].Channel))
    	{
    		result = PwmMode;
    	}
    	else
    	{
    		result = OutputMode;
    	}
    }
    break;
    case GPIO_MODE_AF_PP:
    {
    	result = PwmMode;
    }
    break;
    case GPIO_MODE_ANALOG:
    {
    	result = AnalogMode;
    }
    break;
  }
  return result;
}

bool isTimChIntEnable(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  bool result = false;
  uint32_t itReg = htim->Instance->DIER;
  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {
      if (itReg & TIM_IT_CC1)
      	result = true;
    }
    break;
    case TIM_CHANNEL_2:
    {
      if (itReg & TIM_IT_CC2)
      	result = true;
    }
    break;
    case TIM_CHANNEL_3:
    {
      if (itReg & TIM_IT_CC3)
      	result = true;
    }
    break;
    case TIM_CHANNEL_4:
    {
      if (itReg & TIM_IT_CC4)
      	result = true;
    }
    break;
  }
  return result;
}

bool isTimUpdIntEnable(TIM_HandleTypeDef *htim)
{
  bool result = false;
  uint32_t itReg = htim->Instance->DIER;
  if (itReg & TIM_IT_UPDATE)
   	result = true;
  return result;
}

bool isOnlyTimUpdIntEnable(TIM_HandleTypeDef *htim)
{
  bool result = false;
  uint32_t itReg = htim->Instance->DIER;
  if (itReg & TIM_IT_UPDATE)
  {
  	if (!(itReg & (TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4)))
  		result = true;
  }
  return result;
}
