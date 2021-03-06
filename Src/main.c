/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "pin.h"
#include "adcLoop.h"
#include "encoder.h"
/* USER CODE BEGIN Includes */
#include "bufferFunctions.h"
#include "i2cioCommands.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  uint8_t aRxBuffer[10];
  uint8_t aTxBuffer[10];
  uint32_t masterReadedUid= 0xffffffff;
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
  void SystemClock_Config(void);
  void Error_Handler(void);
  static void MX_NVIC_Init(void);
  static inline void prepareAnswer(uint8_t *commandBuf, uint8_t *answerBuf);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
  uint8_t getI2CAddress(void);
  void saveI2CAddress(uint8_t address);
  void prepareAnswer(uint8_t *commandBuf, uint8_t *answerBuf);
  static inline void builtInLedFader();

  uint8_t addr;
  uint8_t recieveMessageFlag = 0;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
  void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
   LED1_GPIO_Port->BRR |= LED1_Pin;
   recieveMessageFlag = 1;
//   HAL_I2C_EnableListen_IT(hi2c);
 }

 void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
 {
  LED1_GPIO_Port->BSRR |= LED1_Pin;
  if (TransferDirection) {
    HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, aTxBuffer, 4, I2C_FIRST_AND_LAST_FRAME);
  } else {
    HAL_I2C_Slave_Sequential_Receive_IT(hi2c, aRxBuffer, 5, I2C_FIRST_AND_LAST_FRAME);
  }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_I2C_DisableListen_IT(&hi2c1);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  
  MX_GPIO_Init();
//  GPIOF->BSRR |= GPIO_PIN_1;

  MX_I2C1_Init();

  InitTimers();

  MX_ADC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  set_I2C_addr(addr=getI2CAddress());
  HAL_I2C_EnableListen_IT(&hi2c1);

  HAL_ADC_Start(&hadc);

  initEncoders();

  while (!recieveMessageFlag)
  {
    HAL_ADCEx_Calibration_Start(&hadc);
    // we need prepare ADC before extern microcontroller
    HAL_ADC_ConvCheck(&hadc);
    builtInLedFader();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (recieveMessageFlag)
    {
      prepareAnswer(aRxBuffer, aTxBuffer);
      recieveMessageFlag = 0;
      HAL_I2C_EnableListen_IT(&hi2c1);
    }

    HAL_ADC_ConvCheck(&hadc);
    encoderCapture();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
  |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* I2C1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* ADC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
}

/* USER CODE BEGIN 4 */

static inline void builtInLedFader()
{
  // boot indication^
  static bool dig = false;
  static uint8_t bootPwmFreqCounter = 0;
  static uint8_t bootPwmCounter = 0;
  static uint8_t timeCounter = 0;
  static uint32_t lastTick = 0;

  if (HAL_GetTick() != lastTick) {
    ++bootPwmFreqCounter;
    if (bootPwmFreqCounter > 15)
    {
      bootPwmFreqCounter = 0;
      ++ timeCounter;
      if (timeCounter > 2)
      {
        timeCounter = 0;
        if ((bootPwmCounter > 9) || (bootPwmCounter < 1))
        {
          dig = !dig;
        }
        if (dig)
          ++bootPwmCounter;
        else
          --bootPwmCounter;
      }
    }
    if (bootPwmFreqCounter > bootPwmCounter)
      LED1_GPIO_Port->BRR |= LED1_Pin;
    else
      LED1_GPIO_Port->BSRR |= LED1_Pin;

    lastTick = HAL_GetTick();
  }
}

uint8_t getI2CAddress(void)
{
  uint8_t result = HAL_FLASHEx_OBGetUserData(OB_DATA_ADDRESS_DATA0);
  if (result == 0xFF) {
    saveI2CAddress(result = DEFAULT_I2C_ADDR);
  }
  return result;
}

void saveI2CAddress(uint8_t address) {

  HAL_FLASH_Unlock();

    /* Unlock the Options Bytes *************************************************/
  HAL_FLASH_OB_Unlock();

  if (HAL_FLASHEx_OBErase() != HAL_OK) {
    Error_Handler();
  }

  static FLASH_OBProgramInitTypeDef OptionsBytesStruct;
  OptionsBytesStruct.OptionType = OPTIONBYTE_DATA;
  OptionsBytesStruct.DATAAddress = OB_DATA_ADDRESS_DATA0;
  OptionsBytesStruct.DATAData = address;

  if (HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK) {
    Error_Handler();
  }
  HAL_FLASH_OB_Launch();
}

static inline void prepareAnswer(uint8_t *commandBuf, uint8_t *answerBuf){

  uint8_t i = commandBuf[0];
  switch (i) {

    case UID:
    {
      setAnswerBuf_32(answerBuf, getUID());
    }
    break;

    case RESET_SLAVE:
    {
      NVIC_SystemReset();
    }
    break;    

    case CHANGE_I2C_ADDR:
    {
      set_I2C_addr(addr = commandBuf[1]);
//      HAL_I2C_EnableListen_IT(&hi2c1);
    }
    break;

    case SAVE_I2C_ADDR:
    {
      if (addr!=getI2CAddress()) {
        saveI2CAddress(addr);
      }
    }
    break;

    case PORT_MODE_INPUT:
    {
      portMode(concat2U8toU16(commandBuf[1]
        , commandBuf[2])
      , InputMode);
    }
    break;

    case PORT_MODE_PULLUP:
    {
      portMode(concat2U8toU16(commandBuf[1]
        , commandBuf[2])
      , PullUpMode);
    }
    break;

    case PORT_MODE_PULLDOWN: //TODO : add it to ioCommands
    {
      portMode(concat2U8toU16(commandBuf[1]
        , commandBuf[2])
      , PullDownMode);
    }

    case PORT_MODE_OUTPUT:
    {
      portMode(concat2U8toU16(commandBuf[1]
        , commandBuf[2])
      , OutputMode);
    }
    break;

    case DIGITAL_READ:
    {
      setAnswerBuf_16(answerBuf, digitalReadPort());
    }
    break;

    case DIGITAL_WRITE_HIGH:
    {
      digitalWritePort(concat2U8toU16(
        commandBuf[1]
        , commandBuf[2])
      , true);
    }
    break;

    case DIGITAL_WRITE_LOW:
    {
      digitalWritePort(concat2U8toU16(
        commandBuf[1]
        , commandBuf[2])
      , false);
    }
    break;

    case PWM_FREQ:
    {
      setPwmFreq(concat2U8toU16(commandBuf[1]
        , commandBuf[2]));
    }
    break;

    case ANALOG_WRITE:
    {
      analogWrite(commandBuf[1]
        , concat2U8toU16(commandBuf[2]
         , commandBuf[3]));
    }
    break;

    case ANALOG_READ:
    {
      setAnswerBuf_16(answerBuf, analogRead(commandBuf[1]));
    }
    break;

    case ADC_SPEED:
    {
      setAdcSpeed(commandBuf[1]);
    }
    break;

    case MASTER_READED_UID:
    {
      masterReadedUid = getBufData_32(commandBuf);
    }
    break;
    case CHANGE_I2C_ADDR_IF_UID_OK:
    {
      if (masterReadedUid == getUID()){
        set_I2C_addr(addr = commandBuf[1]);
      }
      masterReadedUid = 0xffffffff;
    }
    break;
    case SAY_SLOT:
    {
      uint32_t slot = SLOT; 
      setAnswerBuf_32(answerBuf, slot);
    }
    break;

    case ADC_LOWPASS_FILTER_ON:
    {
      adcLowPassFilterSwitcher(true);
    }
    break;

    case ADC_LOWPASS_FILTER_OFF:
    {
      adcLowPassFilterSwitcher(false);
    }
    break;

    case ADC_AS_DIGITAL_PORT_SET_TRESHOLD:
    {
      adcAsDigitalTreshold = concat2U8toU16(commandBuf[1]
        , commandBuf[2]);
    }
    break;

    case ADC_AS_DIGITAL_PORT_READ:
    {
      setAnswerBuf_16(answerBuf, adcAsDigitalPortRead(adcAsDigitalTreshold));
    }
    break;

    case ENCODER_SET_PINS:
    {
      setEncoderPins(commandBuf[1], ((commandBuf[2] >> 4) & 0x0f), (commandBuf[2] & 0x0f));
    }
    break;

    case ENCODER_GET_DIFF_VALUE:
    {
      answerBuf[0] = (uint8_t)getValueEncoder(commandBuf[1]);
    }
    break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
  void Error_Handler(void)
  {
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      HAL_Delay(330);
    }
  /* USER CODE END Error_Handler */ 
  }

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
   void assert_failed(uint8_t* file, uint32_t line)
   {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

   }

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
