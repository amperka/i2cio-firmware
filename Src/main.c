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


#define P_SIZE 21
#define PULLUPDOWN_MASK 0xFFFF0000U
#define SECTOR_WIDTH 8192U
#define LEFT_EDGE 2U
#define RIGHT_EDGE 62U
#define MAX_DIVIDER 64U
#define DEPTH MAX_DIVIDER/2 - 1

volatile uint16_t INSTRUCTIONCOUNTER = 0;

static const uint16_t pullups[]={
43689
, 43681
, 43649
, 43521
, 43685
, 43653
, 43525
, 43669
, 43013
, 43541
, 43605
, 43093
, 40981
, 43349
, 41045
, 41301
, 42325
, 32853
, 33109
, 34133
, 38229
};

static const uint16_t pullupsVal[]={
  0
  , 1560
  , 3641
  , 6554
  , 10923
  , 14043
  , 18204
  , 21845
  , 24030
  , 26526
  , 32768
  , 39010
  , 41506
  , 43691
  , 47332
  , 51493
  , 54613
  , 58982
  , 61895
  , 63976
  , 65535 // 0
};

static const uint16_t multTo24Bit[]={
5376
,  4032
,  2880
,  1920
,  2688
,  2016
,  2304
,  3840
,  3360
,  1344
,  1344
,  3360
,  3840
,  2304
,  2016
,  2688
,  1920
,  2880
,  4032
,  5376
};
// all comparision will be in 8192 width value
static inline uint16_t pwmValTo14Bit(uint16_t val, uint8_t bottomNumber) {
  uint16_t result = ((uint32_t)val * (uint32_t)multTo24Bit[bottomNumber]) >> 10;
  return result;
}

volatile uint32_t pullBotom = 0;
volatile uint32_t pullTop = 0;
volatile uint16_t sectorWidth = SECTOR_WIDTH;
volatile uint16_t sectorValue = 0;
volatile uint8_t pwmWidth = 0;
volatile uint8_t pwmValue = 0;

static inline void findPullups(uint16_t value) {
  uint16_t bottom = 0;
  uint16_t top = 0;
  /*
  uint8_t i = 0;
  bool finding = true;

  while (finding) {
    bottom = pullupsVal[i];
    top = pullupsVal[i+1];

    if ((top >= value) && (bottom > value)) {
      i++;
    } else {
      finding = false;
    }
  }
  */
  uint8_t bottomNumber = 0;
  // we can do search better
  for (uint8_t i = 0; i < P_SIZE-1; ++i){
    bottom = pullupsVal[i];
    top = pullupsVal[i+1];
    if (bottom < value) {
      if (top >= value) {
        pullBotom = (uint32_t)pullups[i];
        pullTop = (uint32_t)pullups[i+1];
        bottomNumber = i;
      }
    }
  }
  //sectorWidth = top - bottom;
  value = value - bottom;
  sectorValue = pwmValTo14Bit(value, bottomNumber);
  sectorWidth = SECTOR_WIDTH;
  //sectorWidth = 8192;
}

static inline bool reduceEdge(){
  bool result = false;
  uint16_t leftEdgeValue = LEFT_EDGE << 7; // 2/64 to 8192
  uint16_t rightEdgeValue = 8192 - (RIGHT_EDGE<<7); // 62/64 to 8192
  if (sectorValue <= leftEdgeValue || sectorValue >= rightEdgeValue) {
    pwmWidth=sectorWidth>>7;
    pwmValue=sectorValue>>7;
    result = true;
  }
  return result;
}


static inline bool to64(){
//  bool result = false;
//  uint16_t leftEdgeValue = LEFT_EDGE << 7; // 2/64 to 8192
//  uint16_t rightEdgeValue = 8192 - (RIGHT_EDGE<<7); // 62/64 to 8192
//  if (sectorValue <= leftEdgeValue || sectorValue >= rightEdgeValue) {
    pwmWidth=sectorWidth>>7;
    pwmValue=sectorValue>>7;
//    result = true;
//  }
//  return result;
}

static inline void reduceFraction() {
  uint16_t to_cut = sectorWidth | sectorValue;
  uint8_t shift = 0;
  while (!(to_cut & 1U)) {
    ++shift;
    to_cut>>=1;
  }
  sectorWidth>>=shift;
  sectorValue>>=shift;
}

static inline void findFraction(uint8_t depth, uint8_t maxDivider) {
  uint16_t a = 0;
  uint16_t b = 1;
  uint16_t c = 1;
  uint16_t d = 1;
  uint16_t m = 0;
  uint16_t n = 0;
  uint8_t currentDepth = 0;

  while ((currentDepth < depth) && (!((sectorWidth == n) && (sectorValue == m)))) {
    currentDepth++;
    m = a + c;
    n = b + d;
      //TimCh[1].Htim->Instance->CNT = 0;

    bool goRight = (uint32_t)sectorValue * (uint32_t)n < (uint32_t)sectorWidth * (uint32_t)m;

      //INSTRUCTIONCOUNTER = TimCh[1].Htim->Instance->CNT;

    if (n >= maxDivider) break;
    if (goRight) {
      c = m;
      d = n;
    } else {
      a = m;
      b = n;
    }
  }
  pwmWidth = n;
  pwmValue = m;
}

inline void setPullups(uint32_t pulls)
{
  uint32_t temp = GPIOA->PUPDR;
  temp &= PULLUPDOWN_MASK;
  temp |= pulls;
  GPIOA->PUPDR = temp;
}

/// Fast 16-bit approximation of sin(x). This approximation never varies more than
/// 0.69% from the floating point value you'd get by doing
///
///     float s = sin(x) * 32767.0;
///
/// @param theta input angle from 0-65535
/// @returns sin of theta, value between -32767 to 32767.
int16_t sin16_C( uint16_t theta )
{
  static const uint16_t base[] =
  { 0, 6393, 12539, 18204, 23170, 27245, 30273, 32137 };
  static const uint8_t slope[] =
  { 49, 48, 44, 38, 31, 23, 14, 4 };

  uint16_t offset = (theta & 0x3FFF) >> 3; // 0..2047
  if( theta & 0x4000 ) offset = 2047 - offset;

    uint8_t section = offset / 256; // 0..7
    uint16_t b   = base[section];
    uint8_t  m   = slope[section];

    uint8_t secoffset8 = (uint8_t)(offset) / 2;

    uint16_t mx = m * secoffset8;
    int16_t  y  = mx + b;

    if( theta & 0x8000 ) y = -y;

    return y;
}

static inline void setPwm(uint16_t value){
  findPullups(value);
  if (!reduceEdge()) {
    reduceFraction();
    findFraction(DEPTH, MAX_DIVIDER);
  }

 // TimCh[3].Htim->Instance->ARR = sectorWidth;
  TimCh[3].Htim->Init.Period = pwmWidth;
  //TimCh[3].Htim->Instance->CCR1 = sectorValue;
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
/*
    while (!recieveMessageFlag)
    {
      HAL_ADCEx_Calibration_Start(&hadc);
    // we need prepare ADC before extern microcontroller
      HAL_ADC_ConvCheck(&hadc);
      builtInLedFader();
    }
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    // timeOperationsCounter
        HAL_TIM_PWM_Start(TimCh[1].Htim
                       , TimCh[1].Channel);


//    setPwmMode(3);
//    TimCh[3].Htim->Instance->ARR = 64;
//    TimCh[3].Htim->Instance->CCR1 = 32;


    uint16_t increment = 0;
    uint16_t mul=0;
    uint16_t loud=0;
//setPwm(3450);
      uint8_t shr = 5;


      
    while (1)
    {
//      if (!increment) mul = (mul+10);
//      increment+=512;//1512;
      loud--;
      mul++;
      uint16_t i = 0;

      while (i<128){++i;}
      increment+=i*9;

      if (mul>8192){
        shr>>=1;
        if (!shr){
          shr=8;
        }
        mul = 0;
      }

      TimCh[1].Htim->Instance->CNT = 0;

      int16_t sinValue_1 = sin16_C(increment);

      INSTRUCTIONCOUNTER = TimCh[1].Htim->Instance->CNT;



//      int16_t sinValue_1 = sin16_C(increment);
      


      int16_t sinValue_2 = sin16_C(increment*2);
      int16_t sinValue_3 = sin16_C(increment*5);
      int16_t sinValue_4 = sin16_C(increment*7);
      int16_t sinValue_5 = sin16_C(increment*11);
    //  int32_t curVal = (int32_t)sinValue_1;
    /*mul;/* + (int32_t)sinValue_4*mul;/*+
      (int32_t)sinValue_3*mul*3 +  (int32_t)sinValue_4*mul*2 +  (int32_t)sinValue_5*mul;
      */
//      uint32_t val32 = curVal/2 + 4294967295/2;

//      int32_t sinValue = sin16_C(increment+sin16_C(mul)*7) + sin16_C(mul*11+increment*5)/5 + sin16_C(sin16_C(mul)*mul*2+increment*3)/4 + sin16_C(sin16_C(mul)+32767)/10;

      uint16_t data1 = (sinValue_3>>shr)+sinValue_1/2 + 32767;//val32>>16; //sinValue/2 + 32767;
      uint16_t data2 = sinValue_1/2 + 32767;//val32>>16; //sinValue/2 + 32767;

      uint32_t val32 = data2*loud+32767*(65535-loud);

      uint16_t val = val32>>16;

    //  setPwm(data);
      findPullups(val);

//      TimCh[1].Htim->Instance->CNT = 0;

setPwm(data2);

//      INSTRUCTIONCOUNTER = TimCh[1].Htim->Instance->CNT;


      if (val>32767) {
        uint32_t pull_tmp = pullBotom;
        pullBotom = pullTop;
        pullTop = pull_tmp;
      }

      to64();
//      setPwm(data2);

      TimCh[3].Htim->Instance->CCR1 = pwmValue;
//      reduceFraction();
//      findFraction(10, 64);
//      setPullups(pullBotom);
      //  HAL_Delay(1);

//HAL_Delay(1);
    


 /*
      if (recieveMessageFlag)
      {
        prepareAnswer(aRxBuffer, aTxBuffer);
        recieveMessageFlag = 0;
        HAL_I2C_EnableListen_IT(&hi2c1);
      }

      HAL_ADC_ConvCheck(&hadc);
//    encoderCapture();
*/
      /*
      for (int i =0; i< P_SIZE; ++i){
        setPullups(pullups[i]);
        HAL_Delay(1);
      }
      */



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
