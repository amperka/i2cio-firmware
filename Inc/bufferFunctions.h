#include "stm32f0xx_hal.h"

#define TEMP_SENSOR_CALIB_VAL_ADDR 0x1FFFF7B8U

uint32_t getUID(void);

uint16_t concat2U8toU16(uint8_t highVal, uint8_t lowVal);

void setAnswerBuf_16(uint8_t *answerBuf, uint16_t val);

void setAnswerBuf_32(uint8_t *answerBuf, uint32_t val);

uint32_t getBufData_32(uint8_t *dataBuf);
