#include "bufferFunctions.h"

uint16_t concat2U8toU16(uint8_t highVal, uint8_t lowVal){
  uint16_t result = highVal;
  result <<= 8;
  result |= lowVal;
  return result;
}

void setAnswerBuf_16(uint8_t *answerBuf, uint16_t val){
  answerBuf[1] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[0] = val & (uint8_t)0xFF;
}

void setAnswerBuf_32(uint8_t *answerBuf, uint32_t val){
  answerBuf[3] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[2] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[1] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[0] = val & (uint8_t)0xFF;
}

uint32_t getBufData_32(uint8_t *dataBuf){
  uint32_t result = dataBuf[4] | (dataBuf[3]<<8) | (dataBuf[2]<<16) | (dataBuf[1]<<24);
  return result;
}

uint32_t getUID(){
  uint32_t result;
  uint32_t* calibVal = (uint32_t*)TEMP_SENSOR_CALIB_VAL_ADDR;
  result = *calibVal;
  return  result;
}