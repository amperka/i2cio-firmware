/**
  ******************************************************************************
  * File Name          : encoder.c
  * Description        : This file provides code for the encoder logic.
  ******************************************************************************
**/

#include "encoder.h"

typedef struct
{
  uint8_t pinA;
  uint8_t pinB;
  bool lastStateA;
  bool lastStateB;
  int8_t unreadedValue;
} Encoder_Type;

Encoder_Type encoders[MAX_ENCODER_COUNT];

uint8_t encoderCount = 0;
const uint16_t treshold = 0x7FF;

static inline bool pinState(uint8_t pin){
	return (analogRead(pin) > treshold);
}

void addEncoder(uint8_t pinA, uint8_t pinB)
{
	if (encoderCount < MAX_ENCODER_COUNT - 1){
		if ((pinA < ADC_COUNT) && (pinB < ADC_COUNT)){
			encoders[encoderCount].pinA = pinA;
			encoders[encoderCount].pinB = pinB;
			encoders[encoderCount].lastStateA = pinState(pinA);
			encoders[encoderCount].lastStateB = pinState(pinB);
			encoders[encoderCount].unreadedValue = 0;
			encoderCount++;
		}
	}
}

int8_t getValueEncoder(uint8_t encoder){
	int8_t result = 0;
	if (encoder < encoderCount){
		result = encoders[encoder].unreadedValue;
		encoders[encoder].unreadedValue = 0;
	}
	return result;
}

static inline bool detect(bool x1, bool x2, bool y1, bool y2) 
{
	return (x2 ^ y1) & ~(x1 ^ y2);
}

void encoderCapture(){
	for (uint8_t i = 0; i < encoderCount; ++i){
		bool stateA = pinState(encoders[i].pinA);
		bool stateB = pinState(encoders[i].pinB);
		bool _stateA = encoders[i].lastStateA;
		bool _stateB = encoders[i].lastStateB;
		
		if (detect(_stateA, _stateB, stateA, stateB))
			encoders[i].unreadedValue++;
		else if (detect(_stateB, _stateA, stateB, stateA))
			encoders[i].unreadedValue--;

		encoders[i].lastStateA = stateA;
		encoders[i].lastStateB = stateB;
	}
}
