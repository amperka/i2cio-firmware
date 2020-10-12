/**
  ******************************************************************************
  * File Name          : encoder.c
  * Description        : This file provides code for the encoder logic.
  ******************************************************************************
**/

#include "encoder.h"

typedef struct
{
  bool enabled;
  uint8_t pinA;
  uint8_t pinB;
  int8_t unreadedValue;
  uint8_t stateA; //
  uint8_t stateB; // эти переменные используются в качестве 4 битного буфера предыдущих состояний
} Encoder_Type;

static Encoder_Type encoders[MAX_ENCODER_COUNT];

static inline bool pinState(uint8_t pin){
	return digitalRead(pin);
}

void initEncoders()
{
	for (uint8_t i = 0; i < MAX_ENCODER_COUNT; ++i)
	{
		encoders[i].enabled = false;
	}
}

void setEncoderPins(uint8_t encoder, uint8_t pinA, uint8_t pinB)
{
	if (encoder < MAX_ENCODER_COUNT){
		if ((pinA < ADC_COUNT) && (pinB < ADC_COUNT)){
			encoders[encoder].enabled = true;
			encoders[encoder].pinA = pinA;
			encoders[encoder].pinB = pinB;
			encoders[encoder].stateA = 0;
			encoders[encoder].stateB = 0;
			encoders[encoder].unreadedValue = 0;
		}
	}
}

int8_t getValueEncoder(uint8_t encoder){
	int8_t result = 0;
	if (encoder < MAX_ENCODER_COUNT){
		if (encoders[encoder].enabled){
			result = encoders[encoder].unreadedValue;
			encoders[encoder].unreadedValue = 0;
		}
	}
	return result;
}

static inline bool detect(bool x1, bool x2, bool y1, bool y2) 
{
	return (x2 ^ y1) & ~(x1 ^ y2);
}

#define CHANGED(STATE) 		(((encoders[i].STATE & 1) != (int)STATE))
#define ADD_STATE(STATE)	encoders[i].STATE = ((encoders[i].STATE<< 1) | (int)STATE) & 0xf

void encoderCapture() {
	for (uint8_t i = 0; i < MAX_ENCODER_COUNT; ++i){
		bool stateA = pinState(encoders[i].pinA);
		bool stateB = pinState(encoders[i].pinB);
		if (CHANGED(stateA) || CHANGED(stateB)) {
			ADD_STATE(stateA);
			ADD_STATE(stateB);
			if(encoders[i].stateA == 0b00001001 && encoders[i].stateB == 0b00001100)
				encoders[i].unreadedValue++;
			if(encoders[i].stateA == 0b00001100 && encoders[i].stateB == 0b00001001)
				encoders[i].unreadedValue--;
		}
	}
}
