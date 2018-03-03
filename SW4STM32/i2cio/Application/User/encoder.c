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
  bool lastStateA;
  bool lastStateB;
  int8_t unreadedValue;
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
			encoders[encoder].lastStateA = pinState(pinA);
			encoders[encoder].lastStateB = pinState(pinB);
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

void encoderCapture(){
	for (uint8_t i = 0; i < MAX_ENCODER_COUNT; ++i){
		if (encoders[i].enabled){
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
}