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
  uint8_t stateB; // these are 4-bit buffers of the prev state
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

void encoderCapture() {
	for (uint8_t i = 0; i < MAX_ENCODER_COUNT; ++i){
		bool stateA = pinState(encoders[i].pinA);
		bool stateAChanged = (encoders[i].stateA & 1) != (int)stateA;
		bool stateB = pinState(encoders[i].pinB);
		bool stateBChanged = (encoders[i].stateB & 1) != (int)stateB;

		if (stateAChanged || stateBChanged) {
			encoders[i].stateA = ((encoders[i].stateA << 1) | (int)stateA) & 0xf;
			encoders[i].stateB = ((encoders[i].stateB << 1) | (int)stateB) & 0xf;

			if(encoders[i].stateA == 0b00001001 && encoders[i].stateB == 0b00001100)
				encoders[i].unreadedValue++;
			if(encoders[i].stateA == 0b00001100 && encoders[i].stateB == 0b00001001)
				encoders[i].unreadedValue--;
		}
	}
}
