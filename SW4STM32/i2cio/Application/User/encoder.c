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

void encoderCapture() {
	for (uint8_t i = 0; i < MAX_ENCODER_COUNT; ++i){
		if (encoders[i].enabled){
			bool stateA = pinState(encoders[i].pinA);
			bool stateB = pinState(encoders[i].pinB);
			if(((encoders[i].stateA & 1) != (int)stateA) || ((encoders[i].stateB & 1) != (int)stateB)) { // если состояние изменилось
				encoders[i].stateA = ((encoders[i].stateA << 1) | (int)stateA) & 0xf; // записать новое состояние в буфер состояний
				encoders[i].stateB = ((encoders[i].stateB << 1) | (int)stateB) & 0xf; //
				if(encoders[i].stateA == 6 && encoders[i].stateB == 3) // цифры 3 и 6 это битовые шаблоны соответствующих состояний
					encoders[i].unreadedValue++;
				if(encoders[i].stateA == 3 && encoders[i].stateB == 6)
					encoders[i].unreadedValue--;
			}
		}
	}
}
