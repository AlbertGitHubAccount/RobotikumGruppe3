/*
 * Encoder.h
 *
 * Created: 29.06.2023 18:27:14
 *  Author: Marius
 */ 


#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void calcStopCounter_Turn();
void calcStopCounter_Drive();
void encoder_init();

void encoder_getCountersAndReset(int16_t* l, int16_t* r);

int16_t encoder_getCounterR();
int16_t encoder_getCounterL();
int16_t encoder_getStopCounter();


#endif /* ENCODER_H_ */