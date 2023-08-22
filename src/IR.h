/*
 * IR.h
 *
 * Created: 21.07.2023 13:17:56
 *  Author: Marius
 */ 


#ifndef IR_H_
#define IR_H_

#include <stdint.h>

typedef struct __attribute__((__packed__)) {
	uint16_t frontIR;
	uint16_t rightIR;
	uint16_t leftIR;
} IR_value_t;

const IR_value_t* IR_getIR_value();
void IR_setIR_value();

#endif /* IR_H_ */