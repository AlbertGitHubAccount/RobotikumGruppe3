/*
 * IR.c
 *
 * Created: 29.06.2023 16:51:31
 *  Author: Daniel
 */ 

#include "IR.h"

#include <io/adc/adc.h>
#include <math.h>

static IR_value_t IR_value;

const IR_value_t* IR_getIR_value(){
	return &IR_value;
}

void IR_setIR_value(){
	//wenn value öfters als alle 6ms abgefragt wird, if-clausel über upToDateFlags
	IR_value.frontIR	= roundf(-1.72738548530529500979f + (38715.76849258523725438863f / (float)ADC_getFilteredValue(0)));
	IR_value.rightIR	= roundf(-1.72738548530529500979f + (38715.76849258523725438863f / (float)ADC_getFilteredValue(1)));
	IR_value.leftIR		= roundf(-1.72738548530529500979f + (38715.76849258523725438863f / (float)ADC_getFilteredValue(2)));
}
