/*
 * Encoder.c
 *
 * Created: 29.06.2023 16:20:10
 *  Author: Marius
 */ 

#include "Encoder.h"
#include "Position.h"
#include "robotControl.h"
#include "path.h"
#include "OwnLaby.h"

#include <avr/io.h>
#include <math.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <io/uart/uart.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <tools/labyrinth/labyrinth.h>

static uint8_t pinbAlt;

static int16_t counterR;
static int16_t counterL;
static int16_t stopCounter = -10;


void calcStopCounter_Turn(){
	float dtheta;
	if ((getState() == TURN_LEFT) || (getState() == TURN_RIGHT)){
		dtheta = M_PI_2;
	}
	if (getState() == TURN_AROUND){
		dtheta = M_PI;
	}
	if (getState() == TURN_ADJUST){
		dtheta = position_getExpectedPose()->theta;
		if (dtheta > 2.0f * M_PI_2)
			dtheta -= 2.0f * M_PI_2;
		
		float dthetaWanted = ownLaby_getRobotPose()->theta  + M_PI_4;
		if (dthetaWanted > 2.0f * M_PI_2)
			dthetaWanted -= 2.0f * M_PI_2;
		
		dtheta -= dthetaWanted;
		if (dtheta < 0.0f)
			dtheta = -1.0f * dtheta;
	}
	stopCounter	= (int16_t) ((dtheta * value_robotParams.axleWidth / value_robotParams.distPerTick));
}

void calcStopCounter_Drive(){
	float adjustDistance = 0.0f;
	if (getState() == DRIVE_FORWARD){
		adjustDistance = LABY_CELLSIZE;
	}
	else {
		if ((ownLaby_getPose()->cardinalDirection == DIRECTION_NORTH) || (ownLaby_getPose()->cardinalDirection == DIRECTION_SOUTH))
			adjustDistance = ownLaby_getRobotPose()->y - position_getExpectedPose()->y;
		if ((ownLaby_getPose()->cardinalDirection == DIRECTION_EAST) || (ownLaby_getPose()->cardinalDirection == DIRECTION_WEST))
			adjustDistance = ownLaby_getRobotPose()->x - position_getExpectedPose()->x;
			
		if (adjustDistance < 0.0f)
			adjustDistance = adjustDistance * -1.0f;
	}
	stopCounter = (int16_t) ((adjustDistance / value_robotParams.distPerTick) * 2.25f);
}

int16_t encoder_getStopCounter(){
	return stopCounter;
}


void encoder_init() {
	DDRL &= ~((1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3));
	PCICR |=  (1<<PCIE0);
	PCMSK0 |= ((1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2) | (1<<PCINT3));
	pinbAlt = PINB & ((1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2) | (1<<PCINT3));
	//DDRL &= ~((1<<DDL0) | (1<<DDL1) | (1<<DDL2) (1<<DDL3));
	//PORTL |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3); 
	// (Nur für normale INT) EICRA &= ~((0<<ISC01) | (1<<ISC00) | (0<<ISC11) (1<<ISC10) | (0<<ISC21) | (1<<ISC20) | (0<<ISC31) | (1<<ISC30));
}

int16_t encoder_getCounterL(){
	return counterL;
}

int16_t encoder_getCounterR(){
	return counterR;
}

void encoder_getCountersAndReset(int16_t* l, int16_t* r) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		*l = counterL;
		*r = counterR;
		counterL = 0;
		counterR = 0;
	}
}

ISR(PCINT0_vect){
	uint8_t pinb = PINB & ((1<<PCINT0) | (1<<PCINT1) | (1<<PCINT2) | (1<<PCINT3));
	
	//Encoder Rechts
	
	if( ((pinb & (1<<PCINT0)) ^ (pinbAlt & (1<<PCINT0))) != 0) { // Flankenwechsel auf Kanal A
		if ((pinb & (1<<PCINT0)) != 0) {	// Flankenwechsel von LOW auf HIGH
			if ((pinb & (1<<PCINT1)) != 0)		// B ist HIGH
				counterR--;
			else								// B ist LOW
				counterR++;
		} 
		else {							// Flankenwechsel von HIGH auf LOW
			if ((pinb & (1<<PCINT1)) != 0)		// B ist HIGH
				counterR++;
			else								// B ist LOW
				counterR--;
		}
	}
	
	if( ((pinb & (1<<PCINT1)) ^ (pinbAlt & (1<<PCINT1))) != 0) { // Flankenwechsel auf Kanal B
		if ((pinb & (1<<PCINT1)) != 0) {	// Flankenwechsel von LOW auf HIGH
			if ((pinb & (1<<PCINT0)) != 0)		// A ist HIGH
				counterR++;
			else								// A ist LOW
				counterR--;
		} 
		else {							// Flankenwechsel von HIGH auf LOW
			if ((pinb & (1<<PCINT0)) != 0)		// A ist HIGH
				counterR--;
			else								// A ist LOW
				counterR++;
		}
	}
	
	//Encoder Links
	
	if( ((pinb & (1<<PCINT2)) ^ (pinbAlt & (1<<PCINT2))) != 0) { // Flankenwechsel auf Kanal A
		if ((pinb & (1<<PCINT2)) != 0) {	// Flankenwechsel von LOW auf HIGH
			if ((pinb & (1<<PCINT3)) != 0)		// B ist HIGH
				counterL++;
			else								// B ist LOW
				counterL--;
		} 
		else {								// Flankenwechsel von HIGH auf LOW
			if ((pinb & (1<<PCINT3)) != 0)		// B ist HIGH
				counterL--;
			else								// B ist LOW
				counterL++;
		}
	}

	if( ((pinb & (1<<PCINT3)) ^ (pinbAlt & (1<<PCINT3))) != 0) { // Flankenwechsel auf Kanal B
		if ((pinb & (1<<PCINT3)) != 0) {	// Flankenwechsel von LOW auf HIGH
			if ((pinb & (1<<PCINT2)) != 0)		// A ist HIGH
				counterL--;
			else								// A ist LOW
				counterL++;
		} 
		else {								// Flankenwechsel von HIGH auf LOW
			if ((pinb & (1<<PCINT2)) != 0)		// A ist HIGH
				counterL++;
			else								// A ist LOW
				counterL--;
		}
	}
	
	if ((getState() == DRIVE_FORWARD) /* || (getState() == DRIVE_ADJUST)*/){
		if (stopCounter == -10) {
			calcStopCounter_Drive();
		}
		
		
		if( ((pinb & (1<<PCINT0)) ^ (pinbAlt & (1<<PCINT0))) != 0) { // Flankenwechsel auf Kanal A
			if ((pinb & (1<<PCINT0)) != 0) {	// Flankenwechsel von LOW auf HIGH
				if ((pinb & (1<<PCINT1)) == 0)		// B ist HIGH
				stopCounter--;
			}
			else {							// Flankenwechsel von HIGH auf LOW
				if ((pinb & (1<<PCINT1)) != 0)		// B ist HIGH
				stopCounter--;
			}
		}
		
		if( ((pinb & (1<<PCINT1)) ^ (pinbAlt & (1<<PCINT1))) != 0) { // Flankenwechsel auf Kanal B
			if ((pinb & (1<<PCINT1)) != 0) {	// Flankenwechsel von LOW auf HIGH
				if ((pinb & (1<<PCINT0)) != 0)		// A ist HIGH
				stopCounter--;
			}
			else {							// Flankenwechsel von HIGH auf LOW
				if ((pinb & (1<<PCINT0)) == 0)		// A ist HIGH
				stopCounter--;
			}
		}
		
		//Encoder Links
		
		if( ((pinb & (1<<PCINT2)) ^ (pinbAlt & (1<<PCINT2))) != 0) { // Flankenwechsel auf Kanal A
			if ((pinb & (1<<PCINT2)) != 0) {	// Flankenwechsel von LOW auf HIGH
				if ((pinb & (1<<PCINT3)) != 0)		// B ist HIGH
				stopCounter--;
			}
			else {								// Flankenwechsel von HIGH auf LOW
				if ((pinb & (1<<PCINT3)) == 0)		// B ist HIGH
				stopCounter--;
			}
		}

		if( ((pinb & (1<<PCINT3)) ^ (pinbAlt & (1<<PCINT3))) != 0) { // Flankenwechsel auf Kanal B
			if ((pinb & (1<<PCINT3)) != 0) {	// Flankenwechsel von LOW auf HIGH
				if ((pinb & (1<<PCINT2)) == 0)		// A ist HIGH
				stopCounter--;
			}
			else {								// Flankenwechsel von HIGH auf LOW
				if ((pinb & (1<<PCINT2)) != 0)		// A ist HIGH
				stopCounter--;
			}
		}
		
		
		if (stopCounter <= 0) {
			stopCounter = -10;
			/*
			if (getState() == DRIVE_ADJUST)
				setState(STOP);
			else
			*/
				//setState(TURN_ADJUST);
				setState(STOP);
		}
	}
	
	if ((getState() == TURN_LEFT) || (getState() == TURN_RIGHT) || (getState() == TURN_AROUND) || (getState() == TURN_ADJUST)){
		if (stopCounter == -10) {
			calcStopCounter_Turn();
		}
		
		
		if( ((pinb & (1<<PCINT0)) ^ (pinbAlt & (1<<PCINT0))) != 0)
			stopCounter--;
		if( ((pinb & (1<<PCINT1)) ^ (pinbAlt & (1<<PCINT1))) != 0)
			stopCounter--;
		if( ((pinb & (1<<PCINT2)) ^ (pinbAlt & (1<<PCINT2))) != 0)
			stopCounter--;
		if( ((pinb & (1<<PCINT3)) ^ (pinbAlt & (1<<PCINT3))) != 0)
			stopCounter--;
		
		if (stopCounter <= 0) {
			stopCounter = -10;
			
			if (getState() == TURN_ADJUST)
				//setState(DRIVE_ADJUST);
				setState(STOP);
			else
				//setState(TURN_ADJUST);
				setState(STOP);
		}
	}
	
	pinbAlt = pinb;
}



