/*
 * Bumper.c
 *
 * Created: 23.06.2023 16:15:07
 *  Author: Daniel
 */ 
#include "Bumper.h"
#include "robotControl.h"
#include "OwnLaby.h"

#include <avr/io.h>					// AVR IO ports
#include <stdbool.h>

static bitset8_t taster;
static uint16_t contacts = 0;
bool driveAdjustYes = false;

//static uint8_t pinlAlt;

void bumper_init() {
	DDRL &= ~((1<<DDL0) | (1<<DDL1) | (1<<DDL2));
	PORTL |= (1<<PL0) | (1<<PL1) | (1<<PL2);
	taster.value = PINL & ((1<<PL0) | (1<<PL1) | (1<<PL2));
	//pinlAlt = taster.value;
}

bitset8_t bumper_getBumpers() {
	return taster;
}

uint8_t bumper_getContacts() {
	return contacts;
}

void bumper_checkCollision() {
	uint8_t pinl = PINL & ((1<<PL0) | (1<<PL1) | (1<<PL2));
	
	if(taster.value != pinl){
		if((pinl & (1<<PL0)) == 0){//rechts
			contacts++;
			if (getState() == DRIVE_FORWARD)
				robot_isWall(RIGHT);
		}
		if((pinl & (1<<PL1)) == 0){//links
			contacts++;
			if (getState() == DRIVE_FORWARD)
				robot_isWall(LEFT);
		}
		if((pinl & (1<<PL2)) == 0){//vorne
			contacts++;
			if ((getState() == DRIVE_FORWARD) || (getState() == DRIVE_ADJUST)){
				robot_isWall(FORWARD);
				driveAdjustYes = true;
			}
		}
	
		if (getState() != RESTING){
			setState(RESTING);
		}
		
		//Adjusten des Roboters nach Wand-kontakt
		if (getState() != TURN_ADJUST){
			if(driveAdjustYes == false)
				setState(TURN_ADJUST);
			if(driveAdjustYes == true)
				setState(DRIVE_ADJUST);
		}
	}
		
	taster.value = pinl;
}
