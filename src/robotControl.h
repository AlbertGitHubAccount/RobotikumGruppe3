
/*
 * robotControl.h
 *
 * Created: 15.06.2023
 * Author : Marius
 */ 

#ifndef ROBOTCONTROL_H_
#define ROBOTCONTROL_H_

#include <stdint.h>
#include <tools/timeTask/timeTask.h>


typedef enum state {
	IDLE,			//0
	EXPLORE,		//1
	CHECK_SENSORS,	//2
	DRIVE_FORWARD,	//3
	DRIVE_BACKWARD,	//4
	DRIVE_ADJUST,	//5
	RESTING,		//6
	TURN_LEFT,		//7
	TURN_RIGHT,		//8
	TURN_AROUND,	//9
	TURN_ADJUST,	//10
	DRIVE_EXIT,		//11
	STOP			//12
} state_t;

void stateMachine();

void setState(const state_t newState);
	
state_t getState();

timeTask_time_t getStartTime();

#endif /* ROBOTCONTROL_H_ */