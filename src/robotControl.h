
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
	DRIVE_ADJUST_CALC, //5
	DRIVE_ADJUST,	//6
	RESTING,		//7
	TURN_LEFT_PRESTATE,
	TURN_LEFT,		//8
	TURN_RIGHT_PRESTATE,
	TURN_RIGHT,		//9
	TURN_AROUND,	//10
	CALC_ADJUST,	//11
	TURN_ADJUST,	//12
	WAIT_INIT,
	WAIT,
	//DRIVE_EXIT,	//12
	OUT_OF_BOUNDS,	//13
	STOP			//14
} state_t;

void stateMachine();

void setState(const state_t newState);
	
state_t getState();

timeTask_time_t getStartTime();

#endif /* ROBOTCONTROL_H_ */