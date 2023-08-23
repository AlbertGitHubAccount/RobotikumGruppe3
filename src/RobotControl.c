/*
 * RobotControl.c
 *
 * Created: 10.06.2023 15:36:13
 * Author : daniel
 */ 

#include <stdio.h>
#include <io/uart/uart.h>
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <tools/powerSaver.h>
#include <io/led/led.h>
#include <motor/motor.h>
#include <communication/packetTypes.h>
#include <tools/labyrinth/labyrinth.h>
#include <math.h>
                 
#include "robotControl.h"
#include "IR.h"
#include "OwnLaby.h"
#include "Position.h"


static state_t state = IDLE;
static timeTask_time_t startTime;		//timeTask_getTimestamp();

void setState(const state_t newState) {
	state = newState;
}

state_t getState() {
	return state;
}

timeTask_time_t getStartTime(){
	return startTime;
}

void driveForward() {
	Motor_setPWM(3000, 3010);
}

void driveBackward() {
	Motor_setPWM(-3000, -3010);
}

void driveAdjust() {
	if (ownLaby_getPose()->cardinalDirection == DIRECTION_NORTH){
		if (ownLaby_getRobotPose()->y < position_getExpectedPose()->y)
			driveForward();
		else
			driveBackward();
	}
	
	if (ownLaby_getPose()->cardinalDirection == DIRECTION_EAST){
		if (ownLaby_getRobotPose()->x > position_getExpectedPose()->x)
			driveForward();
		else
			driveBackward();
	}
	
	if (ownLaby_getPose()->cardinalDirection == DIRECTION_SOUTH){
		if (ownLaby_getRobotPose()->y > position_getExpectedPose()->y)
			driveForward();
		else
			driveBackward();
	}
	
	if (ownLaby_getPose()->cardinalDirection == DIRECTION_WEST){
		if (ownLaby_getRobotPose()->x < position_getExpectedPose()->x)
			driveForward();
		else
			driveBackward();
	}
}

void turnLeft() {
	Motor_setPWM(-3000, 3000);
}

void turnRight() {
	Motor_setPWM(3000, -3000);
}

void turnAdjust(){
	float dtheta = position_getExpectedPose()->theta + M_PI_4;
	if (dtheta > 2.0f * M_PI_2)
		dtheta -= 2.0f * M_PI_2;
	
	float dthetaWanted = ownLaby_getRobotPose()->theta  + M_PI_4;
	if (dthetaWanted > 2.0f * M_PI_2)
		dthetaWanted -= 2.0f * M_PI_2;
		
	dtheta -= dthetaWanted;
	if (dtheta < 0.0f)
		turnLeft();
	if (dtheta > 0.0f)
		turnRight();
}

void wait_90() {
	timeTask_time_t now;
	timeTask_getTimestamp(&now);
	if (timeTask_getDuration(&startTime, &now) > 1050000UL)
	setState(STOP);
}


// Funktion zur Steuerung des Roboters
void stateMachine() {
	switch (state) {
		case IDLE:
			break;
		case DRIVE_FORWARD:
			driveForward();
			break;
		/*
		case DRIVE_ADJUST:
			driveAdjust();
			break;
		*/
		case WAIT_90:
			wait_90();
			break;
		case TURN_LEFT:
			turnLeft();
			break;
		case TURN_RIGHT:
			turnRight();
			break;
		case TURN_AROUND:
			turnLeft();
			break;
		case TURN_ADJUST:
			turnRight();
			break;
		case STOP:
			Motor_stopAll();
			setState(IDLE);
			break;
	}
}