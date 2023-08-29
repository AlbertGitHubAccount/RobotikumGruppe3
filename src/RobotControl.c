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
	Motor_setPWM(-1500, -1505);
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
		
	if (dtheta < dthetaWanted)
		turnLeft();
	if (dtheta > dthetaWanted)
		turnRight();
}

void resting() {
	Motor_stopAll();
	timeTask_time_t now;
	timeTask_getTimestamp(&now);
	if (timeTask_getDuration(&startTime, &now) > 3000000UL)
		setState(IDLE);
}

void checkSensors(){
	robot_isWall(0);
	robot_isWall(1);
	robot_isWall(2);
}

// Funktion zur Steuerung des Roboters
void stateMachine() {
	switch (state) {
		case IDLE:
			break;
		case EXPLORE:
			ownLaby_explore();
			break;
		case CHECK_SENSORS:
			checkSensors();
			break;
		case DRIVE_FORWARD:
			driveForward();
			break;
		case DRIVE_BACKWARD:
			driveBackward();
			break;
		case DRIVE_ADJUST:
			driveAdjust();
			break;
		case RESTING:
			resting();
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
			turnAdjust();
			break;
		case STOP:
			Motor_stopAll();
			setState(IDLE);
			break;
	}
}