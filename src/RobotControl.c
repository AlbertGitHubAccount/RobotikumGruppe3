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
#include <stdbool.h>
                 
#include "robotControl.h"
#include "IR.h"
#include "OwnLaby.h"
#include "Position.h"
#include "Bumper.h"


static state_t state = IDLE;
static timeTask_time_t startTime;		//timeTask_getTimestamp();

void setState(const state_t newState) {
	communication_log(LEVEL_INFO, "S%"PRIu8, newState);
	state = newState;
}

state_t getState() {
	return state;
}

//timeTask_time_t getStartTime(){
//	return startTime;
//}

void driveForward() {
	ownLaby_setPose();
	ownLaby_setRobotPose();
	Motor_setPWM(2500, 2570);
}

void driveBackward() {
	Motor_setPWM(-1500, -1600);
}

void driveAdjustCalc(){
	ownLaby_setPose();
	ownLaby_setRobotPose();
	if (ownLaby_getPose()->cardinalDirection == DIRECTION_NORTH){
		if (ownLaby_getRobotPose()->y > position_getExpectedPose()->y)
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
		if (ownLaby_getRobotPose()->y < position_getExpectedPose()->y)
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
	
	setState(DRIVE_ADJUST);	
}

void turnLeft_preState(){
	ownLaby_setPose();
	ownLaby_setRobotPose();
	Motor_setPWM(-3000, 3000);
	setState(TURN_LEFT);
}

void turnLeft() {

}

void turnRight_preState(){
	ownLaby_setPose();
	ownLaby_setRobotPose();
	Motor_setPWM(3000, -3000);
	setState(TURN_RIGHT);
}

void turnRight() {

}




void calcAdjust(){
	ownLaby_setPose();
	ownLaby_setRobotPose();

	float dtheta = position_getExpectedPose()->theta - ownLaby_getRobotPose()->theta;
	if (dtheta >  M_PI_4)
	dtheta -= 2.0f * M_PI;
	
	if (dtheta < -M_PI_4)
	dtheta += 2.0f * M_PI;
	
	if (dtheta <= 0.0f)
	turnLeft();
	if (dtheta  > 0.0f)
	turnRight();

	setState(TURN_ADJUST);
}

void resting() {
	Motor_stopAll();
//	timeTask_time_t now;
//	timeTask_getTimestamp(&now);
//	if (timeTask_getDuration(&startTime, &now) > 3000000UL)
		setState(IDLE);
}

/*
void drive_exit() {
	driveForward();
	timeTask_time_t now;
	timeTask_getTimestamp(&now);
	if (timeTask_getDuration(&startTime, &now) > 3000000UL){
		Motor_stopAll();
	}
}
*/

void out_of_bounds_stop() {
//	timeTask_time_t now;
//	timeTask_getTimestamp(&now);
//	if (timeTask_getDuration(&startTime, &now) > 3000000UL){
		Motor_stopAll();
//	}
}

void checkSensors(){
	robot_isWall(LEFT);
	robot_isWall(FORWARD);
	robot_isWall(RIGHT);
	setState(IDLE);
}

void wait_init() {
	timeTask_getTimestamp(&startTime);
	setState(WAIT);
}

void wait() {
	timeTask_time_t now;
	timeTask_getTimestamp(&now);
	if (timeTask_getDuration(&startTime, &now) > 200000UL) {
		setState(EXPLORE);
	}
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
		case DRIVE_ADJUST_CALC:
			driveAdjustCalc();
			break;
		case DRIVE_ADJUST:
			break;
		case RESTING:
			resting();
			break;
		case TURN_LEFT_PRESTATE:
			turnLeft_preState();
			break;
		case TURN_LEFT:
			break;
		case TURN_RIGHT_PRESTATE:
			turnRight_preState();
			break;
		case TURN_RIGHT:
			turnRight();
			break;
		case TURN_AROUND:
			turnLeft();
			break;
		case CALC_ADJUST:
			calcAdjust();
			break;
		case TURN_ADJUST:
			break;
		case WAIT_INIT:
			wait_init();
			break;
		case WAIT:
			wait();
			break;
		/*
		case DRIVE_EXIT:
			drive_exit();
			break;
		*/
		case OUT_OF_BOUNDS:
			out_of_bounds_stop();
			break;
		case STOP:
			Motor_stopAll();
			setState(IDLE);
			break;
	}
}