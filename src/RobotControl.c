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

#include "globalVariables.h"                 
#include "robotControl.h"
#include "IR.h"
#include "OwnLaby.h"
#include "Position.h"
#include "helperFunctions.h"

enum ChosenDirectionValue {
	CHOSE_LEFT,
	CHOSE_RIGHT,
	CHOSE_UP,
	CHOSE_DOWN
};

enum ChosenDirectionValue chosenDirectionValue = CHOSE_LEFT;

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
	//Fahre
	Motor_setPWM(3000, 3010);
	
	//aktualisiere Position
	currentPosition = targetTile;
	
	//erhöhe visit count
	visitedArray[targetTile.x][targetTile.y]++;
	
	//reset flags
	canGoEast = false;
	canGoNorth = false;
	canGoSouth = false;
	canGoWest = false;
	
	//starte Zyklus erneut
	setState(CHECK_SENSORS);
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
	
	//muss nach einer Drehung in DriveForward
}

void turnRight() {
	Motor_setPWM(3000, -3000);
	
	//muss nach einer Drehung in DriveForward
}

void turnAround(){
	
	//muss nach einer Drehung in DriveForward	
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

void wait_90() {
	timeTask_time_t now;
	timeTask_getTimestamp(&now);
	if (timeTask_getDuration(&startTime, &now) > 1050000UL)
	setState(STOP);
}

void checkSensors(){
	//prüfe Wände
	robot_isWall(0);
	robot_isWall(1);
	robot_isWall(2);
	robot_isWall(3);
	
	//Setze Wände in HWPControllSystem
	
	//Wechsle State
	setState(CHOOSE_DIRECTION);
}

void chooseDirection(){
	// Wände checken, Roboter schaut nach Norden
	if(*currentDirectionPtr == NORTH){
		if (robot_isWall(LEFT) == false) canGoWest = true;
		if (robot_isWall(RIGHT) == false) canGoEast = true;
		if (robot_isWall(FORWARD) == false) canGoNorth = true;
		if (true)	canGoSouth = true;
	}
	if(*currentDirectionPtr == WEST){
		if (robot_isWall(LEFT) == false) canGoSouth = true;
		if (robot_isWall(RIGHT) == false) canGoNorth = true;
		if (robot_isWall(FORWARD) == false) canGoWest = true;
		if (true)	canGoEast = true;
	}
	if(*currentDirectionPtr == SOUTH){
		if (robot_isWall(LEFT) == false) canGoEast = true;
		if (robot_isWall(RIGHT) == false) canGoWest = true;
		if (robot_isWall(FORWARD) == false) canGoSouth = true;
		if (true)	canGoNorth = true;
	}
	if(*currentDirectionPtr == EAST){
		if (robot_isWall(LEFT) == false) canGoNorth = true;
		if (robot_isWall(RIGHT) == false) canGoSouth = true;
		if (robot_isWall(FORWARD) == false) canGoEast = true;
		if (true)	canGoWest = true;
	}


	
	//aus aktueller Position tiles berechnen die der Roboter besuchen kann
	if(*currentDirectionPtr == NORTH){
	
		//heads north and can go north
		if (canGoNorth) {
			// declare target tile in first case, because there must be one
			targetTile.x = currentPosition.x;
			targetTile.y = currentPosition.y + 1;

			// store visits of northern tile
			int visited_north = visitedArray[currentPosition.x][currentPosition.y + 1];

			// compare with visits of target tile
			if (visited_north < visitedArray[targetTile.x][targetTile.y]) {
				targetTile.x = currentPosition.x;
				targetTile.y = currentPosition.y + 1;
			
				// safe chosen direction for later
				chosenDirectionValue = CHOSE_UP;
			
			}

		}

		if (canGoWest) {
			// if that's the first tile you can visit you have to "declare" target tile
			if (!canGoNorth) {
				targetTile.x = currentPosition.x - 1;
				targetTile.y = currentPosition.y;
			}

			// store visits of western tile
			int visited_west = visitedArray[currentPosition.x - 1][currentPosition.y];

			// compare with visits of target tile
			if (visited_west < visitedArray[targetTile.x][targetTile.y]) {
				targetTile.x = currentPosition.x - 1;
				targetTile.y = currentPosition.y;
			
				// safe chosen direction for later
				chosenDirectionValue = CHOSE_LEFT;
			}
		}

		if (canGoSouth) {
			// if that's the first tile you can visit you have to "declare" target tile
			if (!canGoNorth && !canGoWest) {
				targetTile.x = currentPosition.x;
				targetTile.y = currentPosition.y - 1;
			}

			// store visits of southern tile
			int visited_south = visitedArray[currentPosition.x][currentPosition.y - 1];

			// compare with visits of target tile
			if (visited_south < visitedArray[targetTile.x][targetTile.y]) {
				targetTile.x = currentPosition.x;
				targetTile.y = currentPosition.y - 1;
			
				// safe chosen direction for later
				chosenDirectionValue = CHOSE_DOWN;
			}
		}

		if (canGoEast) {
			// if that's the first tile you can visit you have to "declare" target tile
			if (!canGoNorth && !canGoWest && !canGoSouth) {
				targetTile.x = currentPosition.x + 1;
				targetTile.y = currentPosition.y;
			}

			// store visits of eastern tile
			int visited_east = visitedArray[currentPosition.x + 1][currentPosition.y];

			// compare with visits of target tile
			if (visited_east < visitedArray[targetTile.x][targetTile.y]) {
				targetTile.x = currentPosition.x + 1;
				targetTile.y = currentPosition.y;
			
			// safe chosen direction for later
			chosenDirectionValue = CHOOSE_RIGHT;
			
			}
	}

		
	}
	
	if(*currentDirectionPtr == WEST){
			
	}
		
	if(*currentDirectionPtr == SOUTH){
		
	}
	
	if(*currentDirectionPtr == EAST){
			
	}


	//aus den möglichen Richtungen die auswählen, die am seltensten besucht wurde
	if(*currentDirectionPtr == NORTH){
		if(chosenDirectionValue == CHOSE_LEFT) setState(CHOOSE_LEFT);
		if(chosenDirectionValue == CHOSE_UP) setState(CHOOSE_FORWARD);
		if(chosenDirectionValue == CHOSE_DOWN) setState(CHOOSE_BACKWARD);
		if(chosenDirectionValue == CHOSE_RIGHT) setState(CHOOSE_RIGHT);

	}
	if(*currentDirectionPtr == WEST){
		if(chosenDirectionValue == CHOSE_LEFT) setState(CHOOSE_FORWARD);
		if(chosenDirectionValue == CHOSE_UP) setState(CHOOSE_RIGHT);
		if(chosenDirectionValue == CHOSE_DOWN) setState(CHOOSE_LEFT);
		if(chosenDirectionValue == CHOSE_RIGHT) setState(CHOOSE_BACKWARD);	
	}
		
	if(*currentDirectionPtr == SOUTH){
		if(chosenDirectionValue == CHOSE_LEFT) setState(CHOOSE_RIGHT);
		if(chosenDirectionValue == CHOSE_UP) setState(CHOOSE_BACKWARD);
		if(chosenDirectionValue == CHOSE_DOWN) setState(CHOOSE_FORWARD);
		if(chosenDirectionValue == CHOSE_RIGHT) setState(CHOOSE_LEFT);	
	}
		
	if(*currentDirectionPtr == EAST){
		if(chosenDirectionValue == CHOSE_LEFT) setState(CHOOSE_BACKWARD);
		if(chosenDirectionValue == CHOSE_UP) setState(CHOOSE_LEFT);
		if(chosenDirectionValue == CHOSE_DOWN) setState(CHOOSE_RIGHT);
		if(chosenDirectionValue == CHOSE_RIGHT) setState(CHOOSE_FORWARD);	
	}
}

void leftState(){	
	//aktualisiere Himmelsrichtung
	turnLeftCardinalChange(*currentDirectionPtr);
	
	//gehe in den nächsten Status
	setState(TURN_LEFT);
}

void rightState(){
	//drehe nach rechts
	turnRight();
	
	//aktualisiere Himmelsrichtung
	turnRightCardinalChange(*currentDirectionPtr);
	
	//gehe in den nächsten Status
	setState(DRIVE_FORWARD);
}

void backwardState(){
	//drehe dich um
	turnAround();
	
	//aktualisiere Himmelsrichtung
	turnBackwardCardinalChange(*currentDirectionPtr);
	
	//gehe in den nächsten Modus
	setState(DRIVE_FORWARD);
}

void forwardState(){
	setState(DRIVE_FORWARD);
}


// Funktion zur Steuerung des Roboters
void stateMachine() {
	switch (state) {
		case IDLE:
			break;
		case CHECK_SENSORS:
			checkSensors();
			break;
		case CHOOSE_DIRECTION:
			chooseDirection();
			break;
		case CHOOSE_LEFT:
			leftState();
			break;
		case CHOOSE_RIGHT:
			rightState();
			break;
		case CHOOSE_BACKWARD:
			backwardState();
			break;
		case CHOOSE_FORWARD:
			forwardState();
			break;
		case DRIVE_BACKWARD:
			driveBackward();
			break;
		case DRIVE_FORWARD:
			driveForward();
		case DRIVE_ADJUST:
			driveAdjust();
			break;
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
			turnAdjust();
			break;
		case STOP:
			Motor_stopAll();
			setState(IDLE);
			break;
	}
}