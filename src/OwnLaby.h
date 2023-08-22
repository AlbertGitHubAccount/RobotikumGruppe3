/*
 * OwnLaby.c
 *
 * Created: 01.08.2023 18:55:30
 *  Author: Albert
 */ 

#include "OwnLaby.h"
#include "Position.h"
#include "robotControl.h"
#include "IR.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <tools/labyrinth/labyrinth.h>


#define MAX_X 21
#define MAX_Y 21
#define START_X 11
#define START_Y 11

LPose_t labyPose = {3, 3, 0};
Pose_t labyRobotPose = {0.0f, 0.0f, M_PI/2};

int* visit_count[LABYRINTH_ROWS][LABYRINTH_COLS];
Position current_position;
Position target_tile;

/*const char* directionToString(RobotDirection_t direction) {
    switch(direction) {
        case LEFT: return "LEFT";
        case FORWARD: return "FORWARD";
        case RIGHT: return "RIGHT";
        case BACKWARD: return "BACKWARD";
        default: return "UNKNOWN";
    }
}*/

const Position* ownLaby_getCurrentPosition(){
	return &current_position;
}

void ownLaby_setCurrentPosition(const LPose_t* labyPose){
	current_position.x = labyPose->row;
	current_position.y = labyPose->column;
}

int ownLaby_getVisitCount(int row, int column) {
	return *(visit_count[row][column]);
}

void ownLaby_setVisitCount(int row, int col) {
	(*(visit_count[row][col]))++;
}



const Position* ownLaby_getTargetTile(){
	return &target_tile;
}

void ownLaby_setTargetTile(Position current_position, int8_t x_change, int8_t y_change){
	target_tile.x = current_position.x + x_change;
	target_tile.y = current_position.y + y_change;
}

const LPose_t* ownLaby_getPose(){
	return &labyPose; 
}

void ownLaby_setPose(const Pose_t* expextedPose)
{
	labyPose.row	= (uint8_t) floor((expextedPose->x + 125.0f) / LABY_CELLSIZE) + 3.0f;
	labyPose.column = (uint8_t) floor((expextedPose->y + 125.0f) / LABY_CELLSIZE) + 3.0f;
	
	float adjustedTheta = expextedPose->theta + 1.0f * M_PI_4;
	if (adjustedTheta >= 2.0f * M_PI)
		adjustedTheta -= 2.0f * M_PI;
	
	adjustedTheta = -(floor(adjustedTheta/(1.0f * M_PI_2))) + 5.0f;
	if (adjustedTheta >= 4.0f)
		adjustedTheta -= 4.0f;
	
	labyPose.cardinalDirection = (Direction_t) adjustedTheta;
}

const Pose_t* ownLaby_getRobotPose(){
	return &labyRobotPose;
}

void ownLaby_setRobotPose(const LPose_t* labyPose)
{
	labyRobotPose.x = (((float) labyPose->row    - 3.0f) * LABY_CELLSIZE) - 125.0f;
	labyRobotPose.y = (((float) labyPose->column - 3.0f) * LABY_CELLSIZE) - 125.0f;

	if (labyPose->cardinalDirection == 1) //East
		labyRobotPose.theta = 0.0f;
	if (labyPose->cardinalDirection == 0) //North
		labyRobotPose.theta = 1.0f * M_PI_2;
	if (labyPose->cardinalDirection == 3) //West
		labyRobotPose.theta = 1.0f * M_PI;	
	if (labyPose->cardinalDirection == 2) //South
		labyRobotPose.theta = 3.0f * M_PI_2;
}

/*bool robot_canContinue()
{
	assert(!"The method or operation is not implemented.");
	return true;
}*/

void ownLaby_init() {
	memset(visit_count, 0, LABYRINTH_ROWS * LABYRINTH_COLS * sizeof(int));
	
	labyPose = *ownLaby_getPose();
	
	current_position	= *ownLaby_getCurrentPosition();
	ownLaby_setTargetTile(current_position, -3, -3);
	target_tile			= *ownLaby_getTargetTile();
} // explore()

int8_t robot_getExitDirection()
{
	int8_t exitDirection = -1;
	
	if ((ownLaby_getPose()->row == 0) && (ownLaby_getPose()->column == 3)){
		if ((ownLaby_getPose()->cardinalDirection == DIRECTION_NORTH))
			exitDirection = 1;
		
		if ((ownLaby_getPose()->cardinalDirection == DIRECTION_EAST)){
			exitDirection = 0;
		} 
		else{
			exitDirection = 2;
		}
	}
	return exitDirection;
}

void robot_rotate(RobotDirection_t localDirection)
{
	if(localDirection == LEFT)
		setState(TURN_LEFT);
	if(localDirection == RIGHT)
		setState(TURN_RIGHT);
	if(localDirection == BACKWARD)
		setState(TURN_AROUND);
}

bool robot_move(RobotDirection_t moveState)
{
	bool hasMoved = false;
	if (moveState == FORWARD){
		if (robot_canMove(moveState) == true){
			setState(DRIVE_FORWARD);
			hasMoved = true;
		}
	}

	return hasMoved;
}

Direction_t ownLaby_localToCardinal(RobotDirection_t localDirection, Direction_t cardinalDirection)
{
	int8_t newCardinalDirection = ((int8_t)localDirection - 1) + ((int8_t)cardinalDirection);
	
	if (newCardinalDirection >= 4)
		newCardinalDirection -= 4;
	if (newCardinalDirection < 0)
		newCardinalDirection += 4;
	
	return (Direction_t) newCardinalDirection;
}

bool robot_isWall(RobotDirection_t localDirection){
	LPose_t pose = *ownLaby_getPose();
	Direction_t cardinalDirection = ownLaby_localToCardinal(localDirection, pose.cardinalDirection);

	Walls_t walls = labyrinth_getWalls(pose.row, pose.column);
	
	bool isWall = ((walls.walls & (1 << cardinalDirection)) >> cardinalDirection) == WALLSTATE_SET;
	if (! isWall) {
		const IR_value_t* IR_value = IR_getIR_value();
		float IR_specificValue = 50.0f; 
		if (localDirection == FORWARD)
			IR_specificValue = IR_value->frontIR;
		if (localDirection == LEFT)
			IR_specificValue = IR_value->leftIR;
		if (localDirection == RIGHT)
			IR_specificValue = IR_value->rightIR;
			
		if(IR_specificValue < 50.0f){
			isWall = true;
			walls.walls |= 1 << cardinalDirection;
			labyrinth_setWalls(pose.row, pose.column, walls);
		}
	}
	return isWall;
}

LPose_t labyPose;
const LPose_t* ownLaby_getPose();
void ownLaby_setPose();

bool robot_canMove(RobotDirection_t localDirection){
	bool canMove = true;
	if(robot_isWall(localDirection) == true){
		canMove = false;
	}
	return canMove;	
}

void ownLaby_explore(){
	RobotDirection_t leastVisitedDirection = FORWARD;
	bool canMoveLeft	= false;
	bool canMoveRight	= false;
	bool canMoveForward = false;
	char lowestVisitCount = 127;
	int8_t exitDir = robot_getExitDirection();
	
	if (exitDir > -1) {
		robot_rotate((RobotDirection_t)exitDir);
		robot_move(FORWARD);
	}
	else {
		LPose_t pose = *ownLaby_getPose();        

		visit_count[current_position.x][current_position.y]++;
		/*fprintf(stderr, "Current position: (%d, %d, %d)\n", current_position.x, current_position.y, pose.cardinalDirection);
		fprintf(stderr, "Visited (%d)\n", visit_count[current_position.x][current_position.y]);*/

		//Check which directions are available
		if (robot_canMove(LEFT))	{canMoveLeft	= true;}
		if (robot_canMove(FORWARD))	{canMoveForward	= true;}
	    if (robot_canMove(RIGHT))	{canMoveRight	= true;}
	                   
	    //Check direction robot is facing and calculating distances relative to its direction
	    if (pose.cardinalDirection == DIRECTION_NORTH){
			if (canMoveForward == true) {
				lowestVisitCount = *visit_count[current_position.x][current_position.y+1];
				leastVisitedDirection = FORWARD;
				ownLaby_setTargetTile(current_position, 0, 1);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[current_position.x-1][current_position.y]){
					lowestVisitCount = *visit_count[current_position.x-1][current_position.y];
					leastVisitedDirection = LEFT;
					ownLaby_setTargetTile(current_position, -1, 0);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[current_position.x+1][current_position.y]){
					lowestVisitCount = *visit_count[current_position.x+1][current_position.y];
					leastVisitedDirection = RIGHT;
					ownLaby_setTargetTile(current_position, 1, 0);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				ownLaby_setTargetTile(current_position, 0, -1);
			}
		}
	            	
		//Check that happens if Robot is facing Left
		if (pose.cardinalDirection == DIRECTION_WEST){
			if (canMoveForward == true ) {
				lowestVisitCount = *visit_count[current_position.x-1][current_position.y];
				leastVisitedDirection = FORWARD;
				ownLaby_setTargetTile(current_position, -1, 0);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[current_position.x][current_position.y-1]){
					lowestVisitCount = *visit_count[current_position.x][current_position.y-1];
					leastVisitedDirection = LEFT;
					ownLaby_setTargetTile(current_position, 0, -1);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[current_position.x][current_position.y+1]){
					lowestVisitCount = *visit_count[current_position.x][current_position.y+1];
					leastVisitedDirection = RIGHT;
					ownLaby_setTargetTile(current_position, 0, 1);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				ownLaby_setTargetTile(current_position, 1, 0);
			}
		}
	            	
		//Check that happens if Robot is facing Right       	
		if (pose.cardinalDirection == DIRECTION_EAST){
			if (canMoveForward == true ) {
				lowestVisitCount = *visit_count[current_position.x+1][current_position.y];
				leastVisitedDirection = FORWARD;
				ownLaby_setTargetTile(current_position, 1, 0);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[current_position.x][current_position.y+1]){
					lowestVisitCount = *visit_count[current_position.x][current_position.y+1];
					leastVisitedDirection = LEFT;
					ownLaby_setTargetTile(current_position, 0, 1);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[current_position.x][current_position.y-1]){
					lowestVisitCount = *visit_count[current_position.x][current_position.y-1];
					leastVisitedDirection = RIGHT;
					ownLaby_setTargetTile(current_position, 0, -1);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				ownLaby_setTargetTile(current_position, -1, 0);
			}
		}
	            	
		//Robot is facing Downwards
		if (pose.cardinalDirection == DIRECTION_SOUTH){
			if (canMoveForward == true ) {
				lowestVisitCount = *visit_count[current_position.x][current_position.y-1];
				leastVisitedDirection = FORWARD;
				ownLaby_setTargetTile(current_position, 0, -1);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[current_position.x+1][current_position.y]){
					lowestVisitCount = *visit_count[current_position.x+1][current_position.y];
					leastVisitedDirection = LEFT;
					ownLaby_setTargetTile(current_position, 1, 0);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[current_position.x-1][current_position.y]){
					lowestVisitCount = *visit_count[current_position.x-1][current_position.y];
					leastVisitedDirection = RIGHT;
					ownLaby_setTargetTile(current_position, -1, 0);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				ownLaby_setTargetTile(current_position, 0, 1);
			}
		}
	            		        
		//fprintf(stderr, "Rotating: %s\n", directionToString(leastVisitedDirection));
		robot_rotate(leastVisitedDirection);
				 
		/*if (!robot_move(FORWARD)) {
			fprintf(stderr, "cannot move forward\n");
		}*/
				 
		current_position = *ownLaby_getCurrentPosition();
		/*fprintf(stderr, "\n");*/

				/*
	                // Update the current position
	                switch (leastVisitedDirection) {
	                    case LEFT:
	                        switch (currentFacingDirection) {
	                            case FORWARD: current_position.x--; currentFacingDirection = LEFT; break;
	                            case RIGHT: current_position.y++; currentFacingDirection = FORWARD; break;
	                            case BACKWARD: current_position.x++; currentFacingDirection = RIGHT; break;
	                            case LEFT: current_position.y--; currentFacingDirection = BACKWARD; break;
	                        }
	                        break;
	                    case FORWARD:
	                        switch (currentFacingDirection) {
	                            case FORWARD: current_position.y++; break;
	                            case RIGHT: current_position.x++; break;
	                            case BACKWARD: current_position.y--; break;
	                            case LEFT: current_position.x--; break;
	                        }
	                        break;
	                    case RIGHT:
	                        switch (currentFacingDirection) {
	                            case FORWARD: current_position.x++; currentFacingDirection = RIGHT; break;
	                            case RIGHT: current_position.y--; currentFacingDirection = BACKWARD; break;
	                            case BACKWARD: current_position.x--; currentFacingDirection = LEFT; break;
	                            case LEFT: current_position.y++; currentFacingDirection = FORWARD; break;
	                        }
	                        break;
	                    case BACKWARD:
	                        switch (currentFacingDirection) {
	                            case FORWARD: current_position.y--; currentFacingDirection = BACKWARD; break;
	                            case RIGHT: current_position.x--; currentFacingDirection = LEFT; break;
	                            case BACKWARD: current_position.y++; currentFacingDirection = FORWARD; break;
	                            case LEFT: current_position.x++; currentFacingDirection = RIGHT; break;
	                        }
	                        break;
	                }

	                visit_count[current_position.x][current_position.y] += 1;
	                fprintf(stderr, "Possible directions: ");
	                for(uint8_t i = 0; i < directions; i++) {
	                    fprintf(stderr, "%s ", directionToString(possibilities[i]));
	                }
	                
	                */
	/*                fprintf(stderr, "\n");
	                fprintf(stderr, "Visited (%d)\n", visit_count[current_position.x][current_position.y]);
	                fprintf(stderr, "Current position: (%d, %d)\n", current_position.x, current_position.y);
	                fprintf(stderr, "Currently facing: %s\n", directionToString(currentFacingDirection));
	*/                   
	}
}


