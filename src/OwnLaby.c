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

#include <motor/motor.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <tools/labyrinth/labyrinth.h>
#include <communication/packetTypes.h>


//#define MAX_X 21
//#define MAX_Y 21
//#define START_X 11
//#define START_Y 11

//static timeTask_time_t startTime;

LPose_t labyPose = {3, 3, 0};
Pose_t labyRobotPose = {0.0f, 0.0f, M_PI_2};	
int8_t exitDirection = -1;

int* visit_count[LABYRINTH_ROWS][LABYRINTH_COLS];
//Position current_position;
//Position target_tile;

/*const char* directionToString(RobotDirection_t direction) {
    switch(direction) {
        case LEFT: return "LEFT";
        case FORWARD: return "FORWARD";
        case RIGHT: return "RIGHT";
        case BACKWARD: return "BACKWARD";
        default: return "UNKNOWN";
    }
}*/

/*
const Position* ownLaby_getCurrentPosition(){
	return &current_position;
}

void ownLaby_setCurrentPosition(const LPose_t* labyPose){
	current_position.x = labyPose->row;
	current_position.y = labyPose->column;
}
*/

uint8_t ownLaby_getVisitCount(uint8_t row, uint8_t column) {
	return *(visit_count[row][column]);
}

void ownLaby_setVisitCount(uint8_t row, uint8_t col) {
	(*(visit_count[row][col]))++;
}

/*
const Position* ownLaby_getTargetTile(){
	return &target_tile;
}

void ownLaby_setTargetTile(Position current_position, int8_t x_change, int8_t y_change){
	target_tile.x = current_position.x + x_change;
	target_tile.y = current_position.y + y_change;
}
*/

const LPose_t* ownLaby_getPose(){
	return &labyPose; 
}

void ownLaby_setPose(){
	//labypose.row		= (uint8_t) floor(( position_getExpectedPose()->x + 3.5f * LABY_CELLSIZE) / LABY_CELLSIZE);
	//labypose.column	= (uint8_t) floor((-position_getExpectedPose()->y + 3.5f * LABY_CELLSIZE) / LABY_CELLSIZE);
	
	//vertauschen von column und row wegen der anderen Orientierung von AprilTags
	labyPose.column		= (uint8_t) floor(( position_getExpectedPose()->x + 3.5f * LABY_CELLSIZE) / LABY_CELLSIZE); 
	labyPose.row		= (uint8_t) floor((-position_getExpectedPose()->y + 3.5f * LABY_CELLSIZE) / LABY_CELLSIZE);
	
	float adjustedTheta = position_getExpectedPose()->theta + M_PI_4;
	if (adjustedTheta >= 2.0f * M_PI)
		adjustedTheta -= 2.0f * M_PI;
	
	adjustedTheta = -(floor(adjustedTheta/(M_PI_2))) + 5.0f;
	if (adjustedTheta >= 4.0f)
		adjustedTheta -= 4.0f;
	
	labyPose.cardinalDirection = (Direction_t) adjustedTheta;
}

const Pose_t* ownLaby_getRobotPose(){
	return &labyRobotPose;
}

void ownLaby_setRobotPose(){
	//labyRobotPose.x =   (((float) labyPose->row   ) * LABY_CELLSIZE) - 3.0f * LABY_CELLSIZE ;
	//labyRobotPose.y = -((((float) labyPose->column) * LABY_CELLSIZE) - 3.0f * LABY_CELLSIZE);

	//vertauschen von x und y wegen der anderen Orientierung von AprilTags
	labyRobotPose.y =   (((float) ownLaby_getPose()->row   ) * LABY_CELLSIZE) - 3.0f * LABY_CELLSIZE ;
	labyRobotPose.x = -((((float) ownLaby_getPose()->column) * LABY_CELLSIZE) - 3.0f * LABY_CELLSIZE);

	if (ownLaby_getPose()->cardinalDirection == 1) //East
		labyRobotPose.theta = 0.0f;
	if (ownLaby_getPose()->cardinalDirection == 0) //North
		labyRobotPose.theta = 1.0f * M_PI_2;
	if (ownLaby_getPose()->cardinalDirection == 3) //West
		labyRobotPose.theta = 1.0f * M_PI;	
	if (ownLaby_getPose()->cardinalDirection == 2) //South
		labyRobotPose.theta = 3.0f * M_PI_2;
}

/*bool robot_canContinue()
{
	assert(!"The method or operation is not implemented.");
	return true;
}*/

void ownLaby_init() {
	labyrinth_clearAllWalls();
	memset(visit_count, 0, LABYRINTH_ROWS * LABYRINTH_COLS * sizeof(int));
	
	//labyPose = *ownLaby_getPose();
	//current_position	= *ownLaby_getCurrentPosition();
	//ownLaby_setTargetTile(current_position, -3, -3);
	//target_tile			= *ownLaby_getTargetTile();
}

int8_t robot_getExitDirection(){
	exitDirection = -1;
	
	const LPose_t* pose = ownLaby_getPose();
	//pose.row	= ownLaby_getPose()->column;
	//pose.column	= ownLaby_getPose()->row;
	
	Walls_t walls = labyrinth_getWalls(pose->row, pose->column);
	
	if ((walls.walls != 7) || (walls.walls != 11) || (walls.walls != 13) || (walls.walls != 14)){ //Wenn es drei Wände gibt ist es auf keinen Fall der Exit
		//		0111				1011					1101					1110
		if ((pose->row == 0) && (walls.wall.north == WALLSTATE_CLEARED)){ //Roboter ist an der Nördlichen Kante
			if ((pose->cardinalDirection == DIRECTION_NORTH))
				exitDirection = 1;
			if ((pose->cardinalDirection == DIRECTION_EAST))
				exitDirection = 0;
			else
				exitDirection = 2;
		}
		
		if ((pose->column == 6) && (walls.wall.east == WALLSTATE_CLEARED)){ //Roboter ist an der Östlichen Kante
			if ((pose->cardinalDirection == DIRECTION_EAST))
				exitDirection = 1;
			if ((pose->cardinalDirection == DIRECTION_SOUTH))
				exitDirection = 0;
			else
				exitDirection = 2;
		}
		
		if ((pose->row == 6) && (walls.wall.south == WALLSTATE_CLEARED)){ //Roboter ist an der Südlichen Kante
			if ((pose->cardinalDirection == DIRECTION_SOUTH))
				exitDirection = 1;
			if ((pose->cardinalDirection == DIRECTION_WEST))
				exitDirection = 0;
			else
				exitDirection = 2;
		}
		
		if ((pose->column == 0) && (walls.wall.west == WALLSTATE_CLEARED)){ //Roboter ist an der Westlichen Kante
			if ((pose->cardinalDirection == DIRECTION_WEST))
				exitDirection = 1;
			if ((pose->cardinalDirection == DIRECTION_NORTH))
				exitDirection = 0;
			else
				exitDirection = 2;
		}
	}
	
	if (walls.walls == 15){
		exitDirection = 5;
	}
	
	return exitDirection;
}

bool out_of_bounds_check(){
	const LPose_t* pose = ownLaby_getPose();
	
	return (pose->row >= 7) || (pose->column >= 7);
}

void robot_rotate(RobotDirection_t localDirection){
	//Motor_stopAll();
	//timeTask_time_t now;
	//timeTask_getTimestamp(&now);
	//if (timeTask_getDuration(&startTime, &now) > 3000000UL){
		if (localDirection == FORWARD)
			setState(DRIVE_FORWARD);
		if(localDirection == LEFT)
			setState(TURN_LEFT);
		if(localDirection == RIGHT)
			setState(TURN_RIGHT);
		if(localDirection == BACKWARD)
			setState(TURN_AROUND);
	//}
}

/*bool robot_move(RobotDirection_t moveState){
	bool hasMoved = false;
	if (moveState == FORWARD){
		if (robot_canMove(moveState) == true){
			setState(DRIVE_FORWARD);
			hasMoved = true;
		}
	}

	return hasMoved;
}*/

Direction_t ownLaby_localToCardinal(RobotDirection_t localDirection, Direction_t cardinalDirection){
	int8_t newCardinalDirection = ((int8_t)localDirection - 1) + ((int8_t)cardinalDirection);
	
	if (newCardinalDirection >= 4)
		newCardinalDirection -= 4;
	if (newCardinalDirection < 0)
		newCardinalDirection += 4;
	
	return (Direction_t) newCardinalDirection;
}

bool robot_isWall(RobotDirection_t localDirection){
	const LPose_t* pose = ownLaby_getPose();
	//pose.row	= ownLaby_getPose()->column;
	//pose.column	= ownLaby_getPose()->row;
	
	Direction_t cardinalDirection = ownLaby_localToCardinal(localDirection, pose->cardinalDirection);

	Walls_t walls = labyrinth_getWalls(pose->cardinalDirection, pose->cardinalDirection);
	
	bool isWall = ((walls.walls & (1 << pose->cardinalDirection)) >> pose->cardinalDirection) == WALLSTATE_SET;
	if (! isWall) {
		const IR_value_t* IR_value = IR_getIR_value();
		float IR_specificValue = 150.0f;
		if (localDirection == FORWARD)
			IR_specificValue = IR_value->frontIR;
		if (localDirection == LEFT)
			IR_specificValue = IR_value->leftIR;
		if (localDirection == RIGHT)
			IR_specificValue = IR_value->rightIR;
			
		if(IR_specificValue < 150.0f){
			isWall = true;
			walls.walls |= 1 << cardinalDirection;
			labyrinth_setWalls(pose->row, pose->column, walls);
		}
	}
	return isWall;
}

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

	ownLaby_setPose();
	ownLaby_setRobotPose();

	if (out_of_bounds_check()) {
		setState(IDLE);
		return;
	}
	
	exitDirection = robot_getExitDirection();
	if (exitDirection > -1) {
		if (exitDirection == 5){
			Motor_stopAll();
		}
		robot_rotate((RobotDirection_t)exitDirection);
		//setState(DRIVE_EXIT); würde hier zu früh gemacht
	}
	else {
		const LPose_t* pose = ownLaby_getPose();        

		visit_count[pose->row][pose->column]++;
		/*fprintf(stderr, "Current position: (%d, %d, %d)\n", current_position.x, current_position.y, pose.cardinalDirection);
		fprintf(stderr, "Visited (%d)\n", visit_count[current_position.x][current_position.y]);*/

		//Check which directions are available
		if (robot_canMove(LEFT))	{canMoveLeft	= true;}
		if (robot_canMove(FORWARD))	{canMoveForward	= true;}
	    if (robot_canMove(RIGHT))	{canMoveRight	= true;}
	                   
	    //Check direction robot is facing and calculating distances relative to its direction
	    if (pose->cardinalDirection == DIRECTION_NORTH){
			if (canMoveForward == true) {
				lowestVisitCount = *visit_count[pose->row][pose->column+1];
				leastVisitedDirection = FORWARD;
				//ownLaby_setTargetTile(current_position, 0, 1);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[pose->row-1][pose->column]){
					lowestVisitCount = *visit_count[pose->row-1][pose->column];
					leastVisitedDirection = LEFT;
					//ownLaby_setTargetTile(current_position, -1, 0);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[pose->row+1][pose->column]){
					lowestVisitCount = *visit_count[pose->row+1][pose->column];
					leastVisitedDirection = RIGHT;
					//ownLaby_setTargetTile(current_position, 1, 0);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				//ownLaby_setTargetTile(current_position, 0, -1);
			}
		}
	            	
		//Check that happens if Robot is facing Left
		if (pose->cardinalDirection == DIRECTION_WEST){
			if (canMoveForward == true ) {
				lowestVisitCount = *visit_count[pose->row-1][pose->column];
				leastVisitedDirection = FORWARD;
				//ownLaby_setTargetTile(current_position, -1, 0);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[pose->row][pose->column-1]){
					lowestVisitCount = *visit_count[pose->row][pose->column-1];
					leastVisitedDirection = LEFT;
					//ownLaby_setTargetTile(current_position, 0, -1);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[pose->row][pose->column+1]){
					lowestVisitCount = *visit_count[pose->row][pose->column+1];
					leastVisitedDirection = RIGHT;
					//ownLaby_setTargetTile(current_position, 0, 1);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				//ownLaby_setTargetTile(current_position, 1, 0);
			}
		}
	            	
		//Check that happens if Robot is facing Right       	
		if (pose->cardinalDirection == DIRECTION_EAST){
			if (canMoveForward == true ) {
				lowestVisitCount = *visit_count[pose->row+1][pose->column];
				leastVisitedDirection = FORWARD;
				//ownLaby_setTargetTile(current_position, 1, 0);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[pose->row][pose->column+1]){
					lowestVisitCount = *visit_count[pose->row][pose->column+1];
					leastVisitedDirection = LEFT;
					//ownLaby_setTargetTile(current_position, 0, 1);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[pose->row][pose->column-1]){
					lowestVisitCount = *visit_count[pose->row][pose->column-1];
					leastVisitedDirection = RIGHT;
					//ownLaby_setTargetTile(current_position, 0, -1);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				//ownLaby_setTargetTile(current_position, -1, 0);
			}
		}
	            	
		//Robot is facing Downwards
		if (pose->cardinalDirection == DIRECTION_SOUTH){
			if (canMoveForward == true ) {
				lowestVisitCount = *visit_count[pose->row][pose->column-1];
				leastVisitedDirection = FORWARD;
				//ownLaby_setTargetTile(current_position, 0, -1);
			}
			if (canMoveLeft == true){
				if (lowestVisitCount > *visit_count[pose->row+1][pose->column]){
					lowestVisitCount = *visit_count[pose->row+1][pose->column];
					leastVisitedDirection = LEFT;
					//ownLaby_setTargetTile(current_position, 1, 0);
				}
			}
			if (canMoveRight == true){
				if (lowestVisitCount > *visit_count[pose->row-1][pose->column]){
					lowestVisitCount = *visit_count[pose->row-1][pose->column];
					leastVisitedDirection = RIGHT;
					//ownLaby_setTargetTile(current_position, -1, 0);
				}
			}
			if (!canMoveRight && !canMoveLeft && !canMoveForward){
				leastVisitedDirection = BACKWARD;
				//ownLaby_setTargetTile(current_position, 0, 1);
			}
		}
	            		        
		//fprintf(stderr, "Rotating: %s\n", directionToString(leastVisitedDirection));
		robot_rotate(leastVisitedDirection);
		//robot_move(FORWARD);
				 
		/*if (!robot_move(FORWARD)) {
			fprintf(stderr, "cannot move forward\n");
		}*/
				 
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


