
/*
 * helperFunctions.c
 *
 * Created: 28.08.2023 14:36:11
 *  Author: Albert
 */ 

#include "directions.h"

enum CardinalDirection turnLeft(enum CardinalDirection currentDirection) {
	switch (currentDirection) {
		case NORTH:
		return WEST;
		case WEST:
		return SOUTH;
		case SOUTH:
		return EAST;
		case EAST:
		return NORTH;
		default:
		return currentDirection; // Handle invalid direction gracefully
	}
}
