#include "directions.h"

enum CardinalDirection turnLeftCardinalChange(enum CardinalDirection currentDirection) {
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

enum CardinalDirection turnRightCardinalChange(enum CardinalDirection currentDirection) {
	switch (currentDirection) {
		case NORTH:
		return EAST;
		case EAST:
		return SOUTH;
		case SOUTH:
		return WEST;
		case WEST:
		return NORTH;
		default:
		return currentDirection; // Handle invalid direction gracefully
	}
}

enum CardinalDirection turnBackwardCardinalChange(enum CardinalDirection currentDirection) {
	switch (currentDirection) {
		case NORTH:
		return SOUTH;
		case EAST:
		return WEST;
		case SOUTH:
		return NORTH;
		case WEST:
		return EAST;
		default:
		return currentDirection; // Handle invalid direction gracefully
	}
}
