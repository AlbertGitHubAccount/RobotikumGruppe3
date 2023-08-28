// globalVariables.h
#ifndef GLOBAL_VARIABLES_H
#define GLOBAL_VARIABLES_H

// Include the necessary headers
#include "directions.h"

// Declare your global variables here
extern enum CardinalDirection *currentDirectionPtr;

// Declare a 2D array for visited locations
extern int visitedArray[10][10]; // Adjust the dimensions as needed

// Declare a structure for tile coordinates
struct Tile {
	int x;
	int y;
};

// Declare the global variable for the target tile
extern struct Tile targetTile;

// Declare the global variable for the current position
extern struct Tile currentPosition;

#endif
