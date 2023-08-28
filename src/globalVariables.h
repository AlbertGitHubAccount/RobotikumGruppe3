#ifndef GLOBAL_VARIABLES_H
#define GLOBAL_VARIABLES_H

#include "directions.h"
#include <stdbool.h> // Include the bool type

// Declare your global variables here
extern enum CardinalDirection *currentDirectionPtr;

// Declare a 2D array for visited locations
extern int visitedArray[21][21]; // Adjust the dimensions as needed

// Declare the global variable for the target tile
struct Tile {
	int x;
	int y;
};

extern struct Tile targetTile;

// Declare the global variable for the current position
extern struct Tile currentPosition;

// Declare boolean flags for movement directions
extern bool canGoNorth;
extern bool canGoSouth;
extern bool canGoEast;
extern bool canGoWest;

#endif
