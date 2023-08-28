/*
 * globalVariables.c
 *
 * Created: 28.08.2023 14:02:35
 *  Author: Albert
 */ 

// globalVariables.c
#include "globalVariables.h"
#include <stdbool.h> 

// Define your global variables here
enum CardinalDirection actualCurrentDirection;
enum CardinalDirection *currentDirectionPtr = &actualCurrentDirection;

int visitedArray[10][10]; // Adjust the dimensions as needed

// Define the global variables for the target tile and current position
struct Tile targetTile = {0, 0}; // Set initial target tile coordinates
struct Tile currentPosition = {13, 13}; // Set initial current position coordinates

// Define boolean flags for movement directions
bool canGoNorth = true;
bool canGoSouth = true;
bool canGoEast = true;
bool canGoWest = true;