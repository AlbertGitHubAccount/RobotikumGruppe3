
/*
 * helperFunctions.h
 *
 * Created: 28.08.2023 14:35:55
 *  Author: Albert
 */ 

#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include "directions.h"

// Declare the functions defined in helperFunctions.c
enum CardinalDirection turnLeftCardinalChange(enum CardinalDirection currentDirection);
enum CardinalDirection turnRightCardinalChange(enum CardinalDirection currentDirection);
enum CardinalDirection turnBackwardCardinalChange(enum CardinalDirection currentDirection);

#endif


//HOW TO USE THE METHODS:
/*

#include <stdio.h>
#include "helperFunctions.h"

int main() {
	enum CardinalDirection currentDirection = NORTH;
	printf("Current direction: %d\n", currentDirection);

	// Perform a right turn using the function from helperFunctions.c
	currentDirection = turnRightCardinalChange(currentDirection);
	printf("After right turn: %d\n", currentDirection);

	// Perform a left turn using the function from helperFunctions.c
	currentDirection = turnLeftCardinalChange(currentDirection);
	printf("After left turn: %d\n", currentDirection);

	// Perform a backward turn using the function from helperFunctions.c
	currentDirection = turnBackwardCardinalChange(currentDirection);
	printf("After backward turn: %d\n", currentDirection);

	return 0;
}

*/