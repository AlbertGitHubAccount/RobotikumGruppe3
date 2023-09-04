/*
 * Bumper.h
 *
 * Created: 23.06.2023 16:20:57
 *  Author: Daniel
 */ 


#ifndef BUMPER_H_
#define BUMPER_H_

#include <tools/variablesAccess.h>
#include <stdint.h>
#include <stdbool.h>

void bumper_init();
bool driveAdjustYes;

bitset8_t bumper_getBumpers();

uint8_t bumper_getContacts();

void bumper_checkCollision();

#endif /* BUMPER_H_ */
