/*
 * movement.h
 *
 *  Created on: 21 nov. 2019
 *      Author: payno
 */

#ifndef INC_MOVEMENT_H_
#define INC_MOVEMENT_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "fsm.h"

#define MIN_INCLINATION -700
#define MAX_INCLINATION 0

enum fsm_states_mov {
	IDLE_MOV,
};

int timeout_mov(fsm_t*);

void read_mov(fsm_t*);

uint8_t mapping_v2(float, float, float, uint8_t, uint8_t);



#endif /* INC_MOVEMENT_H_ */
