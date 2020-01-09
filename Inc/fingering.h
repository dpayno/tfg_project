/*
 * fingering.h
 *
 *  Created on: 2 nov. 2019
 *      Author: payno
 */

#ifndef INC_FINGERING_H_
#define INC_FINGERING_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "fsm.h"

enum fsm_states_btn {
	IDLE_BTN,
};

int timeout_btn(fsm_t*);

void read_btn(fsm_t*);

uint8_t get_note();

#endif /* INC_FINGERING_H_ */
