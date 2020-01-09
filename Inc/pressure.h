/*
 * pressure.h
 *
 *  Created on: 21 oct. 2019
 *      Author: payno
 */

#ifndef INC_PRESSURE_H_
#define INC_PRESSURE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "fsm.h"

enum fsm_states_adc {
	IDLE_ADC,
};

int timeout_adc(fsm_t*);

void read_adc(fsm_t*);

uint8_t mapping(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

#endif /* INC_PRESSURE_H_ */
