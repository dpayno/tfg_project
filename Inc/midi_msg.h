/*
 * midi_msg.h
 *
 *  Created on: 2 nov. 2019
 *      Author: payno
 */

#ifndef INC_MIDI_MSG_H_
#define INC_MIDI_MSG_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "fsm.h"

enum fsm_states_msg {
	IDLE_MSG,
};


int check_btn_flag (fsm_t*);
int check_pres_flag (fsm_t*);
int check_mov_flag (fsm_t*);

void send_new_note(fsm_t*);
void send_new_pres(fsm_t*);
void send_new_mov(fsm_t*);



#endif /* INC_MIDI_MSG_H_ */
