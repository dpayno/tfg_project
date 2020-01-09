/*
 * midi_msg.c
 *
 *  Created on: 2 nov. 2019
 *      Author: payno
 */

#include "midi_msg.h"

/* Declaración de variables de mensaje */
uint8_t midi_pres_msg[3] = {0xB0, 0x0B, 0x00}; // [3] Hasta 0x7F (Expression controller CC)
uint8_t midi_note_on_msg[3] = {0x90, 0x00, 0x7F}; // [2] nota [3] valor maximo
uint8_t midi_note_off_msg[3] = {0x90, 0x00, 0x00};
uint8_t midi_all_notes_off[3] = {0xB0, 0x7B, 0x00};
uint8_t midi_mod_msg[3] = {0xB0, 0x01, 0x00};

/* Declaración de variables de nota (externas) */
extern uint8_t note_now, note_last;

/* Declaración de flags (externos) */
extern int btn_flag;
extern int pres_flag;
extern int mov_flag;

/* Declaración de USART (externa) */
extern UART_HandleTypeDef huart2;

/* Declaración de variables adc (externas) */
extern uint8_t pres_val;

/* Declaración de variables mov (externas) */
extern uint8_t mod_value_now;

/* Declaración de semáforo (externo) */
//extern SemaphoreHandle_t xMutexMsg;
//extern SemaphoreHandle_t xMutexBtn, xMutexPres, xMutexMov;

/* Funciones de entrada */
int check_btn_flag (fsm_t* this){
	if(btn_flag){//if(xSemaphoreTake(xMutexBtn, portMAX_DELAY) == pdTRUE){  //portMAX_DELAY
		return 1;//btn_flag;
	}else{
		return 0;
	}
}

int check_pres_flag (fsm_t* this){
	if(pres_flag){//if(xSemaphoreTake(xMutexPres, portMAX_DELAY) == pdTRUE){
		return 1;//pres_flag;
	}else{
		return 0;
	}
}

int check_mov_flag (fsm_t* this){
	if(mov_flag){//if(xSemaphoreTake(xMutexMov, portMAX_DELAY) == pdTRUE){
		return 1;//mov_flag;
	}else{
		return 0;
	}
}



/*Funciones de salida */
void send_new_note(fsm_t* this){
	if(midi_note_on_msg[1] != 0){
		midi_note_off_msg[1] = note_last;
		HAL_UART_Transmit(&huart2, (uint8_t*)&midi_note_off_msg, 3, 100);
	}

	midi_note_on_msg[1] = note_now;
	if(midi_note_on_msg[1] != 0){
		HAL_UART_Transmit(&huart2, (uint8_t*)&midi_note_on_msg, 3, 100);
		note_last = note_now;
	}

	btn_flag = 0;
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void send_new_pres(fsm_t* this){
	midi_pres_msg[2] = pres_val;
	HAL_UART_Transmit(&huart2, (uint8_t*)&midi_pres_msg, 3, 100);
	pres_flag = 0;
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}


void send_new_mov(fsm_t* this){
	midi_mod_msg[2] = mod_value_now;
	HAL_UART_Transmit(&huart2, (uint8_t*)&midi_mod_msg, 3, 100);
	mov_flag = 0;
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}


