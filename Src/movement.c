/*
 * movement.c
 *
 *  Created on: 21 nov. 2019
 *      Author: payno
 */

#include "movement.h"

/* Declaración de temporizadores */
TickType_t timeout_movement;

/* Declaración de variables adc */
uint8_t mod_value_now, mod_value_last;
LIS3DSH_DataScaled inc_value;

/* Declaración de mutex (externo) */
//extern SemaphoreHandle_t xMutexMov;

/* Declaración de flags (externo) */
extern int mov_flag;



/* Funciones de entrada */
int timeout_mov (fsm_t* this){
	return timeout_movement < xTaskGetTickCount();
}

/*Funciones de salida */
void read_mov(fsm_t* this){
	timeout_movement += 100/portTICK_RATE_MS;
	if(LIS3DSH_PollDRDY(1000) == true){
		inc_value = LIS3DSH_GetDataScaled();
	}

	mod_value_now = mapping_v2(inc_value.y, MIN_INCLINATION, MAX_INCLINATION, 0, 127);
	if(mod_value_now != mod_value_last){
		mod_value_last = mod_value_now;
		mov_flag = 1;
		//xSemaphoreGive(xMutexMov);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	}


}



/* Funciones auxiliares */
uint8_t mapping_v2(float x, float in_min, float in_max, uint8_t out_min, uint8_t out_max){
	uint8_t aux;
	if(x > in_min){
		aux = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		if(aux > 127){
			aux = 127;
		}
	}else{
		aux = 0;
	}
	return aux;
}
