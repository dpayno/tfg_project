/*
 * fingering.c
 *
 *  Created on: 2 nov. 2019
 *      Author: payno
 */

#include "fingering.h"

/* Declaración de temporizadores */
TickType_t timeout_fingering;

/* Declaración de botones */
int button_now[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* Declaración de variables de nota */
uint8_t note_now, note_last;

/* Declaración de mutex (externo) */
//extern SemaphoreHandle_t xMutexBtn;

/* Declaración de flags (externo) */
extern int btn_flag;

/* Declaración de combinaciones de notas */
				/*	 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16									*/
int cl_a3[17] 	= 	{0,1,0,1,0,1,1,0,0,0, 0, 1, 1, 0, 0, 0, 0};
int cl_bb3[17]	= 	{0,1,0,1,0,1,1,0,0,0, 0, 1, 0, 0, 0, 0, 0};
int cl_b3[17] 	= 	{0,1,0,1,0,1,1,0,0,0, 0, 0, 1, 0, 1, 0, 0};
int cl_c3[17] 	=	{0,1,0,1,0,1,1,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_db3[17] 	= 	{0,1,0,1,0,1,1,1,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_d3[17] 	= 	{0,1,0,1,0,1,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_eb3[17] 	= 	{0,1,0,1,1,1,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_e3[17] 	= 	{0,1,0,1,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_f3[17] 	= 	{0,1,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_gb3[17] 	= 	{0,1,0,0,0,0,0,0,1,0, 0, 0, 0, 0, 0, 0, 0};
int cl_g3[17] 	= 	{0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_ab3[17] 	= 	{0,0,0,0,1,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_a4[17] 	= 	{0,0,1,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_bb4[17] 	= 	{1,0,1,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_b4[17] 	= 	{1,1,0,1,0,1,1,0,0,1, 0, 1, 1, 0, 1, 0, 1};
int cl_c4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 1, 1, 0, 1, 0, 1};
int cl_db4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 1, 1, 1, 0, 1, 0, 1};
int cl_d4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 1, 1, 0, 1, 0, 0};
int cl_eb4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 1, 1, 0, 1, 1, 0};
int cl_e4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 1, 1, 0, 0, 0, 0};
int cl_f4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 1, 0, 0, 0, 0, 0};
int cl_gb4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 1, 0, 1, 0, 0, 0};
int cl_g4[17] 	= 	{1,1,0,1,0,1,1,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_ab4[17] 	= 	{1,1,0,1,0,1,1,1,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_a5[17] 	= 	{1,1,0,1,0,1,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};
int cl_bb5[17] 	= 	{1,1,0,1,1,1,0,0,0,0, 0, 0, 0, 0, 0, 0, 0};


/* Funciones de entrada */
int timeout_btn (fsm_t* this){
	return timeout_fingering < xTaskGetTickCount();
}

/*Funciones de salida */
void read_btn(fsm_t* this){
	timeout_fingering += 25/portTICK_RATE_MS;

	button_now[0] = HAL_GPIO_ReadPin(btn_1_GPIO_Port, btn_1_Pin);
	button_now[1] = HAL_GPIO_ReadPin(btn_2_GPIO_Port, btn_2_Pin);
	button_now[2] = HAL_GPIO_ReadPin(btn_3_GPIO_Port, btn_3_Pin);
	button_now[3] = HAL_GPIO_ReadPin(btn_4_GPIO_Port, btn_4_Pin);
	button_now[4] = HAL_GPIO_ReadPin(btn5_GPIO_Port, btn5_Pin);
	button_now[5] = HAL_GPIO_ReadPin(btn6_GPIO_Port, btn6_Pin);
	button_now[6] = HAL_GPIO_ReadPin(btn_7_GPIO_Port, btn_7_Pin);
	button_now[7] = HAL_GPIO_ReadPin(btn_8_GPIO_Port, btn_8_Pin);
	button_now[8] = HAL_GPIO_ReadPin(btn_9_GPIO_Port, btn_9_Pin);
	button_now[9] = HAL_GPIO_ReadPin(btn_10_GPIO_Port, btn_10_Pin);
	button_now[10] = HAL_GPIO_ReadPin(btn_11_GPIO_Port, btn_11_Pin);
	button_now[11] = HAL_GPIO_ReadPin(btn_12_GPIO_Port, btn_12_Pin);
	button_now[12] = HAL_GPIO_ReadPin(btn_13_GPIO_Port, btn_13_Pin);
	button_now[13] = HAL_GPIO_ReadPin(btn_14_GPIO_Port, btn_14_Pin);
	button_now[14] = HAL_GPIO_ReadPin(btn_15_GPIO_Port, btn_15_Pin);
	button_now[15] = HAL_GPIO_ReadPin(btn_16_GPIO_Port, btn_16_Pin);
	button_now[16] = HAL_GPIO_ReadPin(btn_17_GPIO_Port, btn_17_Pin);

	note_now = get_note(button_now);

	if(note_now != note_last){
		btn_flag = 1;
		//xSemaphoreGive(xMutexBtn);
	}

	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
}


/*Funciones auxiliares*/
uint8_t get_note(){
	uint8_t nota = 0;
	if(memcmp(button_now, cl_a3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_bb3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_b3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_c3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_db3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_d3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_eb3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_e3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_f3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_gb3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_g3, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_a4, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_bb4, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_b4, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_c4, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_db4, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_d4, sizeof(button_now)) == 0){
		nota = 0;
	}
	else if(memcmp(button_now, cl_eb4, sizeof(button_now)) == 0){
		nota = 0;
	}

	else if(memcmp(button_now, cl_g4, sizeof(button_now)) == 0){
		nota = 0x3C;
	}
	else if(memcmp(button_now, cl_ab4, sizeof(button_now)) == 0){
		nota = 0x3E;
	}
	else if(memcmp(button_now, cl_a5, sizeof(button_now)) == 0){
		nota = 0x43;
	}
	else if(memcmp(button_now, cl_bb5, sizeof(button_now)) == 0){
		nota = 0x40;
	}
	else{
		nota = 0;
	}
	return nota;
}


