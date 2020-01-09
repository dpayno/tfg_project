/*
 * fingering.c
 *
 *  Created on: 2 nov. 2019
 *      Author: payno
 */

#include "fingering.h"

/* Declaraci�n de temporizadores */
TickType_t timeout_fingering;

/* Declaraci�n de botones */
int button_now[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* Declaraci�n de variables de nota */
uint8_t note_now, note_last;

/* Declaraci�n de mutex (externo) */
//extern SemaphoreHandle_t xMutexBtn;

/* Declaraci�n de flags (externo) */
extern int btn_flag;

/* Declaraci�n de combinaciones de notas */
int cl_a3[17] 	= 	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
int cl_bb3[17]	= 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
int cl_b3[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1};
int cl_c3[17] 	=	{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1};
int cl_db3[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1};
int cl_d3[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1};
int cl_eb3[17] 	= 	{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1};
int cl_e3[17] 	= 	{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1};
int cl_f3[17] 	= 	{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1};
int cl_gb3[17] 	= 	{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1};
int cl_g3[17] 	= 	{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1};
int cl_ab3[17] 	= 	{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1};
int cl_a4[17] 	= 	{0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1};
int cl_bb4[17] 	= 	{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1};
int cl_b4[17] 	= 	{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
int cl_c4[17] 	= 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
int cl_db4[17] 	= 	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
int cl_d4[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
int cl_eb4[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1};
int cl_e4[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,1};
int cl_f4[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,1};
int cl_gb4[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,1};

int cl_g4[17] 	= 	{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int cl_ab4[17] 	= 	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int cl_a5[17] 	= 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int cl_bb5[17] 	= 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


/* Funciones de entrada */
int timeout_btn (fsm_t* this){
	return timeout_fingering < xTaskGetTickCount();
}

/*Funciones de salida */
void read_btn(fsm_t* this){
	timeout_fingering += 25/portTICK_RATE_MS;

	button_now[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
	button_now[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	button_now[2] = 0;
	button_now[3] = 0;
	button_now[4] = 0;
	button_now[5] = 0;
	button_now[6] = 0;
	button_now[7] = 0;
	button_now[8] = 0;
	button_now[9] = 0;
	button_now[10] = 0;
	button_now[11] = 0;
	button_now[12] = 0;
	button_now[13] = 0;
	button_now[14] = 0;
	button_now[15] = 0;
	button_now[16] = 0;

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

