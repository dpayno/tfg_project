/*
 * pressure.c
 *
 *  Created on: 21 oct. 2019
 *      Author: payno
 */

#include "pressure.h"

/* Declaración de temporizadores */
TickType_t timeout_pressure;

/* Declaración de variables adc */
uint8_t adc_value_now, adc_value_last;
uint8_t adc_value_init;
uint8_t pres_val;

/* Declaración del ADC (externo) */
extern ADC_HandleTypeDef hadc1;

/* Declaración de mutex (externo) */
//extern SemaphoreHandle_t xMutexPres;

/* Declaración de flags (externo) */
extern int pres_flag;



/* Funciones de entrada */
int timeout_adc (fsm_t* this){
	return timeout_pressure < xTaskGetTickCount();
}

/*Funciones de salida */
void read_adc(fsm_t* this){
	timeout_pressure += 10/portTICK_RATE_MS;

	HAL_ADC_PollForConversion(&hadc1, 1000);

	adc_value_now = HAL_ADC_GetValue(&hadc1);

	if(adc_value_now != adc_value_last){
		adc_value_last = adc_value_now;
		pres_val = mapping(adc_value_now, adc_value_init, 255, 0, 127);
		pres_flag = 1;
		//xSemaphoreGive(xMutexPres);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
	}

}



/* Funciones auxiliares */
uint8_t mapping(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max){
	uint8_t aux;
	if(x > in_min){
		aux = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}else{
		aux = 0;
	}
	return aux;
}

