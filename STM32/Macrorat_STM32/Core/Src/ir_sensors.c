/*
 * ir_sensors.c
 *
 *  Created on: May 8, 2024
 *      Author: bat06
 */
#include "main.h"
#include "ir_sensors.h"
#include <stdio.h>
#include <math.h>

ADC_HandleTypeDef hadc1;

uint16_t dis_FL;
uint16_t dis_FR;
uint16_t dis_L;
uint16_t dis_R;

const uint16_t cali_L = 1192; //1682 // 1747 // 1699 // 1753
const uint16_t cali_R = 1206; //1590 //1688 //1600// 1683
const uint16_t cali_FL = 2964; //  AVG: 3029
const uint16_t cali_FR = 3367; // AVG: 3407

const uint16_t nominal_L ;
const uint16_t nominal_R ;
const uint16_t nominal_FL ;
const uint16_t nominal_FR ;

//float left_scale = (float)nominal_L / cali_L;
//float right_scale = (float)nominal_R / cali_R;
//float front_left_scale = (float)nominal_FL / cali_FL;
//float front_right_scale = (float)nominal_FR / cali_FR;



uint16_t measure_dist(dist_t dist) {
	GPIO_TypeDef* emitter_port;
	uint16_t emitter_pin;
	GPIO_TypeDef* receiver_port;
	uint16_t receiver_pin;

	switch(dist) {
		case DIST_FL:
			emitter_port = EMIT_FL_GPIO_Port;
			emitter_pin = EMIT_FL_Pin;
			receiver_port = RECIV_FL_GPIO_Port;
			receiver_pin = RECIV_FL_Pin;
			ADC1_Select_CH9();
			break;
		case DIST_L:
			emitter_port = EMIT_L_GPIO_Port;
			emitter_pin = EMIT_L_Pin;
			receiver_port = RECIV_L_GPIO_Port;
			receiver_pin = RECIV_L_Pin;
			ADC1_Select_CH8();
			break;
		case DIST_R:
			emitter_port = EMIT_R_GPIO_Port;
			emitter_pin = EMIT_R_Pin;
			receiver_port = RECIV_R_GPIO_Port;
			receiver_pin = RECIV_R_Pin;
			ADC1_Select_CH5();
			break;
		case DIST_FR:
			emitter_port = EMIT_FR_GPIO_Port;
			emitter_pin = EMIT_FR_Pin;
			receiver_port = RECIV_FR_GPIO_Port;
			receiver_pin = RECIV_FR_Pin;
			ADC1_Select_CH4();
			break;
		default:
			break;
	}

	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_SET);
	HAL_Delay(5);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_RESET);

	return adc_val;
}


void IR_test()
{
	dis_FR = measure_dist(DIST_FR);
	dis_FL = measure_dist(DIST_FL);
	dis_R = measure_dist(DIST_R);
	dis_L = measure_dist(DIST_L);
}



void ADC1_Select_CH4(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC1_Select_CH5(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC1_Select_CH8(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC1_Select_CH9(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}
