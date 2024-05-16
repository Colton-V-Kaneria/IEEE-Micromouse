/*
 * motors.c
 *
 *  Created on: May 14, 2024
 *      Author: bat06
 */
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "ir_sensors.h"
#include "motors.h"

int32_t enc_left = 0;
int32_t enc_right = 0;
uint16_t raw_count_left = 0;
uint16_t raw_count_right = 0;
uint16_t prev_count_left = 0;
uint16_t prev_count_right = 0;

int32_t d_L = 0;
int32_t d_R = 0;

int32_t d_center = 0;	// center distance
int32_t angle = 0;

float v_batt = 0;
uint16_t battery_reading = 0;

const float min_v = 2.5;
int initial_PWM = 0;
int v_motor = 4;
int motor_PWM = 0;


int calc_PWM(float voltage)
{
	return (voltage/v_batt)*2047;
}

int calc_distance()
{
	return (d_L + d_R)/2;
}

int calc_angle()
{
	int angle = (int)((d_R - d_L)/(2.0 * RW) * (180.0/M_PI)) % 360;

	// These next statements ensure the result is between -180 and 180
	if (angle > 180)
	{
		angle -= 360;
	}
	else if (angle < -180)
	{
		angle += 360;
	}

	return angle;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // this is the left encoder timer
    if (htim->Instance == TIM3) {
        raw_count_left = __HAL_TIM_GET_COUNTER(htim);
        enc_left -= (int16_t)(raw_count_left - prev_count_left);
		d_L = (enc_left / 360.0) * (M_PI * diameter);
		d_center = calc_distance();
		angle = calc_angle();
        prev_count_left = raw_count_left;

    }
    if (htim->Instance == TIM4) {
        raw_count_right = __HAL_TIM_GET_COUNTER(htim);
        enc_right -= (int16_t)(raw_count_right - prev_count_right);
		d_R = (enc_right / 360.0) * (M_PI * diameter);
		d_center = calc_distance();
		angle = calc_angle();
        prev_count_right = raw_count_right;
    }
}


float calc_v_batt()
{
	ADC1_Select_CH1();

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	battery_reading = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// multiply by ratio to convert to V, then multiply by 3
    return battery_reading * v_ratio * 3;

}


void motor_test()
{

	  v_batt = calc_v_batt();		// variables for live expressions
	  initial_PWM = calc_PWM(min_v); // calculate PWM needed to set motors in motion

	  TIM2->CCR4 = initial_PWM; // right motor
	  TIM2->CCR3 = initial_PWM; // left motor

	  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1);	// spin both motors forward
	  HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
	  HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1);
	  HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);

	  HAL_Delay(35);

	  v_batt = calc_v_batt();		// variables for live expressions
	  motor_PWM = calc_PWM(v_motor);

	  TIM2->CCR4 = motor_PWM;
	  TIM2->CCR3 = motor_PWM;

	  HAL_Delay(10000);

	  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0);	// stop both motors
	  HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
	  HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0);
	  HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);


}
