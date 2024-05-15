/*
 * motors.h
 *
 *  Created on: May 14, 2024
 *      Author: bat06
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#define diameter 33		// wheel diameter
#define RW 41
#define max_PWM 1600
#define v_ratio 1/1206.0

float calc_v_batt();
int calc_PWM(float voltage);
int calc_distance();
int calc_angle();
void ADC1_Select_CH1(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void motor_test();


#endif /* INC_MOTORS_H_ */
