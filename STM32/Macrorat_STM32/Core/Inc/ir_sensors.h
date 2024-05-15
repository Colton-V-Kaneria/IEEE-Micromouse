/*
 * ir_sensors.h
 *
 *  Created on: May 8, 2024
 *      Author: bat06
 */

#ifndef INC_IR_SENSORS_H_
#define INC_IR_SENSORS_H_

typedef enum {
	DIST_FL,
	DIST_FR,
	DIST_R,
	DIST_L
} dist_t;


void ADC1_Select_CH4(void);
void ADC1_Select_CH5(void);
void ADC1_Select_CH8(void);
void ADC1_Select_CH9(void);


#endif /* INC_IR_SENSORS_H_ */
