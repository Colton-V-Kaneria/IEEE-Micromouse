/*
 * controllers.c
 *
 *  Created on: May 15, 2024
 *      Author: bat06
 */
#include "main.h"
#include "ir_sensors.h"
#include "controllers.h"
#include <stdio.h>
#include <math.h>

float fwd_error;

float FWD_Controller()
{
	float error = 0;
	float espect_dist = 180;
	int32_t Kp = 2;

	error = espect_dist - d_center;

	float fwd_output_voltage = error*Kp;

	return fwd_output_voltage;

}

