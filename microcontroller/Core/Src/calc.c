/*
 * calc.c
 *
 *  Created on: Apr 27, 2023
 *      Author: ProfessorG
 */

#include "calc.h"

void convert_sensors_ADC_SI(sensors_raw_t data_in, sensors_si_t *data_out)
{

}

void calculate_torque(void)
{

}

void convert_torque_SI_DAC(torque_si_t data_in, torque_out_t *data_out)
{

}

void output_torque(void)
{

}

float steering_wheel_angle_to_steering_angle(float steering_wheel_angle)
{ // y = (8.355*10^ -5)x^2 + 0.139x - 0.03133
	float x = steering_wheel_angle;
	return (8.355E-5)*x*x + 0.139*x - 0.03133;
}

float throttle_percent_to_torque(float throttle_percent)
{

}

