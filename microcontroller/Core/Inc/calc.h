/*
 * calc.h
 *
 *  Created on: Apr 21, 2023
 *      Author: ProfessorG
 */

#include <stdint.h>

#ifndef INC_CALC_H_
#define INC_CALC_H_

typedef struct {
	uint16_t steering_wheel_angle;
	uint16_t throttle_percent;
	uint16_t velocity_left;
	uint16_t velocity_right;
	uint16_t timestamp; // TODO: Format
} sensors_raw_t;

typedef struct { // TODO: Units
	float steering_wheel_angle; // degrees
	float throttle_percent; // ???
	float velocity_left; // ???
	float velocity_right; // ???
	uint16_t timestamp;
} sensors_si_t;

typedef struct {
	float steering_angle;
	float velocity;
} LUT_t_in;

typedef struct {
	uint16_t power_left;
	uint16_t power_right;
} torque_out_t;

typedef struct {
	float power_left;
	float power_right;
} torque_si_t;

void convert_sensors_ADC_SI(sensors_raw_t data_in, sensors_si_t *data_out);
void calculate_torque(void);
void convert_torque_SI_DAC(torque_si_t data_in, torque_out_t *data_out);
void output_torque(void);
float steering_wheel_angle_to_steering_angle(float steering_wheel_angle);
float throttle_percent_to_torque(float throttle_percent);

#endif /* INC_CALC_H_ */
