#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"

// Sensor input pin
#define SENSOR_PIN GPIO_PIN_0
#define SENSOR_PORT GPIOA

//Threshold values
#define OUTLIER_COUNT 2
#define OUTLIER_THRESHOLD 0.1

uint32_t current_counter = 0;
uint32_t last_counter = 0;
uint32_t Difference = 0;
uint32_t outlier_counter = 0;
uint32_t freq = 0;
uint32_t current_freq = 0;


bool check_outliers(uint32_t curr_freq, uint32_t incoming_freq) {
	uint32_t difference = incoming_freq > curr_freq ? incoming_freq - curr_freq: curr_freq - incoming_freq;
	float average = (incoming_freq + curr_freq) / 2.0;
	float percent_difference = (difference / average) * 100.0;
	if (percent_difference > OUTLIER_THRESHOLD) {
	    return true;
	} else {
		return false;
	}
}

void read_freq() {
	
	while (HAL_GPIO_ReadPin(SENSOR_PORT, SENSOR_PIN) == GPIO_PIN_RESET) {}
	
	current_counter = __HAL_TIM_GET_COUNTER(&htim);
	Difference = current_counter - last_counter;
	last_counter = current_count;
	
	freq = 1000/Difference;
	
	if (current_freq == 0) {
		current_freq = freq;
	}
	else {
		if(check_outliers(current_freq, freq)) {
			++outlier_counter;
			if (outlier_counter >= OUTLIER_COUNT) {
				outlier_counter = 0;
				HAL_Delay(1000);
			}
		}
		else {
			if(outlier_counter != 0) 
				outlier_counter = 0;
			current_freq = freq;
		}
	}
}
