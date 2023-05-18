#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"

// Sensor input pin

#define SENSOR_PIN GPIO_PIN_0
#define SENSOR_PORT GPIOA

//Threshold values for outliers
#define OUTLIER_COUNT 2
#define OUTLIER_THRESHOLD 0.1

typedef struct {
	uint32_t Difference;
	uint32_t Frequency;
	uint32_t IC_Value1;
	uint32_t IC_Value2;
	uint32_t CalculationOK;
	uint32_t Is_first_Captured;
} frequency_data;


int check_outliers(uint32_t curr_freq, uint32_t incoming_freq)
{
	uint32_t difference = incoming_freq > curr_freq ? incoming_freq - curr_freq: curr_freq - incoming_freq;
	float average = (incoming_freq + curr_freq) / 2.0;
	float percent_difference = (difference / average) * 100.0;
	if (percent_difference > OUTLIER_THRESHOLD) {
	    return 1;
	} else {
		return 0;
	}
}

frequency_data tim1 = {0};
uint32_t outlier_counter = 0;
uint32_t Current_Frequency = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (tim1.Is_first_Captured == 0)
        {
            tim1.IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            tim1.Is_first_Captured = 1;
        }
        else if (tim1.Is_first_Captured) {
                tim1.IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                if (tim1.IC_Value2 > tim1.IC_Value1)
                {
                    tim1.Difference = tim1.IC_Value2 - tim1.IC_Value1;
                    tim1.Frequency = HAL_RCC_GetPCLK1Freq() / tim1.Difference;
                    tim1.CalculationOK = 1;
					if (Current_Frequency == 0) {
			    		Current_Frequency = tim1.Frequency;
					}
					else {
						if(check_outliers(Current_Frequency, tim1.Frequency)) {
							++outlier_counter;
							if (outlier_counter >= OUTLIER_COUNT) {
								outlier_counter = 0;
								HAL_Delay(1000);
							}
						}
						else {
							if(outlier_counter != 0)
								outlier_counter = 0;
							Current_Frequency = tim1.Frequency;
						}
					}
                }
                else
                    tim1.CalculationOK = 0;
                tim1.Is_first_Captured = 0;
            }
    }
}

