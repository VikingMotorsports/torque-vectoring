#include "stdio.h"
//#include "math.h"



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
	uint32_t outlier_counter;
	uint32_t Current_Frequency;
} frequency_data;

int check_outliers(uint32_t curr_freq, uint32_t incoming_freq);
