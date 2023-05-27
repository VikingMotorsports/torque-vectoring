#include "stdio.h"
//#include "math.h"



//Threshold values for outliers
#define OUTLIER_COUNT 2
#define OUTLIER_THRESHOLD 10

typedef struct {
	unsigned long Difference;
	unsigned long Frequency;
	unsigned long IC_Value1;
	unsigned long IC_Value2;
	uint32_t CalculationOK;
	uint32_t Is_First_Captured;
	uint32_t outlier_counter;
	unsigned long Current_Frequency;
	unsigned long Good_Frequency;
} frequency_data;

void freq_init(frequency_data * freq);
int check_outliers(uint32_t curr_freq, uint32_t incoming_freq);
