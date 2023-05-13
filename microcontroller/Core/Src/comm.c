/*
 * comm.c
 *
 *  Created on: May 12, 2023
 *      Author: ProfessorG
 */

#include "comm.h"

CAN_RxHeaderTypeDef can_header;
uint8_t can_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// Populate data in global variables
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_header, can_data) != HAL_OK)
	{
		Error_Handler();
	}

	// Do something with the new data
	// TODO: Copy into buffers
}
