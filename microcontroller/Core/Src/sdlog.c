/*
 * sdlog.c
 *
 *  Created on: Apr 19, 2023
 *      Author: Isaak
 */

#include "sdlog.h"



void SD_GPIO_Init(){
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = SD_SPI_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SD_SPI_GPIO_PORT, &GPIO_InitStruct);
}

void SD_SPI_Init(){
	GPIO_InitTypeDef GPIO_InitStruct;

	// Configuration of GPIO pins for SPI communication

	GPIO_InitStruct.Pin =  GPIO_PIN_8 |GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Initialization of SD Card

	SD_SPI_HANDLE.Instance = SPI1;
	SD_SPI_HANDLE.Init.Mode = SPI_MODE_MASTER;
	SD_SPI_HANDLE.Init.Direction = SPI_DIRECTION_2LINES;
	SD_SPI_HANDLE.Init.DataSize = SPI_DATASIZE_8BIT;
	SD_SPI_HANDLE.Init.CLKPolarity = SPI_POLARITY_LOW;
	SD_SPI_HANDLE.Init.CLKPhase = SPI_PHASE_1EDGE;
	SD_SPI_HANDLE.Init.NSS = SPI_NSS_SOFT;
	SD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SD_SPI_HANDLE.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SD_SPI_HANDLE.Init.TIMode = SPI_TIMODE_DISABLE;
    SD_SPI_HANDLE.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SD_SPI_HANDLE.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&SD_SPI_HANDLE);
}

void SD_Write(const char * filename, const char * data){
	FATFS fs;
	FIL file;
	FRESULT res;

	// Mount SD card
	res = f_mount(&fs, "", 1);
	if(res != FR_OK)
		return;

	// Open file for writing
	res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if(res != FR_OK)
		return;

	// Write data to file
	for(int i = 0; i < strlen(data); i++){
		uint8_t byte = data[i];
		res = HAL_SPI_Transmit(&SD_SPI_HANDLE, &byte, 1, HAL_MAX_DELAY);
		if(res != HAL_OK)
			return;
	}

	// Close file

	res = f_close(&file);
	if(res != FR_OK)
		return;

	// Unmount SD card
	f_mount(NULL, "", 0);
}

void SD_READ_WRITE(){
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9))
		printf("There is an SD card detected");
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
	else
		printf("No SD card detected");
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
}


void test(){
	HAL_Init();
	SD_GPIO_Init();
	SD_SPI_Init();

	const char * filename = "test.txt";
	const char * data = "I'm working!";
	SD_Write(filename, data);

	while(1);
}

