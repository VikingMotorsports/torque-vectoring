/*
 * sdlog.h
 *
 *  Created on: Apr 23, 2023
 *      Author: Isaak
 */

/*#ifndef SRC_SDLOG_H_
#define SRC_SDLOG_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_sd.h"
//#include "ff.h"
#include <string.h>
#include <stdio.h>

SD_HandleTypeDef hspi2;
#define SD_SPI_HANDLE hspi2
#define SD_SPI_CS_PIN GPIO_PIN_4
#define SD_SPI_GPIO_PORT  GPIOA

#endif /* SRC_SDLOG_H_ */
