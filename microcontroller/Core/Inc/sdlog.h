/*
 * sdlog.h
 *
 *  Created on: Apr 23, 2023
 *      Author: Isaak
 */

#ifndef SRC_SDLOG_H_
#define SRC_SDLOG_H_

/*IN COMPILER PATH MUST INCLUDE
STM32Cube\Repository\STM32Cube_FW_F4_V1.27.1\Middlewares\Third_Party\FatFs\src
OR ELSE THIS WILL NOT COMPILE
*/

//Import FatFS File System
#include "ff.h"
#include <stdio.h>
#include <string.h>

//SD Mounting Function
FRESULT sd_mount(const TCHAR * sd_path);

//SD Unmount Function
FRESULT sd_unmount(const TCHAR * sd_path);

//Write FIle
FRESULT sd_write(char * name, char * to_write);

//ReadFile
FRESULT sd_read(char * name);

//Check if SD card is saving to the file
void sd_check(FRESULT file_result);

//CreateFIle
FRESULT file_create(char * name);

//RemoveFile
FRESULT file_remove(char * name);

//CSV HEADERS
void csv_header(char * name, int file_length);

//CSV UPDATE
void csv_update(char * name, int file_length);

#endif /* SRC_SDLOG_H_ */
