/*
 * sdlog.c
 *
 *  Created on: Apr 19, 2023
 *      Author: Isaak
 */

#include "sdlog.h"

//DECLARATION OF GLOBAL VARIABLES

FATFS file_system;
FIL file;
FILINFO file_info;
FRESULT file_result;
UINT bytes_to_write, bytes_read;

FRESULT sd_mount(const TCHAR * sd_path){
	file_result = f_mount(&file_system, sd_path, 1);
	if(file_result != FR_OK)
		perror("!!!! ERROR !!!! SD CARD NOT MOUNTED !!!!");
	printf("SD card mounted successfully");
	return file_result;
}

FRESULT sd_unmount(const TCHAR * sd_path){
	file_result = f_mount(NULL, sd_path, 0);
	if(file_result == FR_OK)
		printf("SD card has been unmounted successfully");
	perror("!!!! ERROR !!!! SD CARD STILL MOUNTED !!!!");
	return file_result;
}

FRESULT Write_File (char * name, char * to_write){
	file_result = f_stat (name, &file_info);
	//Check if file exists	``
	if (file_result != FR_OK)
	{
		char * buf = malloc(100*sizeof(char));
		sprintf (buf, "ERROR!!! *%s* does not exists\n", name);
		printf (buf);
	    free(buf);
	    return file_result;
	}

	else
	{
	    //Create a file with R/W Permissions
	    file_result = f_open(&file, name, FA_OPEN_EXISTING | FA_WRITE);
	    if (file_result != FR_OK)
	    {
	    	char * buf = malloc(100*sizeof(char));
	    	sprintf (buf, "ERROR!!! No. %d in opening file *%s*\n", file_result, name);
	    	printf(buf);
	        free(buf);
	        return file_result;
	    }

	    else
	    {
	    	file_result = f_write(&file, to_write, strlen(to_write), &bytes_to_write);
	    	if (file_result != FR_OK)
	    	{
	    		char * buf = malloc(100*sizeof(char));
	    		sprintf (buf, "ERROR!!! No. %d while writing to the FILE *%s*\n", file_result, name);
	    		printf(buf);
	    		free(buf);
	    	}

	    	//Close the file
	    	file_result = f_close(&file);
	    	if (file_result != FR_OK)
	    	{
	    		char * buf = malloc(100*sizeof(char));
	    		sprintf (buf, "ERROR!!! No. %d in closing file *%s* after writing it\n", file_result, name);
	    		printf(buf);
	    		free(buf);
	    	}
	    	else
	    	{
	    		char *buf = malloc(100*sizeof(char));
	    		sprintf (buf, "File *%s* is WRITTEN and CLOSED successfully\n", name);
	    		printf(buf);
	    		free(buf);
	    	}
	    }
	    return file_result;
	}
}

//ReadFile
FRESULT sd_read(char * name){
	/**** check whether the file exists or not ****/
		file_result = f_stat (name, &file_info);
		if (file_result != FR_OK)
		{
			char *buf = malloc(100*sizeof(char));
			sprintf (buf, "ERRROR!!! *%s* does not exists\n", name);
			printf (buf);
			free(buf);
		    return file_result;
		}

		else
		{
			/* Open file to read */
			file_result = f_open(&file, name, FA_READ);

			if (file_result != FR_OK)
			{
				char *buf = malloc(100*sizeof(char));
				sprintf (buf, "ERROR!!! No. %d in opening file *%s*\n", file_result, name);
				printf(buf);
			    free(buf);
			    return file_result;
			}

			/* Read data from the file
			* see the function details for the arguments */

			char *buffer = malloc(sizeof(f_size(&file)));
			file_result = f_read (&file, buffer, f_size(&file), &bytes_read);
			if (file_result != FR_OK)
			{
				char *buf = malloc(100*sizeof(char));
				free(buffer);
			 	sprintf (buf, "ERROR!!! No. %d in reading file *%s*\n", file_result, name);
			 	printf(buffer);
			  	free(buf);
			}

			else
			{
				printf(buffer);
				free(buffer);

				/* Close file */
				file_result = f_close(&file);
				if (file_result != FR_OK)
				{
					char *buf = malloc(100*sizeof(char));
					sprintf (buf, "ERROR!!! No. %d in closing file *%s*\n", file_result, name);
					printf(buf);
					free(buf);
				}
				else
				{
					char *buf = malloc(100*sizeof(char));
					sprintf (buf, "File *%s* CLOSED successfully\n", name);
					printf(buf);
					free(buf);
				}
			}
		    return file_result;
		}
}

//Check if SD card is saving
void sd_check(FRESULT fresult){
	if(fresult != FR_OK)
		printf("!!!! ERROR !!!!! DATA IS NOT SAVING !!!!");
}


//CreateFIle
FRESULT file_create(char * name){
		file_result = f_stat (name, &file_info);
		if (file_result == FR_OK)
		{
			char * buf = malloc(100*sizeof(char));
			sprintf (buf, "ERROR!!! *%s* already exists!!!!\n use Update_File \n",name);
			printf(buf);
			free(buf);
		    return file_result;
		}
		else
		{
			file_result = f_open(&file, name, FA_CREATE_ALWAYS|FA_READ|FA_WRITE);
			if (file_result != FR_OK)
			{
				char * buf = malloc(100*sizeof(char));
				sprintf (buf, "ERROR!!! No. %d in creating file *%s*\n", file_result, name);
				printf(buf);
				free(buf);
			    return file_result;
			}
			else
			{
				char * buf = malloc(100*sizeof(char));
				sprintf (buf, "*%s* created successfully\n Now use Write_File to write data\n", name);
				printf(buf);
				free(buf);
			}

			file_result = f_close(&file);
			if (file_result != FR_OK)
			{
				char * buf = malloc(100*sizeof(char));
				sprintf (buf, "ERROR No. %d in closing file *%s*\n", file_result, name);
				printf(buf);
				free(buf);
			}
			else
			{
				char * buf = malloc(100*sizeof(char));
				sprintf (buf, "File *%s* CLOSED successfully\n", name);
				printf(buf);
				free(buf);
			}
		}
	    return file_result;
}

//RemoveFile
FRESULT file_remove(char * name){
	/**** check whether the file exists or not ****/
		file_result = f_stat (name, &file_info);
		if (file_result != FR_OK){
			char * buf = malloc(100*sizeof(char));
			sprintf (buf, "ERROR!!! *%s* does not exists\n", name);
			printf(buf);
			free(buf);
			return file_result;
		} else{
			file_result = f_unlink(name);
			if (file_result == FR_OK){
				char * buf = malloc(100*sizeof(char));
				sprintf (buf, "*%s* has been removed successfully\n", name);
				printf(buf);
				free(buf);
			}else{
				char * buf = malloc(100*sizeof(char));
				sprintf (buf, "ERROR No. %d in removing *%s*\n",file_result, name);
				printf(buf);
				free(buf);
			}
		}
		return file_result;
}


void csv_header(char * name, int file_length){
}


void csv_update(char * name, int file_length){

}


