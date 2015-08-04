/*
 * Parameters.c
 *
 *  Created on: Jul 22, 2015
 *      Author: NHH
 */
#include <stdint.h>
#include <stdbool.h>
//#include <iostream.h>
#include "EEPROM.h"
#include "parameters.h"

static void param_software_init(void);

/*
* Important: Param's length must be multiplied by 4
*/
const static uint16_t ui16_param_length[PARAM_ID_MAX] =
{
		4,		//PARAM_ID_CHECKSUM = 0,			//4 bytes
		4,		//PARAM_ID_FIRMWARE_VERSION,		//4 bytes
		4,		//PARAM_ID_LENGTH,				//4 bytes
		//device_Id, host_Id, voltage_threshold, current_threshold
		128,		//PARAM_ID_SYSTEM_INFO,			//
		//MAC, name, password, uart, ...
		128,	//PARAM_ID_BLUETOOTH_INFO,		//
		4,		//PARAM_ID_BUTTON_INFO,			//
		64,		//PARAM_ID_IR_INFO,				//
		128,	//PARAM_ID_PID_INFO,				//
		64,		//PARAM_ID_STR_INFO,				//
		1024,	//PARAM_MAZE_INFO,				//
		128	//PARAM_IMAGE_PROCESSING_INFO,	//
};

void param_init(void)
{
	EEPROM_Init();
	param_software_init();
}

static void param_software_init(void)
{

}

void param_Set(E_PARAM_ID param_id, uint8_t *au8_value)
{
	uint8_t i;
	volatile uint32_t ui32_paramStartAddress;
	
	ui32_paramStartAddress = 0;
	for (i = 0; i < param_id; i ++)
	{
		ui32_paramStartAddress += ui16_param_length[i];
	}
	
	ui32_paramStartAddress >>= 2;
	
	EEPROMWrite((uint32_t *)au8_value, ui32_paramStartAddress, ui16_param_length[param_id] >> 2);
}


void param_Get(E_PARAM_ID param_id, uint8_t *au8_value)
{
	uint8_t i;
	volatile uint32_t ui32_paramStartAddress;
	
	ui32_paramStartAddress = 0;
	for (i = 0; i < param_id; i ++)
	{
		ui32_paramStartAddress += ui16_param_length[i];
	}
	
	ui32_paramStartAddress >>= 2;
	
	EEPROMReadWords((uint32_t *)au8_value, ui32_paramStartAddress, ui16_param_length[param_id] >> 2);
}

void param_Reset(void)
{
	uint8_t i;
	volatile uint32_t ui32_paramStartAddress;
	volatile uint32_t reset_value = 0;
	
	ui32_paramStartAddress = 0;
	for (i = 0; i < PARAM_ID_MAX; i ++)
	{
		ui32_paramStartAddress += ui16_param_length[i];
	}
	
	ui32_paramStartAddress >>= 2;
	
	EEPROMWrite((uint32_t *)au8_value, ui32_paramStartAddress, ui16_param_length[param_id] >> 2);
}