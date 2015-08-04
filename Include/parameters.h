/*
 * parameters.h
 *
 *  Created on: Jul 9, 2015
 *      Author: NHH
 */

#ifndef EEPROM_PARAMETERS_H_
#define EEPROM_PARAMETERS_H_

#define SYSTEM_PARAM_OFFSET
#define SYSTEM_PARAM_LENGTH

#define PID_PARAM_OFFSET
#define PID_PARAM_LENGTH

#define SPEED_CTL_PARAM_OFFSET
#define SPEED_CTL_PARAM_LENGTH

#define MAZE_PARAM_OFFSET
#define MAZE_PARAM_LENGTH

#define IR_PARAM_OFFSET
#define IR_PARAM_LENGTH

typedef enum
{
	PARAM_ID_CHECKSUM = 0,			//4 bytes
	PARAM_ID_FIRMWARE_VERSION,		//4 bytes
	PARAM_ID_LENGTH,				//4 bytes
	//device_Id, host_Id, voltage_threshold, current_threshold
	PARAM_ID_SYSTEM_INFO,			//128
	//MAC, name, password, uart, ...
	PARAM_ID_BLUETOOTH_INFO,		//128
	PARAM_ID_BUTTON_INFO,			//4
	PARAM_ID_IR_INFO,				//64
	PARAM_ID_PID_INFO,				//128
	PARAM_ID_STR_INFO,				//64
	PARAM_MAZE_INFO,				//1024
	PARAM_IMAGE_PROCESSING_INFO,	//128
	PARAM_ID_MAX					//
} E_PARAM_ID;

extern void param_init(void);
extern void param_Set(E_PARAM_ID param_id, uint8_t *au8_value);
extern void param_Get(E_PARAM_ID param_id, uint8_t *au8_value);
extern void param_Reset(void);

#endif /* EEPROM_PARAMETERS_H_ */
