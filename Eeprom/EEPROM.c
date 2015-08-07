/*
 * EEPROM.c
 *
 *  Created on: Aug 6, 2013
 *      Author: Admin
 */
#include "../include.h"
#include "driverlib/eeprom.h"

typedef enum
{
	EEPROM_MSG_INVALID_ADDRESS = 0,
	EEPROM_MSG_INVALID_NUMOFWORDS,
	EEPROM_MSG_OK
} EEPROM_MSG;

static EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);
static EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);

static EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
{
	uint32_t Address;
	if (ui32WordAddress > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_ADDRESS);
	}
	if ((ui32WordAddress + NumOfWords) > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_NUMOFWORDS);
	}
	Address = ui32WordAddress << 2;
	ROM_EEPROMProgram(pui32_Data, Address, NumOfWords << 2);

	return EEPROM_MSG_OK;
}

static EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
{
	uint32_t Address;
	if (ui32WordAddress > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_ADDRESS);
	}
	if ((ui32WordAddress + NumOfWords) > 0x7ff)
	{
		return (EEPROM_MSG_INVALID_NUMOFWORDS);
	}
	Address = ui32WordAddress << 2;
	ROM_EEPROMRead(pui32_Data, Address, NumOfWords << 2);

	return EEPROM_MSG_OK;
}
void EEPROMConfig()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);	//Enable EEPROM
	EEPROMInit();
}
void saveMotorModel()
{
	real_T Theta[4];
	int32_t intTheta[4];
	speed_GetMotorModel(MOTOR_LEFT,Theta);
	intTheta[0]=(int32_t)(Theta[0]*1e6);
	intTheta[1]=(int32_t)(Theta[1]*1e6);
	intTheta[2]=(int32_t)(Theta[2]*1e6);
	intTheta[3]=(int32_t)(Theta[3]*1e6);
	EEPROMWrite(intTheta, 0x10, 4);

	speed_GetMotorModel(MOTOR_RIGHT,Theta);
	intTheta[0]=(int32_t)(Theta[0]*1e6);
	intTheta[1]=(int32_t)(Theta[1]*1e6);
	intTheta[2]=(int32_t)(Theta[2]*1e6);
	intTheta[3]=(int32_t)(Theta[3]*1e6);
	EEPROMWrite(intTheta, 0x20, 4);
}
void loadMotorModel()
{
	real_T Theta[4];
	int32_t intTheta[4]={0};
	EEPROMReadWords(intTheta, 0x10, 4);
	Theta[0]=((real_T)intTheta[0])*1e-6;
	Theta[1]=((real_T)intTheta[1])*1e-6;
	Theta[2]=((real_T)intTheta[2])*1e-6;
	Theta[3]=((real_T)intTheta[3])*1e-6;
	speed_SetMotorModel(MOTOR_LEFT,Theta);

	EEPROMReadWords(intTheta, 0x20, 4);
	Theta[0]=((real_T)intTheta[0])*1e-6;
	Theta[1]=((real_T)intTheta[1])*1e-6;
	Theta[2]=((real_T)intTheta[2])*1e-6;
	Theta[3]=((real_T)intTheta[3])*1e-6;
	speed_SetMotorModel(MOTOR_RIGHT,Theta);
}
void saveIRData()
{
	int32_t intIR[8];
	intIR[0]=(int32_t)(IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT));
	intIR[1]=(int32_t)(IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT));
	intIR[2]=(int32_t)(IR_get_calib_value(IR_CALIB_BASE_LEFT));
	intIR[3]=(int32_t)(IR_get_calib_value(IR_CALIB_BASE_RIGHT));
	intIR[4]=(int32_t)(IR_get_calib_value(IR_CALIB_MAX_FRONT_LEFT));
	intIR[5]=(int32_t)(IR_get_calib_value(IR_CALIB_MAX_FRONT_RIGHT));
	intIR[6]=(int32_t)(IR_get_calib_value(IR_CALIB_MAX_LEFT));
	intIR[7]=(int32_t)(IR_get_calib_value(IR_CALIB_MAX_RIGHT));
	EEPROMWrite(intIR, 0x30, 8);
}
void loadIRData()
{
	int32_t intIR[8];
	EEPROMReadWords(intIR, 0x30, 8);
	IR_load_calib_value(intIR);
}
