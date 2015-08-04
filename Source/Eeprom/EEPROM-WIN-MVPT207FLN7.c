/*
 * EEPROM.c
 *
 *  Created on: Aug 6, 2013
 *      Author: Admin
 */
#include "include.h"
#include "driverlib/eeprom.h"
#include "EEPROM.h"

void EEPROM_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();
}

EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
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
	EEPROMProgram(pui32_Data, Address, NumOfWords << 2);

	return EEPROM_MSG_OK;
}

EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords)
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
	EEPROMRead(pui32_Data, Address, NumOfWords << 2);

	return EEPROM_MSG_OK;
}

