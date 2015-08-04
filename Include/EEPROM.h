#ifndef EEPROM_H_
#define EEPROM_H_

typedef enum
{
	EEPROM_MSG_INVALID_ADDRESS = 0,
	EEPROM_MSG_INVALID_NUMOFWORDS,
	EEPROM_MSG_OK
} EEPROM_MSG;

extern void EEPROM_Init(void);
extern EEPROM_MSG EEPROMWrite(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);
extern EEPROM_MSG EEPROMReadWords(uint32_t *pui32_Data, uint32_t ui32WordAddress, uint32_t NumOfWords);

#endif /* EEPROM_H_ */
