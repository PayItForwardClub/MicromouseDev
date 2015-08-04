/*
 * IR.h
 *
 *  Created on: Jul 4, 2015
 *      Author: NHH
 */

#ifndef IR_H_
#define IR_H_

#include <stdint.h>
#include <stdbool.h>

#define TURN_ON_IRD1()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0xff)
#define TURN_OFF_IRD1()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00)

#define TURN_ON_IRD2()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0xff)
#define TURN_OFF_IRD2()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00)

#define TURN_ON_IRD3()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0xff)
#define TURN_OFF_IRD3()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00)

#define TURN_ON_IRD4()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0xff)
#define TURN_OFF_IRD4()			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00)

#define TURN_ON_IRD_ALL()		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xff);
#define TURN_OFF_IRD_ALL()		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);

typedef enum
{
	IR_CALIB_NONE = 0,
	IR_CALIB_BASE_LEFT,
	IR_CALIB_BASE_RIGHT,
	IR_CALIB_BASE_FRONT_LEFT,
	IR_CALIB_BASE_FRONT_RIGHT,
	IR_CALIB_MAX_LEFT,
	IR_CALIB_MAX_RIGHT,
	IR_CALIB_MAX_FRONT_LEFT,
	IR_CALIB_MAX_FRONT_RIGHT,
	IR_CALIB_INVALID = 255
} IR_CALIB;

typedef struct
{
	uint32_t BaseLeft;
	uint32_t BaseRight;
	uint32_t BaseFrontLeft;
	uint32_t BaseFrontRight;
	uint32_t MaxLeft;
	uint32_t MaxRight;
	uint32_t MaxFrontLeft;
	uint32_t MaxFrontRight;
} IR_CALIB_VALUE;

extern void IRDetector_init(void);
extern uint32_t IR_GetIrDetectorValue(uint8_t Select);
extern uint32_t IR_get_calib_value(IR_CALIB select);
extern bool IR_set_calib_value(IR_CALIB select);
extern void vIR_BatteryIntRegister(void (*ISR)(void));
extern uint32_t vIR_Battery_ADCGet(void);
extern void vIR_BattSenseTrigger(void);

#endif /* IR_H_ */
