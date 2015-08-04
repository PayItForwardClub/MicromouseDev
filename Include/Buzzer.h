/*
 * Buzzer.h
 *
 *  Created on: Jul 7, 2015
 *      Author: NHH
 */

#ifndef BUZZER_BUZZER_H_
#define BUZZER_BUZZER_H_

#include <stdint.h>
#include <stdbool.h>

#define BUZZER_GPIO_PERIPHERAL_ENABLE()		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB)
#define BUZZER_PORT												GPIO_PORTB_BASE
#define BUZZER_PIN												GPIO_PIN_0
#define BUZZER_TIMER_PIN_AF								GPIO_PB0_T2CCP0
#define BUZZER_TIMER_PERIPHERAL_ENABLE()	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2)
#define BUZZER_TIMER											TIMER2_BASE
#define BUZZER_TIMER_CHANNEL							TIMER_A

typedef struct
{
	uint32_t Freq;
	uint32_t msTime;
} BUZZER_PULSE;

extern void buzzer_init(void);
extern void buzzer_setsound(uint32_t ulFrequency);
extern bool buzzer_low_batterry_notify(void);
extern bool buzzer_low_battery_shutdown(void);
extern void buzzer_on(uint32_t Freq, uint32_t msTime);

#endif /* BUZZER_BUZZER_H_ */
