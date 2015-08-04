/*
 * SystemConfig.h
 *
 *  Created on: Jul 15, 2013
 *      Author: Admin
 */

#ifndef SYSTEMCONFIG_H_
#define SYSTEMCONFIG_H_

#include <stdint.h>

#define DRV_ENABLE_LEFT_CHN_PERIPHERAL		SYSCTL_PERIPH_GPIOB
#define DRV_ENABLE_RIGHT_CHN_PERIPHERAL		SYSCTL_PERIPH_GPIOB
#define DRV_ENABLE_LEFT_CHN_PORT			GPIO_PORTB_BASE
#define DRV_ENABLE_RIGHT_CHN_PORT			GPIO_PORTB_BASE
#define DRV_ENABLE_LEFT_CHN_PIN				GPIO_PIN_3
#define DRV_ENABLE_RIGHT_CHN_PIN			GPIO_PIN_7

#define BOOST_ENABLE_PREIPHERAL				SYSCTL_PERIPH_GPIOA
#define BOOST_ENABLE_PORT					GPIO_PORTA_BASE
#define BOOST_ENABLE_PIN					GPIO_PIN_7

#define LED1_PERIPHERAL						SYSCTL_PERIPH_GPIOA
#define LED1_PORT							GPIO_PORTA_BASE
#define LED1_PIN							GPIO_PIN_4
#define LED1_ON()							GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN)
#define LED1_OFF()							GPIOPinWrite(LED1_PORT, LED1_PIN, 0x00)

#define LED2_PERIPHERAL						SYSCTL_PERIPH_GPIOC
#define LED2_PORT							GPIO_PORTC_BASE
#define LED2_PIN							GPIO_PIN_4
#define LED2_ON()							GPIOPinWrite(LED2_PORT, LED2_PIN, LED2_PIN)
#define LED2_OFF()							GPIOPinWrite(LED2_PORT, LED2_PIN, 0x00)

#define LED3_PERIPHERAL						SYSCTL_PERIPH_GPIOA
#define LED3_PORT							GPIO_PORTA_BASE
#define LED3_PIN							GPIO_PIN_6
#define LED3_ON()							GPIOPinWrite(LED3_PORT, LED3_PIN, LED3_PIN)
#define LED3_OFF()							GPIOPinWrite(LED3_PORT, LED3_PIN, 0x00)
typedef enum
{
	SYSTEM_POWER_UP = 0,
	SYSTEM_INITIALIZE,
	SYSTEM_CALIB_SENSOR,
	SYSTEM_SAVE_CALIB_SENSOR,
	SYSTEM_ESTIMATE_MOTOR_MODEL,
	SYSTEM_SAVE_MOTOR_MODEL,
	SYSTEM_WAIT_TO_RUN,
	SYSTEM_RUN_SOLVE_MAZE,
	SYSTEM_RUN_IMAGE_PROCESSING,
	SYSTEM_ERROR
} SYSTEM_STATE;

extern void Config_System(void);
extern void BattSense_init(void);
extern void LED_Display_init(void);
extern SYSTEM_STATE system_GetState(void);
extern void system_SetState(SYSTEM_STATE SysState);
extern void system_Process_System_State(void);
extern uint32_t u32_UsrSystemClockGet(void);

#endif /* SYSTEMCONFIG_H_ */
