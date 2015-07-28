/*
 * speed_control.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef STR_SPEED_CONTROL_H_
#define STR_SPEED_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

#define PWM_MOTOR_LEFT			TIMER0_BASE, TIMER_A
#define PWM_MOTOR_RIGHT			TIMER3_BASE, TIMER_A

typedef enum
{
	MOTOR_LEFT = 0,
	MOTOR_RIGHT
} MOTOR_SELECT;

extern void speed_control_init(void);
extern void ProcessSpeedControl(void);
extern void speed_set(MOTOR_SELECT Select, int32_t speed);
extern void speed_Enable_Hbridge(bool Enable);
extern void speed_SetMotorModel(MOTOR_SELECT select, real_T Theta[4]);
extern void speed_GetMotorModel(MOTOR_SELECT select, real_T Theta[4]);

#endif /* STR_SPEED_CONTROL_H_ */
