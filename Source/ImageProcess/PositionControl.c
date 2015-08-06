/*
 * PositionControl.c
 *
 *  Created on: Jul 15, 2013
 *      Author: Admin
 */
#include "include.h"
#include "Estimation_position.h"
#include "STR_Position.h"
#include <math.h>

#define POSITION_TIMER_PERIOD_MS		3

static void position_Control_Stoptimeout(void);
static void position_Control_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);
static void position_timer_ISR(void);

static TIMER_ID position_control_TimerID = INVALID_TIMER_ID;
static int32_t ai32_STRposition[2] = {0, 0};
static bool b_timeout = false;

static uint8_t au8_PositionProfile[2][2048];

static void position_Control_Stoptimeout(void)
{
	if (position_control_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(position_control_TimerID);
	position_control_TimerID = INVALID_TIMER_ID;
}

static void position_Control_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	position_Control_Stoptimeout();
	position_control_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
}

static void position_Control_Timer_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

	TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER5_BASE, TIMER_A, u32_UsrSystemClockGet() * POSITION_TIMER_PERIOD_MS/ 1000);	//Interval: TIMER_PERIOD_MS(ms)

	TimerIntRegister(TIMER5_BASE, TIMER_A, &position_timer_ISR);
	IntEnable(INT_TIMER5A);
	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	TimerControlStall(TIMER5_BASE, TIMER_A, false);
	TimerEnable(TIMER5_BASE, TIMER_A);
}

static void position_timer_ISR(void)
{
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	ai32_STRposition[0] = qei_Get_Position(false);
	ai32_STRposition[1] = qei_Get_Position(true);
	b_timeout = true;
}

void position_Control_Init(void)
{
	int i;
	Initial_Condition_Position();
	position_Control_Timer_Init();
	
	for (i = 0; i < 1024; i++)
	{
		au8_PositionProfile[0][i] = 100;
		au8_PositionProfile[1][i] = 50;
	}
	for (i = 1024; i < 2048; i++)
	{
		au8_PositionProfile[0][i] = 50;
		au8_PositionProfile[1][i] = 100;
	}
}

void position_Control_Process(void)
{
	static int32_t setpoint = 0, temp = 0;
	static int32_t max_pwm[2] = {0, 0};
	
//	ai32_SetPoint[0] = 50000;
	if (b_timeout)
	{
		b_timeout = false;
		Estimation_Algorithm(false);
		Model_Position(false);
		
		Estimation_Algorithm(true);
		Model_Position(true);
		
		if (abs(ai32_STRposition[0] - ai32_SetPoint1[0]) < 1500)
			max_pwm[0] = 5;
		else
			max_pwm[0] = 50;
		
		if (abs(ai32_STRposition[1] - ai32_SetPoint2[0]) < 1500)
			max_pwm[1] = 5;
		else
			max_pwm[1] = 50;

		if (adb_output1[0] > max_pwm[0])
			adb_output1[0] = max_pwm[0];
		else if (adb_output1[0] < -max_pwm[0])
			adb_output1[0] = -max_pwm[0];
		
		if (adb_output2[0] > max_pwm[1])
			adb_output2[0] = max_pwm[1];
		else if (adb_output2[0] < -max_pwm[1])
			adb_output2[0] = -max_pwm[1];
		
		SetPWM(PWM_MOTOR_RIGHT, DEFAULT, adb_output1[0]);
		SetPWM(PWM_MOTOR_LEFT, DEFAULT, adb_output2[0]);
		
		temp++;
		if ((temp > 1) && (setpoint < 2048))
		{
			temp = 0;
			if (setpoint < 2048)
			{
				ai32_SetPoint1[0] += -au8_PositionProfile[0][setpoint];
				ai32_SetPoint2[0] += -au8_PositionProfile[1][setpoint];
				setpoint++;
			}
		}
		
		if (temp > 500)
		{
			qei_Set_Position(false, 0);
			qei_Set_Position(true, 0);
			ai32_STRposition[0] = 0;
			ai32_STRposition[1] = 0;
			ai32_SetPoint1[0] = 0;
			ai32_SetPoint2[0] = 0;
			setpoint = 0;
		}
//		SetPWM(PWM_MOTOR_LEFT, DEFAULT, adb_output[0]);
	}
}


