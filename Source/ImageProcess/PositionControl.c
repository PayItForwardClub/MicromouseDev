/*
 * PositionControl.c
 *
 *  Created on: Jul 15, 2013
 *      Author: Admin
 */
#include "include.h"

#define POSITION_TIMER_PERIOD_MS		10

static void position_Control_Stoptimeout(void);
static void position_Control_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);
static void position_timer_ISR(void);

static TIMER_ID position_control_TimerID = INVALID_TIMER_ID;
static int32_t aui32_position[2] = {0, 0};
static bool b_timeout = false;

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
	aui32_position[0] = qei_Get_Position(false);
	aui32_position[1] = qei_Get_Position(true);
	b_timeout = true;
}

void position_Control_Process(void)
{
	if (b_timeout)
	{
		b_timeout = false;
		
	}
}


