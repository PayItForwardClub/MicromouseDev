/*
 * SystemConfig.c
 *
 *  Created on: Jul 15, 2013
 *      Author: Admin
 */
#include "include.h"
#include "inc/hw_gpio.h"
#include "driverlib/systick.h"

static void SysTickIntHandle(void);
static void system_SystickConfig(uint32_t ui32_msInterval);

static SYSTEM_STATE e_SystemState = SYSTEM_POWER_UP;
static SYSTEM_STATE e_SystemState_ = SYSTEM_POWER_UP;
static uint32_t ms_Tickcount = 0;
static uint32_t Sysclock;

void Config_System(void)
{
	
	FPUEnable();
	FPULazyStackingEnable();
	// Config clock
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	Sysclock=80000000;
	system_SystickConfig(1);
	//
	SysCtlPeripheralEnable(BOOST_ENABLE_PREIPHERAL);
	GPIOPinTypeGPIOOutput(BOOST_ENABLE_PORT, BOOST_ENABLE_PIN);
	GPIOPadConfigSet(BOOST_ENABLE_PORT, BOOST_ENABLE_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPinWrite(BOOST_ENABLE_PORT, BOOST_ENABLE_PIN, 0xff);
}

uint32_t u32_UsrSystemClockGet(void)
{
	return Sysclock;
}

static void system_SystickConfig(uint32_t ui32_msInterval)
{
	SysTickPeriodSet(u32_UsrSystemClockGet() * ui32_msInterval / 1000);
	SysTickIntRegister(&SysTickIntHandle);
	SysTickIntEnable();
	SysTickEnable();
}

static void SysTickIntHandle(void)
{
	ms_Tickcount++;
}

void system_Enable_BoostCircuit(bool Enable)
{
	if (Enable)
	{
		GPIOPinWrite(BOOST_ENABLE_PORT, BOOST_ENABLE_PIN, 0xff);
	}
	else
	{
		GPIOPinWrite(BOOST_ENABLE_PORT, BOOST_ENABLE_PIN, 0x00);
	}
}

void LED_Display_init(void)
{
	SysCtlPeripheralEnable(LED1_PERIPHERAL);
	SysCtlPeripheralEnable(LED2_PERIPHERAL);
	SysCtlPeripheralEnable(LED3_PERIPHERAL);
	GPIOPinTypeGPIOOutput(LED1_PORT, LED1_PIN);
	GPIOPinTypeGPIOOutput(LED2_PORT, LED2_PIN);
	GPIOPinTypeGPIOOutput(LED3_PORT, LED3_PIN);

	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
}

SYSTEM_STATE system_GetState(void)
{
	return e_SystemState;
}

void system_SetState(SYSTEM_STATE SysState)
{
	e_SystemState = SysState;
}

void system_Process_System_State(void)
{
//	ProcessSpeedControl();
	switch (system_GetState())
	{
		case SYSTEM_POWER_UP:
			break;
		case SYSTEM_INITIALIZE:
			break;
		case SYSTEM_CALIB_SENSOR:
			break;
		case SYSTEM_SAVE_CALIB_SENSOR:
			break;
		case SYSTEM_ESTIMATE_MOTOR_MODEL:
			ProcessSpeedControl();
			break;
		case SYSTEM_SAVE_MOTOR_MODEL:
			break;
		case SYSTEM_WAIT_TO_RUN:
			break;
		case SYSTEM_RUN_SOLVE_MAZE:
			pid_Wallfollow_process();
			ProcessSpeedControl();
			break;
		case SYSTEM_RUN_IMAGE_PROCESSING:
			ProcessSpeedControl();
			break;
		case SYSTEM_ERROR:
			speed_Enable_Hbridge(false);
			system_Enable_BoostCircuit(false);
			IntMasterDisable();
			while (1)
			{
				LED1_ON();
				LED2_ON();
				LED3_ON();
				SysCtlDelay(u32_UsrSystemClockGet() / 3);
				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
				SysCtlDelay(u32_UsrSystemClockGet() / 3);
			}
			break;
	}
}
