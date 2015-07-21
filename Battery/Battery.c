/*
 * Bettery.c
 *
 *  Created on: Jul 8, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "Battery.h"

void BattSenseTimerTimout(void);
void BattSenseISR(void);
static void battery_Stoptimeout(void);
static void battery_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);

static TIMER_ID battery_TimerID = INVALID_TIMER_ID;

void BattSense_init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	ROM_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
	ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64);

	ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_END | ADC_CTL_CH10 | ADC_CTL_IE);
	ROM_ADCSequenceEnable(ADC1_BASE, 3);
 	ADCIntRegister(ADC1_BASE, 3, &BattSenseISR);
 	ROM_ADCIntEnable(ADC1_BASE, 3);

 	battery_Runtimeout(&BattSenseTimerTimout, 10000);
}


void BattSenseTimerTimout(void)
{
	battery_TimerID = INVALID_TIMER_ID;
	ROM_ADCProcessorTrigger(ADC1_BASE, 3);

}

volatile float BatteryVoltage = 0;
void BattSenseISR(void)
{
	uint32_t ADCResult;
	ROM_ADCIntClear(ADC1_BASE, 3);
	battery_Runtimeout(&BattSenseTimerTimout, 10000);
	ROM_ADCSequenceDataGet(ADC1_BASE, 3, (uint32_t *)&ADCResult);
	BatteryVoltage = ((float)ADCResult) / 4096 * 3.3 * (100 + 470) / 100;

	if (BatteryVoltage < (float)7.6)
	{
		buzzer_low_battery_shutdown();
		//shutdown robot here to protect battery

	}
	else if (BatteryVoltage < (float)7.4)
	{
		//Notify user to shutdown robot
		buzzer_low_batterry_notify();
	}
}

static void battery_Stoptimeout(void)
{
	if (battery_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(battery_TimerID);
	battery_TimerID = INVALID_TIMER_ID;
}

static void battery_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	battery_Stoptimeout();
	battery_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
}
float GetBatteryVoltage()
{
	return BatteryVoltage;
}
