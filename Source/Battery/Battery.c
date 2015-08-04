/*
 * Bettery.c
 *
 *  Created on: Jul 8, 2015
 *      Author: NHH
 */

#include "include.h"
#include "Battery.h"

void BattSenseTimerTimeout(void);
void BattSenseISR(void);
static void battery_Stoptimeout(void);
static void battery_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);

static TIMER_ID battery_TimerID = INVALID_TIMER_ID;

void BattSense_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
//	ADCHardwareOversampleConfigure(ADC1_BASE, 64);

//	ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
//	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_END | ADC_CTL_CH10 | ADC_CTL_IE);
//	ADCSequenceEnable(ADC1_BASE, 3);
// 	ADCIntRegister(ADC1_BASE, 3, &BattSenseISR);
// 	ADCIntEnable(ADC1_BASE, 3);
//	IntEnable(INT_ADC1SS3);
	vIR_BatteryIntRegister(&BattSenseISR);
 	battery_Runtimeout(&BattSenseTimerTimeout, 1000);
}


void BattSenseTimerTimeout(void)
{
	battery_TimerID = INVALID_TIMER_ID;
//	ADCProcessorTrigger(ADC1_BASE, 3);
	vIR_BattSenseTrigger();
}

volatile float BatteryVoltage = 0;
void BattSenseISR(void)
{
	uint32_t ADCResult;
//	ADCIntClear(ADC1_BASE, 3);
	battery_Runtimeout(&BattSenseTimerTimeout, 10000);
//	ADCSequenceDataGet(ADC1_BASE, 3, (uint32_t *)&ADCResult);
	ADCResult = vIR_Battery_ADCGet();
	BatteryVoltage = ((float)ADCResult) / 4096 * (float)3.3 * (100 + 470) / 100;

	if (BatteryVoltage < (float)7.4)
	{
		buzzer_low_battery_shutdown();
		//shutdown robot here to protect battery

	}
	else if (BatteryVoltage < (float)7.6)
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
