/*
 * IR.c
 *
 *  Created on: Jul 4, 2015
 *      Author: NHH
 */
#include "../include.h"
#include "IR.h"

static uint8_t ADC_Step = 0;
static IR_CALIB_VALUE ir_calib_values;

static int32_t IR_Result[4];
static uint32_t IR_ResultTmp[4];
static void IR_Detector_ISR(void);
static void IR_Timer_Timeout(void);
static void ir_Stoptimeout(void);
static TIMER_ID ir_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);

static TIMER_ID ir_TimerID = INVALID_TIMER_ID;

void IRDetector_init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 32);

	ROM_ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH0 | ADC_CTL_IE);	//PE3, enable interrupt

	ROM_ADCSequenceEnable(ADC0_BASE, 2);
 	ADCIntRegister(ADC0_BASE, 2, &IR_Detector_ISR);
 	ROM_ADCIntEnable(ADC0_BASE, 2);


 	ADC_Step = 0;
 	TURN_ON_IRD1();
 	ir_Runtimeout(&IR_Timer_Timeout, 1);
}

static void IR_Detector_ISR(void)
{
	volatile uint32_t ADCResult;
	ROM_ADCIntClear(ADC0_BASE, 2);
	ROM_ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t *)&ADCResult);

	ADC_Step++;
	ADC_Step %= 8;

	switch (ADC_Step)
	{
		case 0:
			IR_Result[3] = -ADCResult+IR_ResultTmp[3];
			TURN_ON_IRD1();
			break;
		case 1:
			IR_ResultTmp[0] = ADCResult;
			TURN_OFF_IRD1();
			break;
		case 2:
			IR_Result[0] = -ADCResult+IR_ResultTmp[0];
			TURN_ON_IRD2();
			break;
		case 3:
			IR_ResultTmp[1] = ADCResult;
			TURN_OFF_IRD2();
			break;
		case 4:
			IR_Result[1] = -ADCResult+IR_ResultTmp[1];
			TURN_ON_IRD3();
			break;
		case 5:
			IR_ResultTmp[2] = ADCResult;
			TURN_OFF_IRD3();
			break;
		case 6:
			IR_Result[2] = -ADCResult+IR_ResultTmp[2];
			TURN_ON_IRD4();
			break;
		case 7:
			IR_ResultTmp[3] = ADCResult;
			TURN_OFF_IRD4();
			break;


		default:
			//Code should never reach this statement
			ADC_Step = 0;
			TURN_ON_IRD1();

			break;
	}
 	ir_Runtimeout(&IR_Timer_Timeout, 1);
}

static void IR_Timer_Timeout(void)
{
	ir_TimerID = INVALID_TIMER_ID;
	switch (ADC_Step)
	{
		case 0:
		case 1:
			ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH3 | ADC_CTL_IE);
			break;
		case 2:
		case 3:
			ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH2 | ADC_CTL_IE);
			break;
		case 4:
		case 5:
			ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH1 | ADC_CTL_IE);
			break;
		case 6:
		case 7:
			ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH0 | ADC_CTL_IE);
			break;
		default:
			ADC_Step = 0;
			ROM_ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH3 | ADC_CTL_IE);
			break;
	}
	ROM_ADCProcessorTrigger(ADC0_BASE, 2);

}

uint32_t IR_GetIrDetectorValue(uint8_t Select)
{
	if (Select > 3)
		Select = 3;
	return IR_Result[3 - Select];
}
void IR_load_calib_value(int* IRData)
{
	ir_calib_values.BaseFrontLeft=IRData[0];
	bluetooth_print("IR_CALIB_BASE_FRONT_LEFT: %d\r\n", ir_calib_values.BaseFrontLeft);
	ir_calib_values.BaseFrontRight=IRData[1];
	bluetooth_print("IR_CALIB_BASE_FRONT_RIGHT: %d\r\n", ir_calib_values.BaseFrontRight);
	ir_calib_values.BaseLeft=IRData[2];
	bluetooth_print("IR_CALIB_BASE_LEFT: %d\r\n", ir_calib_values.BaseLeft);
	ir_calib_values.BaseRight=IRData[3];
	bluetooth_print("IR_CALIB_BASE_RIGHT: %d\r\n", ir_calib_values.BaseRight);
	ir_calib_values.MaxFrontLeft=IRData[4];
	bluetooth_print("IR_CALIB_MAX_FRONT_LEFT: %d\r\n", ir_calib_values.MaxFrontLeft);
	ir_calib_values.MaxFrontRight=IRData[5];
	bluetooth_print("IR_CALIB_MAX_FRONT_RIGHT: %d\r\n", ir_calib_values.MaxFrontRight);
	ir_calib_values.MaxLeft=IRData[6];
	bluetooth_print("IR_CALIB_MAX_LEFT: %d\r\n", ir_calib_values.MaxLeft);
	ir_calib_values.MaxRight=IRData[7];
	bluetooth_print("IR_CALIB_MAX_RIGHT: %d\r\n", ir_calib_values.MaxRight);
}
bool IR_set_calib_value(IR_CALIB select)
{
	switch (select)
	{
		case IR_CALIB_BASE_FRONT_LEFT:
			ir_calib_values.BaseFrontLeft = IR_GetIrDetectorValue(0);
			bluetooth_print("IR_CALIB_BASE_FRONT_LEFT: %d\r\n", ir_calib_values.BaseFrontLeft);
			break;
		case IR_CALIB_BASE_FRONT_RIGHT:
			ir_calib_values.BaseFrontRight = IR_GetIrDetectorValue(3);
			bluetooth_print("IR_CALIB_BASE_FRONT_RIGHT: %d\r\n", ir_calib_values.BaseFrontRight);
			break;
		case IR_CALIB_BASE_LEFT:
			ir_calib_values.BaseLeft = IR_GetIrDetectorValue(1);
			bluetooth_print("IR_CALIB_BASE_LEFT: %d\r\n", ir_calib_values.BaseLeft);
			break;
		case IR_CALIB_BASE_RIGHT:
			ir_calib_values.BaseRight = IR_GetIrDetectorValue(2);
			bluetooth_print("IR_CALIB_BASE_RIGHT: %d\r\n", ir_calib_values.BaseRight);
			break;
		case IR_CALIB_MAX_FRONT_LEFT:
			ir_calib_values.MaxFrontLeft = IR_GetIrDetectorValue(0);
			bluetooth_print("IR_CALIB_MAX_FRONT_LEFT: %d\r\n", ir_calib_values.MaxFrontLeft);
			break;
		case IR_CALIB_MAX_FRONT_RIGHT:
			ir_calib_values.MaxFrontRight = IR_GetIrDetectorValue(3);
			bluetooth_print("IR_CALIB_MAX_FRONT_RIGHT: %d\r\n", ir_calib_values.MaxFrontRight);
			break;
		case IR_CALIB_MAX_LEFT:
			ir_calib_values.MaxLeft = IR_GetIrDetectorValue(1);
			bluetooth_print("IR_CALIB_MAX_LEFT: %d\r\n", ir_calib_values.MaxLeft);
			break;
		case IR_CALIB_MAX_RIGHT:
			ir_calib_values.MaxRight = IR_GetIrDetectorValue(2);
			bluetooth_print("IR_CALIB_MAX_RIGHT: %d\r\n", ir_calib_values.MaxRight);
			break;
		default:
			return false;
	}
	return true;
}

uint32_t IR_get_calib_value(IR_CALIB select)
{
	switch (select)
	{
		case IR_CALIB_BASE_FRONT_LEFT:
			return ir_calib_values.BaseFrontLeft;
		case IR_CALIB_BASE_FRONT_RIGHT:
			return ir_calib_values.BaseFrontRight;
		case IR_CALIB_BASE_LEFT:
			return ir_calib_values.BaseLeft;
		case IR_CALIB_BASE_RIGHT:
			return ir_calib_values.BaseRight;
		case IR_CALIB_MAX_FRONT_LEFT:
			return ir_calib_values.MaxFrontLeft;
		case IR_CALIB_MAX_FRONT_RIGHT:
			return ir_calib_values.MaxFrontRight;
		case IR_CALIB_MAX_LEFT:
			return ir_calib_values.MaxLeft;
		case IR_CALIB_MAX_RIGHT:
			return ir_calib_values.MaxRight;
		default:
			return 0;
	}
}

static void ir_Stoptimeout(void)
{
	if (ir_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(ir_TimerID);
	ir_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID ir_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	ir_Stoptimeout();
	ir_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
	return ir_TimerID;
}
