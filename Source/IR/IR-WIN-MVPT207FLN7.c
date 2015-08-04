/*
 * IR.c
 *
 *  Created on: Jul 4, 2015
 *      Author: NHH
 */
#include "include.h"
#include "IR.h"

static uint8_t ADC_Step = 0;
static IR_CALIB_VALUE ir_calib_values;

static uint32_t IR_Result[4];
static void IR_Detector_ISR(void);
static void IR_Timer_Timeout(void);
static void ir_Stoptimeout(void);
static TIMER_ID ir_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime);
static void ir_SavetoParams(void);
static void ir_LoadfromParams(void);

static TIMER_ID ir_TimerID = INVALID_TIMER_ID;

void IRDetector_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	ADCHardwareOversampleConfigure(ADC0_BASE, 32);

	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH0 | ADC_CTL_IE);	//PE3, enable interrupt

	ADCSequenceEnable(ADC0_BASE, 2);
 	ADCIntRegister(ADC0_BASE, 2, &IR_Detector_ISR);
 	ADCIntEnable(ADC0_BASE, 2);

 	ADC_Step = 0;
 	TURN_ON_IRD1();
 	ir_Runtimeout(&IR_Timer_Timeout, 1);
	ir_LoadfromParams();
}

static void IR_Detector_ISR(void)
{
	volatile uint32_t ADCResult;
	ADCIntClear(ADC0_BASE, 2);
	ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t *)&ADCResult);
	ADC_Step++;
	ADC_Step %= 4;

	switch (ADC_Step)
	{
		case 0:
			IR_Result[3] = ADCResult;
			TURN_ON_IRD1();
			TURN_OFF_IRD4();
			break;
		case 1:
			IR_Result[0] = ADCResult;
			TURN_ON_IRD2();
			TURN_OFF_IRD1();
			break;
		case 2:
			IR_Result[1] = ADCResult;
			TURN_ON_IRD3();
			TURN_OFF_IRD2();
			break;
		case 3:
			IR_Result[2] = ADCResult;
			TURN_ON_IRD4();
			TURN_OFF_IRD3();
			break;
		default:
			//Code should never reach this statement
			ADC_Step = 0;
			TURN_ON_IRD1();
			TURN_OFF_IRD4();
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
			ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH3 | ADC_CTL_IE);
			break;
		case 1:
			ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH2 | ADC_CTL_IE);
			break;
		case 2:
			ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH1 | ADC_CTL_IE);
			break;
		case 3:
			ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH0 | ADC_CTL_IE);
			break;
		default:
			ADC_Step = 0;
			ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_END | ADC_CTL_CH3 | ADC_CTL_IE);
			break;
	}
	ADCProcessorTrigger(ADC0_BASE, 2);
}

uint32_t IR_GetIrDetectorValue(uint8_t Select)
{
	if (Select > 3)
		Select = 3;
	return IR_Result[3 - Select];
}

bool IR_set_calib_value(IR_CALIB select)
{
	switch (select)
	{
		case IR_CALIB_BASE_FRONT_LEFT:
			ir_calib_values.BaseFrontLeft = IR_GetIrDetectorValue(0);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_BASE_FRONT_LEFT: %d\r\n", ir_calib_values.BaseFrontLeft);
#endif
			break;
		case IR_CALIB_BASE_FRONT_RIGHT:
			ir_calib_values.BaseFrontRight = IR_GetIrDetectorValue(3);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_BASE_FRONT_RIGHT: %d\r\n", ir_calib_values.BaseFrontRight);
#endif
			break;
		case IR_CALIB_BASE_LEFT:
			ir_calib_values.BaseLeft = IR_GetIrDetectorValue(1);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_BASE_LEFT: %d\r\n", ir_calib_values.BaseLeft);
#endif
			break;
		case IR_CALIB_BASE_RIGHT:
			ir_calib_values.BaseRight = IR_GetIrDetectorValue(2);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_BASE_RIGHT: %d\r\n", ir_calib_values.BaseRight);
#endif
			break;
		case IR_CALIB_MAX_FRONT_LEFT:
			ir_calib_values.MaxFrontLeft = IR_GetIrDetectorValue(0);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_MAX_FRONT_LEFT: %d\r\n", ir_calib_values.MaxFrontLeft);
#endif
			break;
		case IR_CALIB_MAX_FRONT_RIGHT:
			ir_calib_values.MaxFrontRight = IR_GetIrDetectorValue(3);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_MAX_FRONT_RIGHT: %d\r\n", ir_calib_values.MaxFrontRight);
#endif
			ir_SavetoParams();
			break;
		case IR_CALIB_MAX_LEFT:
			ir_calib_values.MaxLeft = IR_GetIrDetectorValue(1);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_MAX_LEFT: %d\r\n", ir_calib_values.MaxLeft);
#endif
			break;
		case IR_CALIB_MAX_RIGHT:
			ir_calib_values.MaxRight = IR_GetIrDetectorValue(2);
#ifdef __DEBUG_PRINT__
			bluetooth_print("IR_CALIB_MAX_RIGHT: %d\r\n", ir_calib_values.MaxRight);
#endif
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

static void ir_SavetoParams(void)
{
	volatile uint8_t au8_buf[64];
	
	memcpy((void *)au8_buf, (void *)&ir_calib_values, sizeof(IR_CALIB_VALUE));
	
	param_Set(PARAM_ID_IR_INFO, (unsigned char *)au8_buf);
}

static void ir_LoadfromParams(void)
{
	volatile uint8_t au8_buf[64];
	
	param_Get(PARAM_ID_IR_INFO, (unsigned char *)au8_buf);
	
	memcpy((void *)&ir_calib_values, (void *)au8_buf, sizeof(IR_CALIB_VALUE));
}
