/*
 * speed_control.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "include.h"
#include "speed_control.h"

static real_T Theta[4] = {-1, 1, 1, 1}, Theta_[4] = {-1, 1, 1, 1};
static real_T Theta2[4] = {-1, 1, 1, 1}, Theta2_[4] = {-1, 1, 1, 1};
static int32_t SetPoint[2] = {0, 0};
static int32_t RealSpeedSet[2] = {0, 0};
static real_T udk = 0;
static TIMER_ID speed_control_timID = INVALID_TIMER_ID;

static void Config_PWM(void);
//static void SetPWM(uint32_t ulBaseAddr, uint32_t ulTimer, uint32_t ulFrequency, int32_t ucDutyCycle);
static void speed_update_setpoint(void);
static void speed_control_runtimeout(uint32_t ms);
static void speed_control_stoptimeout(void);

void speed_control_init(void)
{
	float buf[64/4];
	qei_init(20);
	Control_initialize();
	Config_PWM();
	SetPWM(PWM_MOTOR_LEFT, DEFAULT, 0);
	SetPWM(PWM_MOTOR_RIGHT, DEFAULT, 0);
	
	param_Get(PARAM_ID_STR_INFO, (uint8_t *)buf);
	
	Theta[0] = buf[0];
	Theta[1] = buf[1];
	Theta[2] = buf[2];
	Theta[3] = buf[3];

	Theta_[0] = buf[4];
	Theta_[1] = buf[5];
	Theta_[2] = buf[6];
	Theta_[3] = buf[7];

	Theta2[0] = buf[8];
	Theta2[1] = buf[9];
	Theta2[2] = buf[10];
	Theta2[3] = buf[11];

	Theta2_[0] = buf[12];
	Theta2_[1] = buf[13];
	Theta2_[2] = buf[14];
	Theta2_[3] = buf[15];
	
	if (Theta[2] == 0)
	{
		Theta[2] = 1;
	}
	if (Theta2[2] == 0)
	{
		Theta2[2] = 1;
	}
}

void speed_control_save_STR(void)
{
	float buf[64/4];
	
	buf[0] = Theta[0];
	buf[1] = Theta[1];
	buf[2] = Theta[2];
	buf[3] = Theta[3];

	buf[4] = Theta_[0];
	buf[5] = Theta_[1];
	buf[6] = Theta_[2];
	buf[7] = Theta_[3];
	
	buf[8] = Theta2[0];
	buf[9] = Theta2[1];
	buf[10] = Theta2[2];
	buf[11] = Theta2[3];

	buf[12] = Theta2_[0];
	buf[13] = Theta2_[1];
	buf[14] = Theta2_[2];
	buf[15] = Theta2_[3];
	
	param_Set(PARAM_ID_STR_INFO, (uint8_t *) buf);
}

void ProcessSpeedControl(void)
{
	int32_t Velocity[2];
//	SetPoint = 250;
	if (qei_getVelocity(0, &Velocity[0]) == true)
	{
		if (RealSpeedSet[0] != 0)
		{
			udk = STR_Indirect(MOTOR_RIGHT, Theta, RealSpeedSet[0], Velocity[0]);
			SetPWM(PWM_MOTOR_RIGHT, DEFAULT, udk);
			Uocluong(udk, Velocity[0], Theta, Theta_);
		}
		else
		{
			SetPWM(PWM_MOTOR_RIGHT, DEFAULT, 0);
		}
#ifdef _DEBUG_SPEED_
		bluetooth_print("Right: %d\r\n", Velocity[0]);
#endif
	}
	if (qei_getVelocity(1, &Velocity[1]) == true)
	{
		if (RealSpeedSet[1] != 0)
		{
			udk = STR_Indirect(MOTOR_LEFT, Theta2, RealSpeedSet[1], Velocity[1]);
			SetPWM(PWM_MOTOR_LEFT, DEFAULT, udk);
			Uocluong2(udk, Velocity[1], Theta2, Theta2_);
		}
		else
		{
			SetPWM(PWM_MOTOR_LEFT, DEFAULT, 0);
		}
#ifdef _DEBUG_SPEED_
		bluetooth_print("Left: %d\r\n", Velocity[1]);
#endif
	}
}

static void Config_PWM(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB6_T0CCP0);
	GPIOPinConfigure(GPIO_PB2_T3CCP0);
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_6);

	// Configure timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER0_BASE, TIMER_A, DEFAULT);
	TimerMatchSet(TIMER0_BASE, TIMER_A, DEFAULT); // PWM
	TimerEnable(TIMER0_BASE, TIMER_A);

	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	TimerLoadSet(TIMER3_BASE, TIMER_A, DEFAULT);
	TimerMatchSet(TIMER3_BASE, TIMER_A, DEFAULT); // PWM
	TimerControlLevel(TIMER3_BASE, TIMER_A, true);
	TimerEnable(TIMER3_BASE, TIMER_A);

	SysCtlPeripheralEnable(DRV_ENABLE_LEFT_CHN_PERIPHERAL);
	SysCtlPeripheralEnable(DRV_ENABLE_RIGHT_CHN_PERIPHERAL);
	GPIOPinTypeGPIOOutput(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN);
	GPIOPinTypeGPIOOutput(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN);
	GPIOPinWrite(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN, 0);
	GPIOPinWrite(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN, 0);
}

void speed_Enable_Hbridge(bool Enable)
{
	if (Enable)
	{
		GPIOPinWrite(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN, DRV_ENABLE_LEFT_CHN_PIN);
		GPIOPinWrite(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN, DRV_ENABLE_RIGHT_CHN_PIN);
	}
	else
	{
		GPIOPinWrite(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN, 0);
		GPIOPinWrite(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN, 0);
	}
}

void SetPWM(uint32_t ulBaseAddr, uint32_t ulTimer, uint32_t ulFrequency, int32_t ucDutyCycle)
{
	uint32_t ulPeriod;
	ulPeriod = u32_UsrSystemClockGet() / ulFrequency;
	TimerLoadSet(ulBaseAddr, ulTimer, ulPeriod);
	if (ucDutyCycle > 90)
		ucDutyCycle = 90;
	else if (ucDutyCycle < -90)
		ucDutyCycle = -90;
	TimerMatchSet(ulBaseAddr, ulTimer, (100 + ucDutyCycle) * ulPeriod / 200 - 1);
}

void speed_set(MOTOR_SELECT Select, int32_t speed)
{
	if (Select == MOTOR_RIGHT)
	{
		if (SetPoint[0] != speed)
		{
			speed_control_runtimeout(20);
			SetPoint[0] = speed;
		}
	}
	else if (Select == MOTOR_LEFT)
	{
		if (SetPoint[1] != speed)
		{
			speed_control_runtimeout(20);
			SetPoint[1] = speed;
		}
	}
	
}

static void speed_update_setpoint(void)
{
	int i;

	speed_control_timID = INVALID_TIMER_ID;

	for (i = 0; i < 2; i++)
	{
		if ((RealSpeedSet[i] + 20 < SetPoint[i]) && (SetPoint[i] > 0))
			RealSpeedSet[i] += 20;
		else if ((RealSpeedSet[i] > SetPoint[i] + 20) && (SetPoint[i] < 0))
			RealSpeedSet[i] -= 20;
		else
			RealSpeedSet[i] = SetPoint[i];
	}

	speed_control_runtimeout(20);
}

static void speed_control_runtimeout(uint32_t ms)
{
	speed_control_stoptimeout();
	speed_control_timID = TIMER_RegisterEvent(&speed_update_setpoint, ms);
}

static void speed_control_stoptimeout(void)
{
	if (speed_control_timID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(speed_control_timID);
	speed_control_timID = INVALID_TIMER_ID;
}

void speed_SetMotorModel(MOTOR_SELECT select, real_T Theta[4])
{
	int i;
	if (select == MOTOR_RIGHT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta_[i] = Theta[i];
		}
	}
	else if (select == MOTOR_LEFT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta2_[i] = Theta[i];
		}
	}
}

void speed_GetMotorModel(MOTOR_SELECT select, real_T Theta[4])
{
	int i;
	if (select == MOTOR_RIGHT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta[i] = Theta_[i];
		}
	}
	else if (select == MOTOR_LEFT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta[i] = Theta2_[i];
		}
	}
}
