/*
 * qei.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "include.h"
#include "qei.h"

static void QEI0_VelocityIsr(void);
static void QEI1_VelocityIsr(void);

static bool qei_velocity_timeout[2];
static int32_t qei_velocity[2] = {0, 0};

void qei_init(uint16_t ms_Timebase)
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
    		| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 0xFFFFFFFF);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, u32_UsrSystemClockGet() * ms_Timebase/ 1000);
    QEIVelocityEnable(QEI0_BASE);
    QEIEnable(QEI0_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
    		| QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP, 0xFFFFFFFF);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, u32_UsrSystemClockGet() * ms_Timebase/ 1000);
    QEIVelocityEnable(QEI1_BASE);
    QEIEnable(QEI1_BASE);

    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    QEIIntEnable(QEI1_BASE, QEI_INTTIMER);

    QEIIntRegister(QEI0_BASE, &QEI0_VelocityIsr);
    QEIIntRegister(QEI1_BASE, &QEI1_VelocityIsr);

    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);
}

static void QEI0_VelocityIsr(void)
{
	QEIIntClear(QEI0_BASE, QEIIntStatus(QEI0_BASE, true));
	qei_velocity[0] = QEIVelocityGet(QEI0_BASE) * QEIDirectionGet(QEI0_BASE);
	qei_velocity_timeout[0] = true;
}

static void QEI1_VelocityIsr(void)
{
	QEIIntClear(QEI1_BASE, QEIIntStatus(QEI1_BASE, true));
	qei_velocity[1] = QEIVelocityGet(QEI1_BASE) * QEIDirectionGet(QEI1_BASE);
	qei_velocity_timeout[1] = true;
}

bool qei_getVelocity(bool Select, int32_t *Velocity)
{
	if (!Select)
	{
		if (qei_velocity_timeout[0])
		{
			*Velocity = qei_velocity[0];
			qei_velocity_timeout[0] = false;
			return true;
		}
		else
			return false;
	}
	else
		if (qei_velocity_timeout[1])
		{
			*Velocity = qei_velocity[1];
			qei_velocity_timeout[1] = false;
			return true;
		}
		else
			return false;
}

int32_t qei_Get_Position(bool Select)
{
	if (!Select)
	{
		return QEIPositionGet(QEI0_BASE);
	}
	else
	{
		return QEIPositionGet(QEI1_BASE);
	}
}

void qei_Set_Position(bool Select, int32_t position)
{
	if (!Select)
	{
		QEIPositionSet(QEI0_BASE, position);
	}
	else
	{
		QEIPositionSet(QEI1_BASE, position);
	}
}
