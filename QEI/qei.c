/*
 * qei.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "qei.h"

static void QEI0_VelocityIsr(void);
static void QEI1_VelocityIsr(void);

static bool qei_velocity_timeout[2];
static int32_t qei_velocity[2] = {0, 0};

void qei_init(uint16_t ms_Timebase)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
    		| QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP, 0xFFFFFFFF);
    ROM_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    ROM_GPIOPinConfigure(GPIO_PD6_PHA0);
    ROM_GPIOPinConfigure(GPIO_PD7_PHB0);
    ROM_QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, ROM_SysCtlClockGet() * ms_Timebase/ 1000);
    ROM_QEIVelocityEnable(QEI0_BASE);
    ROM_QEIEnable(QEI0_BASE);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
    		| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 0xFFFFFFFF);
    ROM_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
    ROM_GPIOPinConfigure(GPIO_PC5_PHA1);
    ROM_GPIOPinConfigure(GPIO_PC6_PHB1);
    ROM_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, ROM_SysCtlClockGet() * ms_Timebase/ 1000);
    ROM_QEIVelocityEnable(QEI1_BASE);
    ROM_QEIEnable(QEI1_BASE);

    ROM_QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
    ROM_QEIIntEnable(QEI1_BASE, QEI_INTTIMER);

    QEIIntRegister(QEI0_BASE, &QEI0_VelocityIsr);
    QEIIntRegister(QEI1_BASE, &QEI1_VelocityIsr);

    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);
    ROM_GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);
}

static void QEI0_VelocityIsr(void)
{
	ROM_QEIIntClear(QEI0_BASE, ROM_QEIIntStatus(QEI0_BASE, true));
	qei_velocity[0] = ROM_QEIVelocityGet(QEI0_BASE) * ROM_QEIDirectionGet(QEI0_BASE);
	qei_velocity_timeout[0] = true;
}

static void QEI1_VelocityIsr(void)
{
	ROM_QEIIntClear(QEI1_BASE, ROM_QEIIntStatus(QEI1_BASE, true));
	qei_velocity[1] = ROM_QEIVelocityGet(QEI1_BASE) * ROM_QEIDirectionGet(QEI1_BASE);
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
int32_t qei_getPosLeft()
{
	return ROM_QEIPositionGet(QEI0_BASE);
}
int32_t qei_getPosRight()
{
	return ROM_QEIPositionGet(QEI1_BASE);
}
void qei_setPosLeft(int32_t pos)
{
	ROM_QEIPositionSet(QEI0_BASE,pos);
}
void qei_setPosRight(int32_t pos)
{
	ROM_QEIPositionSet(QEI1_BASE,pos);
}
