#include "string.h"
#include "include.h"
#include "HostComm_ext.h"

static void HostCommHandler_HandshakeEvent(unsigned char *data, unsigned char size);
static void HostCommHandler_SpeedSet(unsigned char *data, unsigned char size);
static void HostCommHandler_RobotStart(unsigned char *data, unsigned char size);
static void HostCommHandler_RobotStop(unsigned char *data, unsigned char size);
static void HostCommHandler_SetPID(unsigned char *data, unsigned char size);
static void HostCommHandler_RobotSetSpeed(unsigned char *data, unsigned char size);
static void HostCommHandler_SetMode(unsigned char *data, unsigned char size);
static void HostCommHandler_SetLEDDisplay(unsigned char *data, unsigned char size);
static void HostCommHandler_DebounceButton(unsigned char *data, unsigned char size);
static void HostCommHandler_BL_Name(unsigned char *data, unsigned char size);
static void HostCommHandler_BL_Password(unsigned char *data, unsigned char size);
static void HostCommHandler_BL_Uart(unsigned char *data, unsigned char size);
static void vRobotStartTimeout(void);

static bool b_StartRequest = false;
TIMER_ID StartRobotTimID = INVALID_TIMER_ID;

void HostCommHandler_Init(void)
{
	HostComm_Dev_RegisterCallback(HANDSHAKE_CMD, &HostCommHandler_HandshakeEvent);
	HostComm_Dev_RegisterCallback(MSG_SPEED_SET_CMD, &HostCommHandler_SpeedSet);
	HostComm_Dev_RegisterCallback(ROBOT_START_CMD, &HostCommHandler_RobotStart);
	HostComm_Dev_RegisterCallback(ROBOT_STOP_CMD, &HostCommHandler_RobotStop);
	HostComm_Dev_RegisterCallback(SET_PID_CMD, &HostCommHandler_SetPID);
	HostComm_Dev_RegisterCallback(SET_SPEED_CMD, &HostCommHandler_RobotSetSpeed);
	HostComm_Dev_RegisterCallback(SET_MODE_CMD, &HostCommHandler_SetMode);
	HostComm_Dev_RegisterCallback(SET_LED_DISPLAY_CMD, &HostCommHandler_SetLEDDisplay);
	HostComm_Dev_RegisterCallback(SET_BUTTON_DEBOUNCE_CMD, &HostCommHandler_DebounceButton);
	HostComm_Dev_RegisterCallback(SET_BL_NAME_CMD, &HostCommHandler_BL_Name);
	HostComm_Dev_RegisterCallback(SET_BL_PASSWORD_CMD, &HostCommHandler_BL_Password);
	HostComm_Dev_RegisterCallback(SET_BL_UART_CMD, &HostCommHandler_BL_Uart);
}

static void HostCommHandler_HandshakeEvent(unsigned char *data, unsigned char size)
{
	static unsigned char data_rcv[100];
	
	memset(data_rcv, 0, 100);
	memcpy(data_rcv, data, size);
}

static void HostCommHandler_SpeedSet(unsigned char *data, unsigned char size)
{
	static unsigned char data_rcv[100];
	
	memset(data_rcv, 0, 100);
	memcpy(data_rcv, data, size);
}

static void HostCommHandler_RobotStart(unsigned char *data, unsigned char size)
{
	if (StartRobotTimID != INVALID_TIMER_ID)
	{
		TIMER_UnregisterEvent(StartRobotTimID);
	}
	StartRobotTimID = TIMER_RegisterEvent(&vRobotStartTimeout, 1000);
}

static void HostCommHandler_RobotStop(unsigned char *data, unsigned char size)
{
	if (StartRobotTimID != INVALID_TIMER_ID)
	{
		TIMER_UnregisterEvent(StartRobotTimID);
	}
	StartRobotTimID = INVALID_TIMER_ID;
	speed_Enable_Hbridge(false);
	speed_set(MOTOR_LEFT, 0);
	speed_set(MOTOR_RIGHT, 0);
}

static void HostCommHandler_SetPID(unsigned char *data, unsigned char size)
{
	static PID_PARAMETERS PidParam;
	float f_temp[5];
	
	if (size < 5 * 4)
		return;
	
	memcpy ((void *)&f_temp[0], data, 20);
	/*
	(float)Kp - (float)Ki - (float)Kd - (float)Ts - (float)Saturation
	*/
//	f_temp = (float *)(&data[0]);
	
	PidParam.Kp = f_temp[0];

//	f_temp = (float *)(&data[4]);
	PidParam.Ki = f_temp[1];

//	f_temp = (float *)(&data[8]);
	PidParam.Kd = f_temp[2];

//	f_temp = (float *)(&data[12]);
	PidParam.Ts = f_temp[3];

//	f_temp = (float *)(&data[16]);
	PidParam.PID_Saturation = f_temp[4];
	
	pid_set_parameters(PidParam);
}

static void HostCommHandler_RobotSetSpeed(unsigned char *data, unsigned char size)
{
	int16_t *speed;
	
	speed = (int16_t *)(data + 1);
	if (*data == 0)
		speed_set(MOTOR_LEFT, *speed);
	else
		speed_set(MOTOR_RIGHT, *speed);
}

static void HostCommHandler_SetMode(unsigned char *data, unsigned char size)
{
	system_SetState(data[0]);
}

static void HostCommHandler_SetLEDDisplay(unsigned char *data, unsigned char size)
{
	if ((data[0] & 0x01) != 0)
		LED1_ON();
	else
		LED1_OFF();

	if ((data[0] & 0x02) != 0)
		LED2_ON();
	else
		LED2_OFF();

	if ((data[0] & 0x04) != 0)
		LED3_ON();
	else
		LED3_OFF();
}

static void HostCommHandler_DebounceButton(unsigned char *data, unsigned char size)
{
	button_SetDebounceTime(data[0]);
}

static void HostCommHandler_BL_Name(unsigned char *data, unsigned char size)
{
	
}

static void HostCommHandler_BL_Password(unsigned char *data, unsigned char size)
{
	
}

static void HostCommHandler_BL_Uart(unsigned char *data, unsigned char size)
{
	
}

static void vRobotStartTimeout(void)
{
	StartRobotTimID = INVALID_TIMER_ID;
	speed_Enable_Hbridge(true);
}