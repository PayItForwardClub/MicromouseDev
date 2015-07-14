/*
 * HostComm.c
 *
 *  Created on: Jul 14, 2015
 *      Author: Huan
 */

#include "../include.h"

#define START_BYTE 0xAA
#define UPDATE_TIME_MS 40
#define MAX_MSG_LEN_BYTE 20

#define SET_PID_PARAMS_CMD 1
#define SET_WALL_FOLLOW_CMD 2

static TIMER_ID HostComm_TimerID = INVALID_TIMER_ID;
static bool HostCommFlag = false;

static int32_t batteryVoltage;
static SYSTEM_STATE state;

static int32_t rcvMsgByte=0;
static uint8_t rcvMsg[MAX_MSG_LEN_BYTE];
static int32_t rcvMsgLen=0;

static void HostCommTimeoutCallBack(void)
{
	HostCommFlag = true;
	TIMER_RegisterEvent(&HostCommTimeoutCallBack, UPDATE_TIME_MS);
}
static void HostComm_Stoptimeout(void)
{
	if (HostComm_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(HostComm_TimerID);
	HostComm_TimerID = INVALID_TIMER_ID;
}
static void HostComm_Runtimeout(TIMER_CALLBACK_FUNC TimeoutCallback, uint32_t msTime)
{
	HostComm_Stoptimeout();
	HostComm_TimerID = TIMER_RegisterEvent(TimeoutCallback, msTime);
}
void HostCommInit()
{
	bluetooth_init(115200);
	HostComm_Runtimeout(HostCommTimeoutCallBack,UPDATE_TIME_MS);
}
void HostComm_process(void)
{
	if (HostCommFlag)
	{
		uint8_t data[MAX_MSG_LEN_BYTE];
		int32_t len;

		HostCommFlag = false;

		//SENDING: sending frame=1 START BYTE + 4 BATT_VOLT BYTES + 1 STATE BYTE + N DATA BYTES
		data[0]=START_BYTE;

		batteryVoltage =  (int32_t)(GetBatteryVoltage()*100);
		data[1]=batteryVoltage>>24;
		data[2]=batteryVoltage>>16;
		data[3]=batteryVoltage>>8;
		data[4]=batteryVoltage;

		state = system_GetState();
		data[5]=state;
		switch(state)
		{
		case SYSTEM_RUN_SOLVE_MAZE:
		{
			int32_t PIDError;
			WALL_FOLLOW_SELECT wallFollowSel;
			wallFollowSel = Get_Pid_Wallfollow();
			data[6]=(uint8_t)wallFollowSel;
			PIDError = (int32_t)(pid_get_error()*100);
			data[7]=PIDError>>24;
			data[8]=PIDError>>16;
			data[9]=PIDError>>8;
			data[10]=PIDError;
			len=11;
			break;
		}
		default:
		{
			len=6;
		}
		}

		bluetooth_send(data,len);


		//RECEIVING: rcv frame= 1 START BYTE + 1 CMD_ID BYTE + N DATA BYTES
		len=bluetooth_recv(data,MAX_MSG_LEN_BYTE,false);
		if (len)
		{
			int i;
			for (i=0;i<len;i++)
			{
				if (rcvMsgByte==0)
				{
					if (data[i]==START_BYTE)
					{
						rcvMsg[rcvMsgByte++] = data[i];
					}
					continue;
				}
				rcvMsg[rcvMsgByte++] = data[i];

				if (rcvMsgByte==2)
				{
					switch (rcvMsg[2])
					{
					case SET_PID_PARAMS_CMD:
					{
						rcvMsgLen = 14;//N=12 (4 bytes Kp + 4 bytes Ki + 4 bytes Kd)
						break;
					}
					case SET_WALL_FOLLOW_CMD://N=1
					{
						rcvMsgLen = 3;
						break;
					}
					}
				}

				if ((rcvMsgByte==rcvMsgLen) && (rcvMsgLen != 0))
				{
					rcvMsgByte = 0;
					switch (rcvMsg[2])
					{
					case SET_PID_PARAMS_CMD:
					{
						float Kp,Ki,Kd;
						Kp=(rcvMsg[0]<<24|rcvMsg[1]<<16|rcvMsg[2]<<8|rcvMsg[3])*1.0/1000000000;
						Ki=(rcvMsg[4]<<24|rcvMsg[5]<<16|rcvMsg[6]<<8|rcvMsg[7])*1.0/1000000000;
						Kd=(rcvMsg[8]<<24|rcvMsg[9]<<16|rcvMsg[10]<<8|rcvMsg[11])*1.0/1000000000;
						pid_set_k_params(Kp,Ki,Kd);
						break;
					}
					case SET_WALL_FOLLOW_CMD://N=1
					{
						pid_Wallfollow_set_follow((WALL_FOLLOW_SELECT)rcvMsg[3]);
						break;
					}
					}

				}
			}
		}
	}
}
