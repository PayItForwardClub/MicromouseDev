/*
 * HostComm.c
 *
 *  Created on: Jul 14, 2015
 *      Author: Huan
 */

#include "../include.h"

#define START_BYTE 0xFF
#define UPDATE_TIME_MS 50
#define MAX_MSG_LEN_BYTE 20

#define SET_PID_PARAMS_CMD 1
#define SET_MAZE_ALGORITHM_CMD 2

#define PID_PARAMS_SCALE 100000000

extern uint8_t IR_Calib_Step;
extern PID_PARAMETERS pid_wall;

static TIMER_ID HostComm_TimerID = INVALID_TIMER_ID;
static bool HostCommFlag = false;

static int32_t batteryVoltage;
static SYSTEM_STATE state;

static int32_t rcvMsgByte=0;
static uint8_t rcvMsg[MAX_MSG_LEN_BYTE];
static int32_t rcvMsgLen=0;

static uint8_t data[MAX_MSG_LEN_BYTE];
static int32_t len;

static void HostCommTimeoutCallBack(void)
{
	HostComm_TimerID = INVALID_TIMER_ID;
	HostCommFlag = true;
	if (HostComm_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(HostComm_TimerID);
	HostComm_TimerID = TIMER_RegisterEvent(&HostCommTimeoutCallBack, UPDATE_TIME_MS);
}
void HostCommInit()
{
	bluetooth_init(115200);
	if (HostComm_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(HostComm_TimerID);
	HostComm_TimerID = TIMER_RegisterEvent(&HostCommTimeoutCallBack, UPDATE_TIME_MS);

}
void HostComm_process(void)
{
	if (HostCommFlag)
	{
		//LED1_TOGGLE();
		HostCommFlag = false;

		//SENDING: sending frame=1 START BYTE + 4 BATT_VOLT BYTES + 1 STATE BYTE + N DATA BYTES
		data[0]=START_BYTE;

		batteryVoltage =  (int32_t)(GetBatteryVoltage()*100);
		data[2]=batteryVoltage>>24;
		data[3]=batteryVoltage>>16;
		data[4]=batteryVoltage>>8;
		data[5]=batteryVoltage;

		state = system_GetState();
		data[6]=state;
		switch(state)
		{
		case SYSTEM_CALIB_SENSOR:
		{
			data[7]= IR_Calib_Step;
			len=8;
			break;
		}
		case SYSTEM_RUN_SOLVE_MAZE:
		{
			int32_t PIDError;
			WALL_FOLLOW_SELECT wallFollowSel;
			wallFollowSel = Get_Pid_Wallfollow();
			data[7]=(uint8_t)wallFollowSel;
			PIDError = (int32_t)pid_wall.e;
			data[8]=PIDError>>24;
			data[9]=PIDError>>16;
			data[10]=PIDError>>8;
			data[11]=PIDError;
			len=12;
			break;
		}
		default:
		{
			len=7;
		}
		}
		//data[len++]='\n';
		data[1]=len;
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
					switch (rcvMsg[1])
					{
					case SET_PID_PARAMS_CMD:
					{
						rcvMsgLen = 14;//N=12 (4 bytes Kp + 4 bytes Ki + 4 bytes Kd)

						break;
					}
					case SET_MAZE_ALGORITHM_CMD://N=1
					{
						rcvMsgLen = 3;
						break;
					}
					default:
						rcvMsgByte = 0;
					}
				}

				if ((rcvMsgByte==rcvMsgLen) && (rcvMsgLen != 0))
				{
					rcvMsgByte = 0;

					switch (rcvMsg[1])
					{
					case SET_PID_PARAMS_CMD:
					{
						//float Kp,Ki,Kd;
						//Kp=(rcvMsg[0]<<24|rcvMsg[1]<<16|rcvMsg[2]<<8|rcvMsg[3])*1.0/1000000000;
						//Ki=(rcvMsg[4]<<24|rcvMsg[5]<<16|rcvMsg[6]<<8|rcvMsg[7])*1.0/1000000000;
						//Kd=(rcvMsg[8]<<24|rcvMsg[9]<<16|rcvMsg[10]<<8|rcvMsg[11])*1.0/1000000000;
						//pid_set_k_params(Kp,Ki,Kd);
					    uint32_t Kp,Ki,Kd;
						Kp=(rcvMsg[2]<<24|rcvMsg[3]<<16|rcvMsg[4]<<8|rcvMsg[5]);
						Ki=(rcvMsg[6]<<24|rcvMsg[7]<<16|rcvMsg[8]<<8|rcvMsg[9]);
						Kd=(rcvMsg[10]<<24|rcvMsg[11]<<16|rcvMsg[12]<<8|rcvMsg[13]);

						//bluetooth_print("K %d %d %d\n",Kp,Ki,Kd);
						pid_set_k_params(&pid_wall,(float)Kp/PID_PARAMS_SCALE,
								(float)Ki/PID_PARAMS_SCALE,
								(float)Kd/PID_PARAMS_SCALE);
						break;
					}
					case SET_MAZE_ALGORITHM_CMD://N=1
					{
						pid_Wallfollow_set_follow((WALL_FOLLOW_SELECT)rcvMsg[2]);
						break;
					}
					}

				}
			}
		}
	}
}
