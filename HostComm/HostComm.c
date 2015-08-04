/*
 * HostComm.c
 *
 *  Created on: Jul 14, 2015
 *      Author: Huan
 */

#include "../include.h"

#define UPDATE_TIME_MS 10
#define MSG_ROBOT_STATE_PERIOD 5//x UPDATE_TIME_MS
#define MAX_MSG_LEN_BYTE 40

#define START_BYTE 0xA5

#define SRC_ID 2//robot
#define DEST_ID 1//PC

#define MSG_PID_PARAMS 1
#define MSG_MAZE_ALGORITHM 2
#define MSG_POS_ANGLE 3
#define MSG_ROBOT_STATE 4
#define MSG_PING 5
#define MSG_SYNC_TIME 6

#define END_BYTE 0x0D

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

static uint32_t msgId=1;

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
	//0xa5 +length 2 +msg type 1+src id 1+dest id 1+msg id 1+payload+crc 2+0xd
	if (HostCommFlag)
	{
		//LED1_TOGGLE();
		HostCommFlag = false;

		data[0]=START_BYTE;

		data[3]=MSG_ROBOT_STATE;//payload=4 bytes bat volt + 1 byte state + N byte state data (MSB first)
		data[4]=SRC_ID;
		data[5]=DEST_ID;
		data[6]=msgId++;

		batteryVoltage =  (int32_t)(GetBatteryVoltage());
		data[7]=batteryVoltage>>24;
		data[8]=batteryVoltage>>16;
		data[9]=batteryVoltage>>8;
		data[10]=batteryVoltage;

		state = system_GetState();
		data[11]=state;
		switch(state)
		{
		case SYSTEM_CALIB_SENSOR:
		{
			data[12]=IR_Calib_Step;
			len=13;
			break;
		}
		case SYSTEM_RUN_SOLVE_MAZE:
		{
			int32_t PIDError;
			WALL_FOLLOW_SELECT wallFollowSel;
			wallFollowSel = Get_Pid_Wallfollow();
			data[12]=(uint8_t)wallFollowSel;
			PIDError = (int32_t)(pid_wall.e);
			data[13]=PIDError>>24;
			data[14]=PIDError>>16;
			data[15]=PIDError>>8;
			data[16]=PIDError;
			len=12;
			break;
		}
		default:
		{
			len=13;
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
					//					case SET_PID_PARAMS_CMD:
					//					{
					//						rcvMsgLen = 14;//N=12 (4 bytes Kp + 4 bytes Ki + 4 bytes Kd)
					//
					//						break;
					//					}
					//					case SET_MAZE_ALGORITHM_CMD://N=1
					//					{
					//						rcvMsgLen = 3;
					//						break;
					//					}
					//					default:
					//						rcvMsgByte = 0;
					//					}
					}

					if ((rcvMsgByte==rcvMsgLen) && (rcvMsgLen != 0))
					{
						rcvMsgByte = 0;

						switch (rcvMsg[1])
						{
						//					case SET_PID_PARAMS_CMD:
						//					{
						//						//float Kp,Ki,Kd;
						//						//Kp=(rcvMsg[0]<<24|rcvMsg[1]<<16|rcvMsg[2]<<8|rcvMsg[3])*1.0/1000000000;
						//						//Ki=(rcvMsg[4]<<24|rcvMsg[5]<<16|rcvMsg[6]<<8|rcvMsg[7])*1.0/1000000000;
						//						//Kd=(rcvMsg[8]<<24|rcvMsg[9]<<16|rcvMsg[10]<<8|rcvMsg[11])*1.0/1000000000;
						//						//pid_set_k_params(Kp,Ki,Kd);
						//					    uint32_t Kp,Ki,Kd;
						//						Kp=(rcvMsg[2]<<24|rcvMsg[3]<<16|rcvMsg[4]<<8|rcvMsg[5]);
						//						Ki=(rcvMsg[6]<<24|rcvMsg[7]<<16|rcvMsg[8]<<8|rcvMsg[9]);
						//						Kd=(rcvMsg[10]<<24|rcvMsg[11]<<16|rcvMsg[12]<<8|rcvMsg[13]);
						//
						//						//bluetooth_print("K %d %d %d\n",Kp,Ki,Kd);
						//						pid_set_k_params(&pid_wall,(float)Kp/PID_PARAMS_SCALE,
						//								(float)Ki/PID_PARAMS_SCALE,
						//								(float)Kd/PID_PARAMS_SCALE);
						//						break;
						//					}
						//					case SET_MAZE_ALGORITHM_CMD://N=1
						//					{
						//						pid_Wallfollow_set_follow((WALL_FOLLOW_SELECT)rcvMsg[2]);
						//						break;
						//					}
						}

					}
				}
			}
		}
	}
}
