#ifndef HOST_COMM_INT_H
#define HOST_COMM_INT_H

#include <stdint.h>
#include <stdbool.h>

#include "HostComm_ext.h"

#define MAX_DEV_RESP_LIST      		    20				//dev can receive n commands that require response from device
#define MAX_DEV_WAIT_RESP_LIST 		    10				//dev can send n events that require response from server 
#define MAX_HOSTCOMM_RX_BUF_SIZE			1024
#define MAX_HOSTCOMM_TX_BUF_SIZE			200
#define MAX_HOSTCOMM_EVENT_BUF_SIZE		512


#define HOSTCOMM_MSG_START_CODE		0xA5
#define HOSTCOMM_MSG_END_CODE			0x0D

#define HOSTCOMM_MSG_ID_SIZE			1
#define HOSTCOMM_DEST_ID_SIZE			1
#define HOSTCOMM_SRC_ID_SIZE			1
#define HOSTCOMM_DATE_TIME_SIZE		0
#define HOSTCOMM_MSG_CODE_SIZE		1
#define HOSTCOMM_MSG_HEADER_SIZE 	(HOSTCOMM_MSG_ID_SIZE + HOSTCOMM_DEST_ID_SIZE + HOSTCOMM_SRC_ID_SIZE + HOSTCOMM_DATE_TIME_SIZE + HOSTCOMM_MSG_CODE_SIZE)

#define MAX_OPEN_TCP_RETRY      4
#define SERVER_ID								0x00000001

static unsigned char HostComm_Timer_ID, HostComm_HC05_Timer_ID;
static bool HostComm_HC05_Timer_IsTimeout = false;
static unsigned char HostComm_DeviceId;
static unsigned char HostComm_ServerId;
static unsigned short HostComm_DevMsgId;        

static unsigned long HostComm_RxSize;
static unsigned char HostComm_TimeoutCounter;
static unsigned char HostComm_RxBuf[MAX_HOSTCOMM_RX_BUF_SIZE];
static unsigned char HostComm_TxBuf[MAX_HOSTCOMM_TX_BUF_SIZE];

typedef enum
{
	HOSTCOMM_IDLE_ACTION,
	HOSTCOMM_OPEN_TCP_SOCKET,
	HOSTCOMM_RESET_ACTION,
}HOSTCOMM_ACTIONS;
HOSTCOMM_ACTIONS HostComm_CurrentAction;

typedef struct
{
    unsigned char dst_id;
    unsigned char src_id;  
    unsigned long date_time;
    unsigned char msg_id;
    unsigned char msg_code;
    unsigned short msg_len;
    unsigned char * data_ptr;
    unsigned char  data_len;
}HOSTCOMM_SRV_MSG;

/*
  Define mapping between RESPONSE ID and RESPONSE CODE
*/
typedef struct
{
	unsigned short EventCode;
	HOSTCOMM_EVENT_LIST EventId;
    void (*ack_callback)(void);
	//can add this field to record if this event needs a response from server or not
	//currently, only HANDSHAKE, PING, KEEP_SOCKET_OPEN event need response, so I do not add this field
	//unsigned short CommandCode;	
}HOSTCOMM_DEV_EVENT_INFO;

HOSTCOMM_DEV_EVENT_INFO  HostComm_Dev_EventList[] = 
	{
		{0x0001,HANDSHAKE_EVENT,                    NULL},
		{0x0002,PING_EVENT,                         NULL},
		{0x0005,UPDATE_BOT_EVENT,										NULL},
	};

	
/*
  Define links between Server's commands and Dev's callbacks
*/
typedef struct
{
	unsigned short CommandId;
	unsigned short CommandCode;
	unsigned short ResponseCode;	//if not 0, then it will be an expected RespCode from Dev for a command
	void (*callback) (unsigned char *data, unsigned char size);
}HOST_COMMAND_CALLBACK;

//list of request callbacks for HostComm Device
HOST_COMMAND_CALLBACK   HostComm_DevCallbacks[] = {
		{HANDSHAKE_CMD, 					0x0021,		0, 			NULL},	
		{PING_CMD,								0x0022,		0, 			NULL},	
		{ROBOT_START_CMD, 				0x0023,		0, 			NULL},	
		{ROBOT_STOP_CMD,					0x0024,		0, 			NULL},	
		{SET_PID_CMD, 						0x0025,		0, 			NULL},	
		{SET_SPEED_CMD,						0x0026,		0, 			NULL},	
		{SET_MODE_CMD, 						0x0027,		0, 			NULL},	
		{SET_LED_DISPLAY_CMD,			0x0028,		0, 			NULL},	
		{SET_BUTTON_DEBOUNCE_CMD, 0x0029,		0, 			NULL},	
		{SET_BL_NAME_CMD,					0x002A,		0, 			NULL},	
		{SET_BL_PASSWORD_CMD, 		0x002B,		0, 			NULL},	
		{SET_BL_UART_CMD,					0x002C,		0, 			NULL},	
		{MSG_UPDATE_POS_CMD, 			0x0000,		0, 			NULL},
		{MSG_TIME_CMD, 						0x0001,		0, 			NULL},
		{MSG_BOX_CMD, 						0x0002,		0, 			NULL},
		{MSG_WAYPOINT_CMD, 				0x0003,		0, 			NULL},
		{MSG_TRACK_BOT_CMD, 			0x0004,		0, 			NULL},
		{MSG_UPDATE_BOT_CMD, 			0x0005,		0, 			NULL},
		{MSG_SPEED_SET_CMD, 			0x0006,		0, 			NULL},
		{MSG_START_TEST_CMD, 			0x0007,		0, 			NULL},
		{MSG_SET_PID_CMD, 				0x0008,		0, 			NULL},
	};


/*
 Define record structure used to record:
  - List of responses that dev should send for received commands
  - List of server's response that dev expects to receive after sending an event
*/	
typedef struct
{
  unsigned long DevId;
  unsigned short MsgId;
  unsigned short Code;
}HOSTCOMM_RECORD_DATABASE;

static HOSTCOMM_RECORD_DATABASE HostComm_DevRespList[MAX_DEV_RESP_LIST];
static HOSTCOMM_RECORD_DATABASE HostComm_DevWaitRespList[MAX_DEV_WAIT_RESP_LIST];

void HostComm_RunTimeout(unsigned long ms);
void HostComm_StartTimeout(void);
void HostComm_StopTimeout(void);

void HostComm_ReportFatalError(void);
void HostComm_HandleDevReadyEvt(void);
void HostComm_HandleNetworkChange(void);
void HostComm_HandleRxData(unsigned char *ptr, unsigned long len);
void HostComm_HandleRxMsgData(unsigned char *RxMsg);
void HostComm_Dev_HandleRxMsg(HOSTCOMM_SRV_MSG * SrvMsg);
void HostComm_HandleTimeoutEvt(void);

bool HostComm_Dev_UpdateWaitRespList(unsigned long SrcId, unsigned short MsgId, unsigned short MsgCode);
bool HostComm_Dev_AddWaitRespList(unsigned long DestId, unsigned short MsgId, unsigned short CmdCode);

unsigned char HostComm_Dev_FindRespListIndex(unsigned short RespCode);
unsigned char HostComm_Dev_GetFreeRespListIndex(void);
void HostComm_Dev_FreeRespListIndex(unsigned char index);
unsigned short HostComm_DevGenMsgId(void);
bool HostComm_Dev_SendMsg(unsigned short MsgId, unsigned long DestId, unsigned short MsgCode, unsigned char *data, unsigned char len);

#endif	//HOST_COMM_INT_H

