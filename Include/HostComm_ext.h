#ifndef HOST_COMM_EXT_H
#define HOST_COMM_EXT_H

#include <stdint.h>
#include <stdbool.h>

#define EXTERN extern


typedef enum		//define list of requests sent from server
{
//Discovery commands
	HANDSHAKE_CMD,
	PING_CMD,
	
	ROBOT_START_CMD,
	ROBOT_STOP_CMD,
	
	SET_PID_CMD,
	SET_SPEED_CMD,
	
	SET_MODE_CMD,
	
	SET_LED_DISPLAY_CMD,
	SET_BUTTON_DEBOUNCE_CMD,
	
	SET_BL_NAME_CMD,
	SET_BL_PASSWORD_CMD,
	SET_BL_UART_CMD,
	
	IMAGE_PROCESSING_INFO_CMD,
	
	MSG_UPDATE_POS_CMD,
	MSG_TIME_CMD,
	MSG_BOX_CMD,
	MSG_WAYPOINT_CMD,
	MSG_TRACK_BOT_CMD,
	MSG_UPDATE_BOT_CMD,
	MSG_SPEED_SET_CMD,
	MSG_START_TEST_CMD,
	MSG_SET_PID_CMD,
	
	TOTAL_COMMAND_NUMBER
}HOSTCOMM_COMMAND_ID;


typedef enum		//define list of response/events sent from device
{
//Diagnostic/Urgent events
	HANDSHAKE_EVENT,
	PING_EVENT,
	
	UPDATE_BOT_EVENT,

	TOTAL_EVENT_NUMBER
}HOSTCOMM_EVENT_LIST;

EXTERN 
void HostComm_Init(void);
EXTERN void HostComm_Reset(void);
EXTERN void HostComm_SetDeviceId(unsigned long DevId);
EXTERN void HostComm_SetServerId(unsigned long SrvId);
EXTERN void HostComm_Dev_RegisterCallback(HOSTCOMM_COMMAND_ID commandID, void (*callback)(unsigned char *data, unsigned char size));
EXTERN void HostComm_Dev_UnregisterCallback(HOSTCOMM_COMMAND_ID commandID);
EXTERN void HostComm_Dev_RegisterAckCallback(HOSTCOMM_EVENT_LIST eventID, void (*ack_callback)(void));
EXTERN void HostComm_Dev_UnregisterAckCallback(HOSTCOMM_EVENT_LIST eventID);
EXTERN bool HostComm_Dev_SendEvent(HOSTCOMM_EVENT_LIST eventId, unsigned char *data, unsigned char len);
EXTERN void HostComm_SetDisconnectedCallback(void (*callback)());
EXTERN void HostComm_SetConnectedCallback(void (*callback)());
EXTERN void HostComm_EventProcessing(void);
EXTERN unsigned char Utilities_AsciiToVal(unsigned char ascii);
EXTERN unsigned short SwapShort(unsigned char *data);
EXTERN unsigned long SwapLong(unsigned char *data);
EXTERN uint16_t HostComm_calcCheckSum(uint8_t *data, uint8_t len);
#endif	//HOST_COMM_EXT_H






