#include "string.h"
#include "include.h"
#include "HostComm_ext.h"

void HostCommHandler_HandshakeEvent(unsigned char *data, unsigned char size);
void HostCommHandler_SpeedSet(unsigned char *data, unsigned char size);

void HostCommHandler_Init(void)
{
	HostComm_Dev_RegisterCallback(HANDSHAKE_CMD, &HostCommHandler_HandshakeEvent);
	HostComm_Dev_RegisterCallback(MSG_SPEED_SET_CMD, &HostCommHandler_SpeedSet);
}

void HostCommHandler_HandshakeEvent(unsigned char *data, unsigned char size)
{
	static unsigned char data_rcv[100];
	
	memset(data_rcv, 0, 100);
	memcpy(data_rcv, data, size);
}

void HostCommHandler_SpeedSet(unsigned char *data, unsigned char size)
{
	static unsigned char data_rcv[100];
	
	memset(data_rcv, 0, 100);
	memcpy(data_rcv, data, size);
}
