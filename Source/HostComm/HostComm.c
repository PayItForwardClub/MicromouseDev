#include "string.h"
#include "include.h"
#include "HostComm_int.h"
#include "HostComm_ext.h"
#include "Bluetooth.h"

#define ACK_FROM_MSG_CODE(msg_code)  ((msg_code) | 1<<15)
#define MSG_CODE_FROM_ACK(msg_code)  ((msg_code) & ~(1<<15))

extern void HostCommHandler_Init(void);

void (*AppDisconnectedCallback) (void);
void (*AppConnectedCallback)(void);
void HostComm_HandleBluetoothEvent(void);

void HostComm_Init(void)
{	
	bluetooth_init(115200);
	HostComm_SetServerId(0x0001);
	HostComm_SetDeviceId(0x3333);
	HC05_RegisterEvtNotify(&HostComm_HandleBluetoothEvent);
	HostCommHandler_Init();
}

void HostComm_Reset(void)
{
 
}

void HostComm_SetDisconnectedCallback(void (*callback)())
{
  AppDisconnectedCallback = callback;
}

void HostComm_SetConnectedCallback(void (*callback)())
{
  AppConnectedCallback = callback;
}

void HostComm_Dev_RegisterCallback(HOSTCOMM_COMMAND_ID commandID, void (*callback)(unsigned char *data, unsigned char size))
{
  for(int i = 0; i < TOTAL_COMMAND_NUMBER; i++)
  {
    if(commandID == HostComm_DevCallbacks[i].CommandId)
    {
      HostComm_DevCallbacks[i].callback = callback;
      break;
    }
  }
}

void HostComm_Dev_UnregisterCallback(HOSTCOMM_COMMAND_ID commandID)
{
  for(int i = 0; i < TOTAL_COMMAND_NUMBER; i++)
  {
    if(commandID == HostComm_DevCallbacks[i].CommandId)
    {
      HostComm_DevCallbacks[i].callback = NULL;
      break;
    }
  }  
}

void HostComm_Dev_RegisterAckCallback(HOSTCOMM_EVENT_LIST eventID, void (*ack_callback)(void))
{
  for(int i=0; i<TOTAL_EVENT_NUMBER; i++)
  {
    if(eventID == HostComm_Dev_EventList[i].EventId)
    {
      HostComm_Dev_EventList[i].ack_callback = ack_callback;
    }
  }
}

void HostComm_Dev_UnregisterAckCallback(HOSTCOMM_EVENT_LIST eventID)
{
  for(int i=0; i<TOTAL_EVENT_NUMBER; i++)
  {
    if(eventID == HostComm_Dev_EventList[i].EventId)
    {
      HostComm_Dev_EventList[i].ack_callback = NULL;
    }
  }
}

void HostComm_SetDeviceId(unsigned long DevId)
{
  HostComm_DeviceId = DevId;
}

void HostComm_SetServerId(unsigned long SrvId)
{
  HostComm_ServerId = SrvId;
}

void HostComm_HandleBluetoothEvent(void)
{
  HC05_SYSTEM_INFO_TYPES InfoType;
  HC05_SYSTEM_INFO_ID InfoId;
	static uint16_t datalen;

  InfoType = HC05_GetSystemInfoType();
  InfoId = HC05_GetSystemInfoID();

  if(InfoType == HC05_OPERATION_ERROR)
  {

  }

  switch(InfoId)
  {
    case HC05_RX_AVAILABLE:			//there is Rx data
			HC05_QueryRxData();
    break;
    case HC05_READ_DONE:			//done TCP reading
			datalen = HC05_GetRxSize();
			HC05_GetRxData(HostComm_RxBuf, datalen);
			HostComm_HandleRxData(HostComm_RxBuf, datalen);
    break;
    case HC05_WRITE_DONE:			//done TCP writing
    break;
    default: break;
  }
}

void HostComm_ReportFatalError()
{
}

void HostComm_HandleDevReadyEvt()
{

}

/******************************************************** 
*                   TIMEOUT FUCNTIONS                   *
********************************************************/

void HostComm_HandleTimeoutEvt()
{

}


void HostComm_RunTimeout(unsigned long ms)
{
  HostComm_Timer_ID = TIMER_RegisterEvent(HostComm_HandleTimeoutEvt, ms);     
}

void HostComm_StopTimeout()
{
  TIMER_UnregisterEvent(HostComm_Timer_ID);
}


#define SIZEOF_TEMP_RX_BUF  512
unsigned long HostComm_SplitRemainRx;
unsigned long HostComm_SplitRxSize;
unsigned char HostComm_SplitRxMsg[SIZEOF_TEMP_RX_BUF];

void HostComm_HandleRxData(unsigned char *ptr, unsigned long len)
{
  unsigned char * RxMsgPtr = NULL;
	static unsigned char * TempPtr = NULL;
  unsigned long MsgOffset;
  unsigned short PayloadLen, MsgLen, len_temp;  
	
  MsgOffset = 1;
	TempPtr = ptr;
  while(len)
  {
    if(HostComm_SplitRxSize)       //if we store part of the message before
    {
//			if(len >= HostComm_SplitRemainRx)
//			{
//				memcpy(HostComm_SplitRxMsg + HostComm_SplitRxSize, ptr, HostComm_SplitRemainRx);
//				RxMsgPtr = HostComm_SplitRxMsg;
//				MsgOffset += HostComm_SplitRemainRx;
//				len -= HostComm_SplitRemainRx;
//				HostComm_SplitRemainRx = 0;
//				HostComm_SplitRxSize = 0;
//			}
//			else
//			{
//				memcpy(HostComm_SplitRxMsg + HostComm_SplitRxSize, ptr, len);
//				HostComm_SplitRxSize += len;
//				HostComm_SplitRemainRx -= len;
//				len = 0;

//			}
			memcpy(HostComm_SplitRxMsg + HostComm_SplitRxSize, TempPtr, len);
			len += HostComm_SplitRxSize;

			for (len_temp = 0; len_temp < len; len_temp++)
			{
				if (HostComm_SplitRxMsg[len_temp] == HOSTCOMM_MSG_START_CODE)
					break;
			}
			
			if (len_temp != len)
			{
				MsgOffset = len_temp + 1;
				TempPtr = HostComm_SplitRxMsg;
			}
			else
				HostComm_SplitRxSize = 0;
    }
    else
    {			
			len_temp = 0;
			while ((*TempPtr != HOSTCOMM_MSG_START_CODE) && (len_temp < len))
			{
				TempPtr++;
				len_temp++;
			}
			
			if (len_temp == len)
				break;
			else
				MsgOffset = 1;
		}
		
		{
      PayloadLen = TempPtr[MsgOffset] | ((uint16_t)(TempPtr[MsgOffset+1]) << 8);
      MsgLen = PayloadLen + 6;
      if(len >= MsgLen)            //if the buffer contains full message
      {
        RxMsgPtr = &TempPtr[MsgOffset-1];
        len = len - MsgLen;
        MsgOffset += MsgLen + 1;        //indexing to next message in the buffer
				TempPtr += MsgLen;
      }
      else                        //if the buffer does not contain full message, store part of it and read more data from U200 
      {
        memcpy(HostComm_SplitRxMsg, &TempPtr[MsgOffset], len);
        HostComm_SplitRemainRx = MsgLen - len;       //remain part of the message that will be read
        HostComm_SplitRxSize = len;               //part of the message that has been read
        len = 0;
      }
    }
    if(RxMsgPtr != NULL)
    {
      HostComm_HandleRxMsgData(RxMsgPtr);         //handl full message here
      RxMsgPtr = NULL;
    }
  } //while(...)  
}


void HostComm_HandleRxMsgData(unsigned char *RxMsg)
{
  HOSTCOMM_SRV_MSG SrvMsg;
	uint16_t ui16_crc = 0;

	if (HOSTCOMM_MSG_START_CODE != RxMsg[0])
		return;
	
  SrvMsg.msg_len = RxMsg[1] | ((uint16_t)RxMsg[2] << 8);
	
	if (HOSTCOMM_MSG_END_CODE != RxMsg[SrvMsg.msg_len+5])
		return;
	
  SrvMsg.msg_id = RxMsg[6]; 
  SrvMsg.dst_id = RxMsg[5];
//  if(SrvMsg.dst_id != HostComm_DeviceId) return;		//if it is not our message
  SrvMsg.src_id = RxMsg[4]; 
  SrvMsg.date_time = 0; 
  SrvMsg.msg_code = RxMsg[3];
  SrvMsg.data_len = SrvMsg.msg_len - HOSTCOMM_MSG_HEADER_SIZE;
	
	ui16_crc = *((uint16_t *)(RxMsg + SrvMsg.msg_len + 3));
	if (ui16_crc != HostComm_calcCheckSum(&RxMsg[3], SrvMsg.msg_len))
		return;
	
  if(SrvMsg.data_len)
  {
    SrvMsg.data_ptr = &RxMsg[HOSTCOMM_MSG_HEADER_SIZE+3];
  }
  else
  {
    SrvMsg.data_ptr = NULL;
  }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
//  if(SrvMsg.msg_code > 0x1000 )	//if it is a request
  {

    // Device receives requests/responses from server 
    HostComm_Dev_HandleRxMsg(&SrvMsg);
  }
}


void HostComm_Dev_HandleRxMsg(HOSTCOMM_SRV_MSG * SrvMsg)
{
  bool IsEvtResp;
  unsigned char i, index;
  unsigned short msg_code;

	if (0)
	{
//  if(SrvMsg->msg_code > 0x8000)       //if it is a ACK msg
//  {
//    msg_code = MSG_CODE_FROM_ACK(SrvMsg->msg_code);
//    for(i=0; i<TOTAL_EVENT_NUMBER; i++)
//    {
//      if((msg_code == HostComm_Dev_EventList[i].EventCode) && (HostComm_Dev_EventList[i].ack_callback != NULL))
//      {
//        (HostComm_Dev_EventList[i].ack_callback)();
//        break;
//      }
//    }     
  }
  else        //it is a command from srv   
  {  
    //Do ACK first
//    HostComm_Dev_SendMsg(SrvMsg->msg_id, SrvMsg->src_id, ACK_FROM_MSG_CODE(SrvMsg->msg_code), NULL, 0);      

    //check if it is a response from server for an event from device
    IsEvtResp = HostComm_Dev_UpdateWaitRespList(SrvMsg->src_id, SrvMsg->msg_id, SrvMsg->msg_code);

    //find callback of this msg
    for(i=0; i<TOTAL_COMMAND_NUMBER; i++)
    {
			if(SrvMsg->msg_code == HostComm_DevCallbacks[i].CommandCode) break;
    }    
    //if it is a defined command and has a valid callback func
    if((i != TOTAL_COMMAND_NUMBER) && (HostComm_DevCallbacks[i].callback != NULL))   //if it is a known command
    {
			//record it into DevRespList[] if it is a command requiring a response
			if(HostComm_DevCallbacks[i].ResponseCode != 0 && IsEvtResp == false)
			{
				index = HostComm_Dev_GetFreeRespListIndex();
				if(index == 0xFF) while(1);		//the list is full now, it should not happen
				HostComm_DevRespList[index].DevId = SrvMsg->src_id;
				HostComm_DevRespList[index].MsgId = SrvMsg->msg_id;
				HostComm_DevRespList[index].Code = HostComm_DevCallbacks[i].ResponseCode;
			}
			//involve callback
			(HostComm_DevCallbacks[i].callback)(SrvMsg->data_ptr, SrvMsg->data_len);            
    }
  }
}

bool HostComm_Dev_UpdateWaitRespList(unsigned long SrcId, unsigned short MsgId, unsigned short MsgCode)
{
  unsigned char i;

  for(i=0; i<MAX_DEV_WAIT_RESP_LIST; i++)
  {
    if(HostComm_DevWaitRespList[i].MsgId == MsgId &&
    HostComm_DevWaitRespList[i].DevId == SrcId &&
    HostComm_DevWaitRespList[i].Code == MsgCode) break;
  }
  if(i == MAX_DEV_WAIT_RESP_LIST) 
  {
    return false;
  }
  else 
  {
    HostComm_DevWaitRespList[i].DevId = 0;	//free this item
    return true;
  }
}

bool HostComm_Dev_AddWaitRespList(unsigned long DestId, unsigned short MsgId, unsigned short CmdCode)
{
  unsigned char i;

  for(i=0; i<MAX_DEV_WAIT_RESP_LIST; i++)
  {
    if(HostComm_DevWaitRespList[i].DevId == 0) break;
  }
  if(i == MAX_DEV_WAIT_RESP_LIST) return false;
  else
  {
    HostComm_DevWaitRespList[i].MsgId = MsgId;
    HostComm_DevWaitRespList[i].DevId = DestId;
    HostComm_DevWaitRespList[i].Code = CmdCode;
    return true;
  }
}

bool HostComm_Dev_SendEvent(HOSTCOMM_EVENT_LIST eventId, unsigned char *data, unsigned char len)
{
  unsigned char i, RespListIndex;
  unsigned short RespCode, MsgId;
  unsigned long DestId;
  bool ret = true;

  //if(HostComm_IsTcpReady == false) return false;

  //find response code from eventId
  for(i=0; i<TOTAL_EVENT_NUMBER; i++)
  {
    if(HostComm_Dev_EventList[i].EventId == eventId) break;
  }
  if(i == TOTAL_EVENT_NUMBER) return false;		//invalid eventId
  RespCode = HostComm_Dev_EventList[i].EventCode;


  RespListIndex = HostComm_Dev_FindRespListIndex(RespCode);
  if(RespListIndex != 0xFF)	
  {
    //if it is a response event for a previously received command
    //then get its MsgId and DevId to build Msg header
    MsgId = HostComm_DevRespList[RespListIndex].MsgId;
    DestId = HostComm_DevRespList[RespListIndex].DevId;

    //free DevRespList[]
    HostComm_Dev_FreeRespListIndex(RespListIndex);
  }
  else
  {
    //if it is just an event from Dev
    //then generate a MsgId and send it to ServerId
    MsgId = HostComm_DevGenMsgId();
    DestId = HostComm_ServerId;
//    switch(RespCode)	
//    {	//currently, only these events require response from server
//      //so I put it into WaitRespList[] here
//      case 0x0001:	//HANDSHAKE EVENT 
//        ret = HostComm_Dev_AddWaitRespList(DestId, MsgId, 0x1001);
//      break;
//      case 0x0002:	//PING EVENT
//        ret = HostComm_Dev_AddWaitRespList(DestId, MsgId, 0x1002);
//      break;
//      default: break;
//    }
  }

  //ready to send msg now
  if(ret == true)
  {
    ret = HostComm_Dev_SendMsg(MsgId, DestId, RespCode, data, len);
  }

  return ret;
	
}

unsigned short HostComm_DevGenMsgId()
{
  unsigned short DevMsgId = HostComm_DevMsgId;

  HostComm_DevMsgId++;
  if(HostComm_DevMsgId > 0x00FF) HostComm_DevMsgId = 0;

  return DevMsgId;
}

unsigned char HostComm_Dev_FindRespListIndex(unsigned short RespCode)
{
  unsigned char i;

  for(i=0; i<MAX_DEV_RESP_LIST; i++)
  {
    if(HostComm_DevRespList[i].Code == RespCode) break;
  }
  if(i == MAX_DEV_RESP_LIST) return 0xFF;
  else return i;
}

unsigned char HostComm_Dev_GetFreeRespListIndex()
{
  unsigned char i;

  for(i=0; i<MAX_DEV_RESP_LIST; i++)
  {
    if(HostComm_DevRespList[i].DevId == 0) break;
  }
  if(i == MAX_DEV_RESP_LIST) return 0xFF;
  else return i;    
}

void HostComm_Dev_FreeRespListIndex(unsigned char index)
{
  if(index < MAX_DEV_RESP_LIST)
  {
    HostComm_DevRespList[index].DevId = 0; 
  }
}


//construct Message Header and send to network
bool HostComm_Dev_SendMsg(unsigned short MsgId, unsigned long DestId, unsigned short MsgCode, unsigned char *data, unsigned char len)
{
  unsigned char MsgLen;
  unsigned long DateTime;
  unsigned short ShortTemp;
  unsigned long LongTemp;

  DateTime = 0x12345678;  //AppModule_GetDateTime();		//get Date/Time from AppModule
	
	HostComm_TxBuf[0] = HOSTCOMM_MSG_START_CODE;
	
  MsgLen = len + HOSTCOMM_MSG_HEADER_SIZE;
  HostComm_TxBuf[1] = (MsgLen & 0xff);							//insert MsgLen
  HostComm_TxBuf[2] = ((MsgLen >> 8) & 0xff);							//insert MsgLen
//  ShortTemp = SwapShort((unsigned char *)&MsgId);
//  memcpy(&HostComm_TxBuf[1], &ShortTemp, 2);				//insert MsgId
	HostComm_TxBuf[6] = MsgId & 0xff;
	
//  LongTemp = SwapLong((unsigned char *)&HostComm_ServerId);
//  memcpy(&HostComm_TxBuf[3], &LongTemp, 4);				//insert DestId
	HostComm_TxBuf[4] = HostComm_DeviceId;
//  LongTemp = SwapLong((unsigned char *)&HostComm_DeviceId);
//  memcpy(&HostComm_TxBuf[7], &LongTemp, 4);	            //insert SrcId
	HostComm_TxBuf[5] = DestId & 0xff;
//  LongTemp = SwapLong((unsigned char *)&DateTime);
//  memcpy(&HostComm_TxBuf[11], &LongTemp, 4);			//insert Date/Time
//  ShortTemp = SwapShort((unsigned char *)&MsgCode);
//  memcpy(&HostComm_TxBuf[15], &ShortTemp, 2);			//insert MsgCode
	HostComm_TxBuf[3] = MsgCode & 0xff;
	
  if(len > 0)
  {
    memcpy(&HostComm_TxBuf[7], data, len);			//insert MsgData
  }

	ShortTemp = HostComm_calcCheckSum(&HostComm_TxBuf[3], MsgLen);
	
	HostComm_TxBuf[7+len] = ShortTemp & 0xff;
	HostComm_TxBuf[8+len] = (ShortTemp >> 8) & 0xff;
	HostComm_TxBuf[9+len] = HOSTCOMM_MSG_END_CODE;
	
  return bluetooth_send(HostComm_TxBuf, MsgLen + 6);
}

void HostComm_HC05_HandleTimeoutEvt()
{
	HostComm_HC05_Timer_IsTimeout = true;
}

void HostComm_HC05_RunTimeout(unsigned long ms)
{
  HostComm_HC05_Timer_ID = TIMER_RegisterEvent(HostComm_HC05_HandleTimeoutEvt, ms);     
}

void HostComm_HC05_StopTimeout()
{
  TIMER_UnregisterEvent(HostComm_HC05_Timer_ID);
}

void HostComm_EventProcessing(void)
{
	HC05_EventProcessing();
	if (HostComm_HC05_Timer_IsTimeout == true)
	{

		HostComm_HC05_Timer_IsTimeout = false;

		//HostComm_TCPRunTimeout(SocketSettingTimeThreshold * 1000 * 3600);
	}
}


//return ASCII val of a numeric number (0-9)
unsigned char Utilities_ValToAscii(unsigned char val)
{
  return (val + 0x30);
}

//return numeric value of a ASCII number (0-9)
unsigned char Utilities_AsciiToVal(unsigned char ascii)
{
  return (ascii - 0x30);
}


#define __LITTLE_ENDIAN
#undef __BIG_ENDIAN

unsigned short SwapShort(unsigned char *data)
{
    unsigned short val;
#ifdef __BIG_ENDIAN
    val = *((unsigned short *)data);
#else
    val =  *data  << 8 | *(data+1);
#endif
	return val;
}

unsigned long SwapLong(unsigned char *data)
{
	unsigned long val;
#ifdef __BIG_ENDIAN
	val = *((unsigned long *)data);
#else
	val = *data << 24 | *(data+1) << 16 | *(data+2) << 8 | *(data+3);
#endif
	return val;
}

uint16_t HostComm_calcCheckSum(uint8_t *data, uint8_t len)
{
    uint16_t sum=0;
    for (int i=0; i<len; i++)
    {
        sum+=data[i];
    }
    return sum;
}
