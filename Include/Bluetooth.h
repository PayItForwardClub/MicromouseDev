/*
 * Bluetooth.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef BLUETOOTH_BLUETOOTH_H_
#define BLUETOOTH_BLUETOOTH_H_

#include <stdint.h>
#include <stdbool.h>

/*
    Definition of Error code
*/
typedef enum
{
  HC05_NULL_ERROR,
  HC05_CONFIGURE_DEV_FAILED,
  HC05_WRITE_FAILED,
  HC05_READ_FAILED,
  HC05_QUERY_FAILED,
  HC05_DEVICE_READY,
  HC05_WRITE_DONE,
  HC05_READ_DONE,
  HC05_RX_AVAILABLE,
  HC05_QUERY_NO_RX_DATA,
}HC05_SYSTEM_INFO_ID;

typedef enum
{
	HC05_RX_ACTION_NONE = 0,
	HC05_RX_NOTIFY_AVAIABLE,
	HC05_RX_GET_DATA,
	HC05_RX_ACTION_IDLE
} HC05_RX_ACTIONS;
/*
    Definition of signals notified
*/
typedef enum
{
    HC05_NULL_SIGNAL,
    HC05_OPERATION_ERROR,
    HC05_NETWORK_INFO,
    HC05_RX_TX_INFO,
    HC05_DEVICE_INFO
}HC05_SYSTEM_INFO_TYPES;

typedef enum
{
  HC05_NULL_EVENT = 0,
  HC05_TIMEOUT_EVENT,
  HC05_TX_DONE_EVENT,
  HC05_RX_READY_EVENT
} HC05_EVENT_ID;

extern void bluetooth_init(uint32_t baudrate);
extern bool bluetooth_send(const uint8_t *pui8Buffer, uint32_t ui32Count);
extern uint16_t bluetooth_recv(uint8_t* rxBuf, uint16_t numToRead, bool block);
extern void bluetooth_print(const char * __restrict format, ...);
extern bool HC05_RegisterEvtNotify(void (*callback)());
extern bool HC05_UnregisterEvtNotify(void (*callback)());
extern HC05_SYSTEM_INFO_TYPES HC05_GetSystemInfoType(void);
extern HC05_SYSTEM_INFO_ID HC05_GetSystemInfoID(void);
extern void HC05_EventProcessing(void);
extern void HC05_QueryRxData(void);
extern uint16_t HC05_GetRxData(uint8_t *data, uint16_t len);
extern uint16_t HC05_GetRxSize(void);
extern void HC05_RunTimeout(unsigned long ms);
extern void HC05_StopTimeout(void);

#endif /* BLUETOOTH_BLUETOOTH_H_ */
