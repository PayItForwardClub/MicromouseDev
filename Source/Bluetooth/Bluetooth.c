/*
 * Bluetooth.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "include.h"
#include "Bluetooth.h"

/** Private Defines **/
#define MAX_RX_BUF              1500
#define MAX_TX_BUF              1500

																								
#define HC05_EVENT_QUEUE_SIZE   10
HC05_EVENT_ID hc05_EvtQueue[HC05_EVENT_QUEUE_SIZE];
volatile unsigned char hc05_evt_w_index;
volatile unsigned char hc05_evt_r_index;
volatile bool hc05_IsEvtQueueFull;

typedef void (*HC05_CALLBACK)(void);
#define MAX_NUM_CALLBACK 5
HC05_CALLBACK HC05_Notify_Callbacks[MAX_NUM_CALLBACK];

/** Private Variables **/
static volatile uint8_t rxBuffer[MAX_RX_BUF];
static volatile uint16_t rxHead;
static uint16_t rxTail;
static uint16_t ui16_rxSize = 0;
static bool b_is_has_new_data = false;

static volatile uint8_t txBuffer[MAX_TX_BUF];
static volatile uint16_t txHead;
static uint16_t txTail;

static uint16_t ReadRxBuffer(uint8_t* rxBuf, uint16_t numToRead);
static void Bluetooth_RxTxHandler(void);

static HC05_SYSTEM_INFO_TYPES  hc05_SystemInfoType;
static HC05_SYSTEM_INFO_ID     hc05_SystemInfoID;
static HC05_RX_ACTIONS				hc05_currentRxAction = HC05_RX_ACTION_IDLE;

static unsigned char hc05_Timer_ID;
	
static bool HC05_PutEvtIntoQueue(HC05_EVENT_ID event);
static bool HC05_IsEvtQueueEmpty(void);
static HC05_EVENT_ID HC05_GetEvtFromQueue(void);
void HC05_ClearEvtQueue(void);

void bluetooth_init(uint32_t baudrate)
{
		uint32_t ui32_SystemClock;
	
		ui32_SystemClock = u32_UsrSystemClockGet();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32_SystemClock, baudrate,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

//		UARTFIFOEnable(UART0_BASE);
//		UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX1_8);
    UARTIntRegister(UART0_BASE, &Bluetooth_RxTxHandler);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);
		UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
		HC05_ClearEvtQueue();
}

static void Bluetooth_RxTxHandler(void)
{
		uint32_t IntStatus;
		IntStatus = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, IntStatus);

		if (IntStatus & UART_INT_TX)
		{
				if (txTail < MAX_TX_BUF)
        {
          if(txHead != txTail)
					{
						UARTCharPutNonBlocking(UART0_BASE, txBuffer[txTail++]);
          }
					else
					{
						HC05_PutEvtIntoQueue(HC05_TX_DONE_EVENT);
					}
        }
        else
        {
          if(0 != txHead)
					{
						UARTCharPutNonBlocking(UART0_BASE, txBuffer[0]);
            txTail = 1;
          }
					else
					{
						HC05_PutEvtIntoQueue(HC05_TX_DONE_EVENT);
					}
        }
		}
    while (UARTCharsAvail(UART0_BASE))
    {
				b_is_has_new_data = true;
        if (rxHead + 1 < MAX_RX_BUF)
        {
          if((rxHead + 1) != rxTail)
						{
               rxBuffer[rxHead++] = UARTCharGet(UART0_BASE);
							 ui16_rxSize++;
          }
					else
						UARTCharGet(UART0_BASE);
        }
        else
        {
          if(0 != rxTail)
						{
            rxBuffer[rxHead] = UARTCharGet(UART0_BASE);
						ui16_rxSize++;
            rxHead = 0;
          }
					else
						UARTCharGet(UART0_BASE);
        }
    }
}

uint16_t bluetooth_recv(uint8_t* rxBuf, uint16_t numToRead, bool block)
{
     if (block)
     {
          // Blocking mode, so don't return until we read all the bytes requested
          uint16_t bytesRead;
          // Keep getting data if we have a number of bytes to fetch
          while (numToRead)
          {
               bytesRead = ReadRxBuffer(rxBuf, numToRead);
               if (bytesRead)
               {
                    rxBuf += bytesRead;
                    numToRead -= bytesRead;
               }
          }
          return bytesRead;
     }
     else
     {
          // Non-blocking mode, just read what is available in buffer
          return ReadRxBuffer(rxBuf, numToRead);
     }
}

static uint16_t ReadRxBuffer(uint8_t* rxBuf, uint16_t numToRead)
{
	uint16_t bytesRead = 0;

	while(rxTail != rxHead && bytesRead < numToRead)
	{
		if (rxTail  + 1 < MAX_RX_BUF)
		{
			*rxBuf++ = rxBuffer[rxTail++];
			bytesRead++;
		}
		else
		{
			*rxBuf++ = rxBuffer[rxTail];
			rxTail = 0;
			bytesRead++;
		}
	}

	return bytesRead;
}

bool bluetooth_send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
		if (txHead == txTail)
		{
			UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
			ui32Count--;
		}
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
				if (txHead + 1 < MAX_TX_BUF)
        {
          if((txHead + 1) != txTail)
					{
						txBuffer[txHead++] = *pui8Buffer++;
          }
					else
						return false;
        }
        else
        {
          if(0 != txTail){
            txBuffer[txHead] = *pui8Buffer++;
            txHead = 0;
          }
					else
						return false;
        }
        //
        // Write the next character to the UART.
        //
//        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
		return true;
}

void bluetooth_print(const char * __restrict format, ...)
{
	volatile char Tx_Buf[512];
    va_list arg;

    memset((void *)Tx_Buf, 0, sizeof(Tx_Buf));
    //
    // Start the varargs processing.
    //
    va_start(arg, format);

    //
    // Call vsnprintf to perform the conversion.  Use a large number for the
    // buffer size.
    //
    vsprintf((char *)Tx_Buf, format, arg);

    //
    // End the varargs processing.
    //
    va_end(arg);

    bluetooth_send((const uint8_t *)Tx_Buf, strlen((const char *)Tx_Buf));
}

void HC05_QueryRxData(void)
{
	hc05_currentRxAction = HC05_RX_GET_DATA;
	HC05_RunTimeout(100);
}

uint16_t HC05_GetRxData(uint8_t *data, uint16_t len)
{
	volatile uint16_t ui16_size = 0;
	
	ui16_size = ReadRxBuffer(data, len);
	
	if (ui16_rxSize >= len)
		ui16_rxSize -= len;
	else
		ui16_rxSize = 0;
	
	return ui16_size;
}

uint16_t HC05_GetRxSize(void)
{
	return ui16_rxSize;
}

bool HC05_RegisterEvtNotify(void (*callback)())
{
  for(int i=0; i<MAX_NUM_CALLBACK; i++)
  {
    if(HC05_Notify_Callbacks[i] == NULL)
    {
        HC05_Notify_Callbacks[i] = callback;
        return true;
    }
  }
  return false;
}

bool HC05_UnregisterEvtNotify(void (*callback)())
{
  for(int i=0; i<MAX_NUM_CALLBACK; i++)
  {
    if(HC05_Notify_Callbacks[i] == callback)
    {
        HC05_Notify_Callbacks[i] = NULL;
        return true;
    }
  }
  return false;
}


/******************************************************** 
*                  EVENT FUCNTIONS                      *
********************************************************/
void HC05_NotifyUpperLayer(HC05_SYSTEM_INFO_TYPES type,HC05_SYSTEM_INFO_ID info)
{
    hc05_SystemInfoType = type;
    hc05_SystemInfoID =  info;

    for(int i=0; i<MAX_NUM_CALLBACK; i++)
    {
        if(HC05_Notify_Callbacks[i] != NULL)
        {
            (HC05_Notify_Callbacks[i])();
        }
    }
}

HC05_SYSTEM_INFO_TYPES HC05_GetSystemInfoType(void)
{
    return hc05_SystemInfoType;
}

HC05_SYSTEM_INFO_ID HC05_GetSystemInfoID(void)
{
    return hc05_SystemInfoID;
}

void HC05_EventProcessing(void)
{
	HC05_EVENT_ID event;
	
	while(1)        //try to process all events in queue
	{
		event = HC05_GetEvtFromQueue();
		if(event == HC05_NULL_EVENT) 
		{
			break;
		}

		if(event == HC05_RX_READY_EVENT)
		{
			/*
			If it is RX_READY_EVENT, Rx data can be for the current action or for system info
			Therefore, it tries each AT message with EventHandlerForAction and/or ATParserForSysInfo
			*/
		}
		else if(event == HC05_TX_DONE_EVENT)
		{
			HC05_NotifyUpperLayer(HC05_RX_TX_INFO, HC05_WRITE_DONE);
		}
		else	//TIMEOUT_EVENT
		{
			/*
			For other events, only for EvtHandlerForAction()
			*/
			if (b_is_has_new_data == true)
			{
				b_is_has_new_data = false;
				HC05_RunTimeout(100);
			}
			else
			{
				HC05_NotifyUpperLayer(HC05_RX_TX_INFO, HC05_READ_DONE);
				hc05_currentRxAction = HC05_RX_ACTION_IDLE;
			}
		}
	}
	
	if ((b_is_has_new_data == true) && (hc05_currentRxAction == HC05_RX_ACTION_IDLE))
	{
		b_is_has_new_data = false;
		
		hc05_currentRxAction = HC05_RX_NOTIFY_AVAIABLE;
		HC05_NotifyUpperLayer(HC05_RX_TX_INFO, HC05_RX_AVAILABLE);
		
//		HC05_PutEvtIntoQueue(HC05_RX_READY_EVENT);
	}
	
	if (hc05_currentRxAction == HC05_RX_GET_DATA)
	{
		if (HC05_GetRxSize() > 1024)
		{
			HC05_NotifyUpperLayer(HC05_RX_TX_INFO, HC05_READ_DONE);
			HC05_StopTimeout();
			HC05_QueryRxData();
		}
	}
}

/*-----------------------------------------------------------------\
Bluetooth Event Queue
-----------------------------------------------------------------*/
static bool HC05_PutEvtIntoQueue(HC05_EVENT_ID event)
{
  bool ret = false;
  unsigned char i,j;
  if(hc05_IsEvtQueueFull == false)
  {
    /* 
          TODO: What if interrupt happens and puts an event to queue while app is putting to queue
    */
		if (event == HC05_RX_READY_EVENT)
		{
			if (hc05_evt_r_index < hc05_evt_w_index)
			{
				for (i = hc05_evt_r_index; i < hc05_evt_w_index; i++)
				{
					if (hc05_EvtQueue[i] == HC05_RX_READY_EVENT)
					{
						return true;
					}
				}
			}
			else
			{
				for (i = hc05_evt_r_index; i < hc05_evt_w_index + HC05_EVENT_QUEUE_SIZE; i++)
				{
					if (hc05_EvtQueue[i%HC05_EVENT_QUEUE_SIZE] == HC05_RX_READY_EVENT)
					{
						return true;
					}
				}
			}
		}
		
    hc05_EvtQueue[hc05_evt_w_index] = event;
    hc05_evt_w_index++;
    if(hc05_evt_w_index == HC05_EVENT_QUEUE_SIZE) 
    {
      hc05_evt_w_index = 0;
    }
		
    if(hc05_evt_w_index == hc05_evt_r_index) hc05_IsEvtQueueFull = true;
    else hc05_IsEvtQueueFull = FALSE;
    ret = true;
  }
  return ret;
}

static bool HC05_IsEvtQueueEmpty(void)
{
		unsigned char i,j,k;
    i = hc05_evt_r_index;
    j = hc05_evt_w_index;
    k = hc05_IsEvtQueueFull;
    if((i != j) || (k == true)) return false;
    else return true;
}

static HC05_EVENT_ID HC05_GetEvtFromQueue(void)
{
    HC05_EVENT_ID event = HC05_NULL_EVENT;
    if(HC05_IsEvtQueueEmpty() == false)
    {
        event = hc05_EvtQueue[hc05_evt_r_index++];
        if(hc05_evt_r_index == HC05_EVENT_QUEUE_SIZE) hc05_evt_r_index = 0;
        hc05_IsEvtQueueFull = false;
    }
    return event;
}

void HC05_ClearEvtQueue(void)
{
  hc05_IsEvtQueueFull = false;
  hc05_evt_w_index = 0;
  hc05_evt_r_index = 0;
  memset(hc05_EvtQueue, 0, HC05_EVENT_QUEUE_SIZE);
}


/******************************************************** 
*                   TIMEOUT FUCNTIONS                   *
********************************************************/

void HC05_HandleTimeoutEvt(void)
{
	HC05_PutEvtIntoQueue(HC05_TIMEOUT_EVENT);
}


void HC05_RunTimeout(unsigned long ms)
{
  hc05_Timer_ID = TIMER_RegisterEvent(HC05_HandleTimeoutEvt, ms);     
}

void HC05_StopTimeout(void)
{
  TIMER_UnregisterEvent(hc05_Timer_ID);
}

