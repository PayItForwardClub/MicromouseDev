/*
 * Timer.c
 *
 *  Created on: Jul 4, 2015
 *      Author: NHH
 */
#include <stdint.h>
#include <stdbool.h>
#include "../include.h"
#include "Timer.h"

#define TIMER_PERIOD_MS 1
#define MAX_TIMEOUT_EVT 10

typedef struct
{
  TIMER_CALLBACK_FUNC callback;
  unsigned long period_cnt;
}TIMEOUT_EVT;

TIMEOUT_EVT timer_event_list[MAX_TIMEOUT_EVT];

void TIMER_ISR(void);

void Timer_Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);

	ROM_TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(TIMER4_BASE, TIMER_A, ROM_SysCtlClockGet() * TIMER_PERIOD_MS/ 1000);	//Interval: TIMER_PERIOD_MS(ms)

	TimerIntRegister(TIMER4_BASE, TIMER_A, &TIMER_ISR);
	ROM_IntEnable(INT_TIMER4A);
	ROM_TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerControlStall(TIMER4_BASE, TIMER_A, false);
	ROM_TimerEnable(TIMER4_BASE, TIMER_A);
}

TIMER_ID TIMER_RegisterEvent(TIMER_CALLBACK_FUNC callback, unsigned long ms)
{
    int i;
    for(i=0; i< MAX_TIMEOUT_EVT; i++)
    {
      if((timer_event_list[i].period_cnt == 0) && (timer_event_list[i].callback == NULL)) break;
    }
    if(i == MAX_TIMEOUT_EVT)
    	return INVALID_TIMER_ID;

    timer_event_list[i].period_cnt = (unsigned long)(ms/TIMER_PERIOD_MS);
    timer_event_list[i].callback = callback;

    return (TIMER_ID)i;
}

bool TIMER_UnregisterEvent(TIMER_ID timer_id)
{
	bool ret = false;
    if(timer_id < MAX_TIMEOUT_EVT)
    {
        timer_event_list[timer_id].period_cnt = 0;
        timer_event_list[timer_id].callback = NULL;
        ret = true;
    }
    return ret;
}

void TIMER_ISR(void)
{
    int i;
	ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	for(i=0; i<MAX_TIMEOUT_EVT; i++)
	{
		if(timer_event_list[i].period_cnt > 0)
		{
			timer_event_list[i].period_cnt--;
			if(timer_event_list[i].period_cnt == 0 && timer_event_list[i].callback != NULL)
			{
				(timer_event_list[i].callback)();
				/*
				 * Only clear timeout callback when period equal to 0
				 * Another callback could be register in current timeout callback
				 */
				if (timer_event_list[i].period_cnt == 0)
					timer_event_list[i].callback = NULL;
			}
		}
	}

}
