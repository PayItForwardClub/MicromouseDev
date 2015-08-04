/*
 * Timer.h
 *
 *  Created on: Jul 4, 2015
 *      Author: NHH
 */

#ifndef TIMER_H_
#define TIMER_H_

typedef void (*TIMER_CALLBACK_FUNC)();

typedef unsigned char TIMER_ID;

#define INVALID_TIMER_ID 0xff

void Timer_Init(void);
TIMER_ID TIMER_RegisterEvent(TIMER_CALLBACK_FUNC callback, unsigned long ms);
bool TIMER_UnregisterEvent(TIMER_ID timer_id);

#endif /* TIMER_H_ */
