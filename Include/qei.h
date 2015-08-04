/*
 * qei.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef QEI_QEI_H_
#define QEI_QEI_H_


extern void qei_init(uint16_t ms_Timebase);
extern bool qei_getVelocity(bool Select, int32_t *Velocity);
extern int32_t qei_Get_Position(bool Select);
extern void qei_Set_Position(bool Select, int32_t position);

#endif /* QEI_QEI_H_ */
