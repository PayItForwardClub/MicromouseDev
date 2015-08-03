/*
 * qei.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef QEI_QEI_H_
#define QEI_QEI_H_


extern void qei_init(uint16_t ms_Timebase);
bool qei_getVelocityLeft(int32_t *Velocity);
bool qei_getVelocityRight(int32_t *Velocity);
int32_t qei_getPosRight();
int32_t qei_getPosLeft();
void qei_setPosLeft(int32_t pos);
void qei_setPosRight(int32_t pos);

#endif /* QEI_QEI_H_ */
