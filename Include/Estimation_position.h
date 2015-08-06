#ifndef __ESTIMATION_POSITION_H__
#define __ESTIMATION_POSITION_H__

#include <stdbool.h>
#include <stdint.h>

extern double adb_Error1[5];//e, e_, e__, ...
extern int32_t ai32_SetPoint1[4];//SP, SP_, SP__, SP___
extern int32_t ai32_Position1[8];
extern double adb_output1[8];

extern double adb_Error2[5];//e, e_, e__, ...
extern int32_t ai32_SetPoint2[4];//SP, SP_, SP__, SP___
extern int32_t ai32_Position2[8];
extern double adb_output2[8];

extern void Estimation_Algorithm(bool Select);

#endif
