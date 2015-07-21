/*
 * WallFollow.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef WALLFOLLOW_WALLFOLLOW_H_
#define WALLFOLLOW_WALLFOLLOW_H_

typedef enum
{
	WALL_FOLLOW_NONE = 0,	//Don't use pid
	WALL_FOLLOW_LEFT,
	WALL_FOLLOW_RIGHT,
	WALL_FOLLOW_BOTH
} WALL_FOLLOW_SELECT;

extern void pid_Wallfollow_init(PID_PARAMETERS pid_param);
extern void pid_Wallfollow_process(void);
extern void pid_Wallfollow_set_follow(WALL_FOLLOW_SELECT follow_sel);
extern WALL_FOLLOW_SELECT Get_Pid_Wallfollow();
#endif /* WALLFOLLOW_WALLFOLLOW_H_ */
