/*
 * WallFollow.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "WallFollow.h"

static void Pid_process_callback(void);
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;

TIMER_ID pid_TimerID = 0xff;

void pid_Wallfollow_init(PID_PARAMETERS pid_param)
{
	pid_init();
	pid_set_parameters(pid_param);
	if (pid_TimerID != 0xff)
	{
		TIMER_UnregisterEvent(pid_TimerID);
	}
	pid_TimerID = TIMER_RegisterEvent(&Pid_process_callback, pid_param.Ts);

	ui32_msLoop =  pid_param.Ts;
}

static bool pid_wallfollow(float delta_IR_left, float delta_IR_right, float averageSpeed)
{
	static float error, u;
	int32_t set_speed[2];

	switch (e_wall_follow_select)
	{
		case WALL_FOLLOW_NONE:	//Do nothing
			return true;
		case WALL_FOLLOW_LEFT:
			error = delta_IR_left;
			break;
		case WALL_FOLLOW_RIGHT:
			error = delta_IR_right;
			break;
		case WALL_FOLLOW_BOTH:
			error = delta_IR_left - delta_IR_right;
			break;
		default:
			return false;
	}

	u = pid_process(error);
	set_speed[0] = averageSpeed + (int32_t)(u / 2);
	set_speed[1] = averageSpeed - (int32_t)(u / 2);

	speed_set(MOTOR_RIGHT, set_speed[0]);
	speed_set(MOTOR_LEFT, set_speed[1]);
	return true;
}

static void Pid_process_callback(void)
{
	ControlFlag = true;
	TIMER_RegisterEvent(&Pid_process_callback, ui32_msLoop);
}

void pid_Wallfollow_process(void)
{
	if (ControlFlag)
	{
		ControlFlag = false;
		pid_wallfollow((float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IR_GetIrDetectorValue(1), (float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IR_GetIrDetectorValue(2), 500);
	}
}

void pid_Wallfollow_set_follow(WALL_FOLLOW_SELECT follow_sel)
{
	e_wall_follow_select = follow_sel;
}

WALL_FOLLOW_SELECT Get_Pid_Wallfollow()
{
	return e_wall_follow_select;
}
