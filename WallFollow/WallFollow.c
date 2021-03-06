/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	WallFollow.c
 * @brief	wall follow controller
 */


#include "../include.h"
#include "WallFollow.h"

//* Private function prototype ----------------------------------------------*/
static void Pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
//* Private variables -------------------------------------------------------*/
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;
static TIMER_ID pid_TimerID = INVALID_TIMER_ID;
PID_PARAMETERS pidWallParam = {.Kp=1.0 , .Kd=0.001, .Ki=0.001, .Ts=20};

/**
 * @brief Init Wall follow controller
 * @param pid param
 */
void pid_Wallfollow_init(PID_PARAMETERS pid_param)
{
//	if (pid_TimerID != 0xff)
//	{
//		TIMER_UnregisterEvent(pid_TimerID);
//	}
	pid_Runtimeout(&Pid_process_callback, pid_param.Ts);

	ui32_msLoop =  pid_param.Ts;
}

/**
 * @brief Wall follow controller
 */
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

	u = pid_process(&pidWallParam,error);
	set_speed[0] = averageSpeed + (int32_t)(u / 2);
	set_speed[1] = averageSpeed - (int32_t)(u / 2);

	speed_set(MOTOR_RIGHT, set_speed[0]);
	speed_set(MOTOR_LEFT, set_speed[1]);
	return true;
}


static void Pid_process_callback(void)
{
	pid_TimerID = INVALID_TIMER_ID;
	ControlFlag = true;
	pid_Runtimeout(&Pid_process_callback, ui32_msLoop);
}

void pid_Wallfollow_process(void)
{
	if (ControlFlag)
	{
		ControlFlag = false;
		pid_wallfollow((float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IR_GetIrDetectorValue(1), (float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IR_GetIrDetectorValue(2), 500);
		bluetooth_print("IR: %d, %d, %d, %d\r\n", IR_GetIrDetectorValue(0), IR_GetIrDetectorValue(1), IR_GetIrDetectorValue(2), IR_GetIrDetectorValue(3));
	}
}

void pid_Wallfollow_set_follow(WALL_FOLLOW_SELECT follow_sel)
{
	e_wall_follow_select = follow_sel;
}

static void pid_StopTimeout(void)
{
	if (pid_TimerID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(pid_TimerID);
	pid_TimerID = INVALID_TIMER_ID;
}

static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime)
{
	pid_StopTimeout();
	pid_TimerID = TIMER_RegisterEvent(CallbackFcn, msTime);
	return pid_TimerID;
}
