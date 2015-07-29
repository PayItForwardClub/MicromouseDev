/*
 * WallFollow.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "WallFollow.h"

#define _DEBUG_IR_

static void Pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;

static TIMER_ID pid_TimerID = INVALID_TIMER_ID;
static int32_t PrintStep=0;
static int32_t CtrlStep=0;
static int32_t CtrlStep2=0;
static int32_t CtrlStep3=0;
typedef enum{
	TURN_LEFT,
	TURN_RIGHT,
	TURN_BACK,
	FORWARD,
	BACKWARD
}MOVE;
MOVE eMove=FORWARD;

void pid_Wallfollow_init(PID_PARAMETERS pid_param)
{
	pid_init();
	pid_set_parameters(pid_param);

	ui32_msLoop =  pid_param.Ts * 1000;	//ui32_msLoop(ms) = pid_param.Ts(s)*1000
	pid_Runtimeout(&Pid_process_callback, ui32_msLoop);
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
	pid_TimerID = INVALID_TIMER_ID;
	ControlFlag = true;
}

void pid_Wallfollow_process(void)
{
#ifdef _DEBUG_IR_
	static uint8_t PrintStep = 0;
#endif

#ifdef _DEBUG_PID_
	PID_PARAMETERS pid_params;
#endif

	if (ControlFlag)
	{
		uint32_t IRLeft=IR_GetIrDetectorValue(1);
		uint32_t IRRight=IR_GetIrDetectorValue(2);
		uint32_t IRFrontLeft=IR_GetIrDetectorValue(0);
		uint32_t IRFrontRight=IR_GetIrDetectorValue(3);
		bool isWallLeft=false, isWallRight=false, isWallFront=false;

		pid_Runtimeout(&Pid_process_callback, ui32_msLoop);
		ControlFlag = false;

		isWallLeft = (IRLeft<IR_get_calib_value(IR_CALIB_MAX_LEFT));
		isWallRight = (IRRight<IR_get_calib_value(IR_CALIB_MAX_RIGHT));
		isWallFront = ((IRFrontLeft<IR_get_calib_value(IR_CALIB_MIN_FRONT_LEFT))
				&& (IRFrontRight<IR_get_calib_value(IR_CALIB_MIN_FRONT_RIGHT)));

		switch(eMove)
		{
		case FORWARD:
		{
			LED2_ON();LED2_OFF();LED3_OFF();
			pid_wallfollow((float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IRLeft,
					(float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IRRight, 200);

			switch (e_wall_follow_select)
			{
			case WALL_FOLLOW_LEFT:
			{
				if (!isWallLeft)
				{
					eMove = TURN_LEFT;
					CtrlStep=0;
				}
				else if (!isWallFront)
					eMove = FORWARD;
				else if (!isWallRight)
				{
					eMove = TURN_RIGHT;
					CtrlStep=0;
				}
				else
					eMove = TURN_BACK;
				break;
			}
			case WALL_FOLLOW_RIGHT:
			{
				if (!isWallRight)
				{
					eMove = TURN_RIGHT;
					CtrlStep=0;
				}
				else if (!isWallFront)
					eMove = FORWARD;
				else if (!isWallLeft)
				{
					eMove = TURN_LEFT;
					CtrlStep=0;
				}
				else
					eMove = TURN_BACK;
				break;
			}
			}

			break;
		}
		case TURN_LEFT:
		{
			LED1_OFF();LED1_ON();LED3_OFF();
			if (CtrlStep<14)
			{
				speed_set(MOTOR_RIGHT, 200);
				speed_set(MOTOR_LEFT, 200);
				CtrlStep++;
			}
			else
			{
				speed_set(MOTOR_RIGHT, 200);
				speed_set(MOTOR_LEFT, 120);
				CtrlStep2=0;
			}
			switch (e_wall_follow_select)
			{
			case WALL_FOLLOW_LEFT:
			{
				if (isWallLeft)
				{
					if (!isWallFront)
						eMove = FORWARD;
					else if (!isWallRight)
					{
						eMove = TURN_RIGHT;
						CtrlStep=0;
					}
					else
						eMove = TURN_BACK;
				}
				break;
			}
			case WALL_FOLLOW_RIGHT:
			{
				CtrlStep2++;
				if (CtrlStep2>=10)
				{
					if (!isWallRight)
					{
						eMove = TURN_RIGHT;
						CtrlStep=0;
					}
					else if (!isWallFront)
					{
						eMove = FORWARD;
					}
					else
						eMove = TURN_BACK;
				}
				break;
			}
			}
			break;
		}
		case TURN_RIGHT:
		{
			LED1_OFF();LED2_OFF();LED3_ON();

			if (CtrlStep<14)
			{
				speed_set(MOTOR_RIGHT, 200);
				speed_set(MOTOR_LEFT, 200);
				CtrlStep++;
			}
			else
			{
				speed_set(MOTOR_RIGHT, 120);
				speed_set(MOTOR_LEFT, 200);
				CtrlStep2=0;
			}

			switch (e_wall_follow_select)
			{
			case WALL_FOLLOW_RIGHT:
			{
				if (isWallRight)
				{
					if (!isWallFront)
						eMove = FORWARD;
					else if (!isWallLeft)
					{
						eMove = TURN_LEFT;
						CtrlStep=0;
					}
					else
						eMove = TURN_BACK;
				}
				break;
			}
			case WALL_FOLLOW_LEFT:
			{
				CtrlStep2++;
				if (CtrlStep2>=10)
				{
					if (!isWallLeft)
					{
						eMove = TURN_LEFT;
						CtrlStep=0;
					}
					else if (!isWallFront)
					{
						eMove = FORWARD;
					}
					else
						eMove = TURN_BACK;
				}
				break;
			}
			}
			break;
		}
		case TURN_BACK:
		{
			LED1_ON();LED2_ON();LED3_ON();
			speed_set(MOTOR_RIGHT, 100);
			speed_set(MOTOR_LEFT, -150);
			if (!isWallFront)
			{
				eMove = BACKWARD;
				CtrlStep3 = 0;
			}
			break;
		}
		case BACKWARD:
		{
			speed_set(MOTOR_RIGHT,-100);
			speed_set(MOTOR_LEFT,-100);
			CtrlStep3++;
			if (CtrlStep>=5)
				eMove = FORWARD;
			break;
		}
		}
		PrintStep++;
		if (PrintStep > (500 / ui32_msLoop))
		{
			PrintStep = 0;
		}
#ifdef _DEBUG_IR_
if (PrintStep == 0)
{
	bluetooth_print("IR: %5d %5d %5d %5d\r\n", IR_GetIrDetectorValue(0), IR_GetIrDetectorValue(1), IR_GetIrDetectorValue(2), IR_GetIrDetectorValue(3));
}
#endif

#ifdef _DEBUG_PID_
if (PrintStep == 0)
{
	pid_get_parameters(&pid_params);
	bluetooth_print("U Control: %3d.%03d\r\n", (int32_t)pid_params.u, (int32_t)(pid_params.u * 1000) % 1000);
}
#endif

//bluetooth_print("IR: %d, %d, %d, %d\r\n", IR_GetIrDetectorValue(0), IR_GetIrDetectorValue(1), IR_GetIrDetectorValue(2), IR_GetIrDetectorValue(3));
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
