/*
 * WallFollow.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "WallFollow.h"

#define _DEBUG_IR_
//#define _DEBUG_PID_

#define CELL_ENC 13500
#define TURN_STEP1_MIN 4500//~ 1/2 CELL_ENC (led ngang coc ->banh ngang coc:4300)
#define TURN_STEP1_MAX 6500//~ 1/2 CELL_ENC (led ngang coc ->banh ngang coc:4300)
#define TURN_STEP2 10000//rotate ~ 1/2 CELL EMC * PI/2
//#define TURN_STEP2 4000
#define TURN_STEP3 3000

#define TURN_BACK_STEP1 8000

#define TURN_SPEED_MAX 150
#define TURN_SPEED_MIN 60
#define TURN_SPEED_BACK 100
#define AVG_SPEED_TURN 150
#define AVG_SPEED_FWD 200

static void Pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;

static TIMER_ID pid_TimerID = INVALID_TIMER_ID;
static int32_t PrintStep=0;
static int32_t CtrlStep=1;

static int32_t robotX=0,robotY=0;

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

static bool pid_wallfollow(float delta_IR_left, float delta_IR_right, float averageSpeed,
		WALL_FOLLOW_SELECT wall_follow_select)
{
	static float error, u;
	static WALL_FOLLOW_SELECT preSelect=WALL_FOLLOW_NONE;
	int32_t set_speed[2];

	if (preSelect!=WALL_FOLLOW_NONE)
		if (preSelect!=wall_follow_select)
			pid_reset();

	switch (wall_follow_select)
	{
	case WALL_FOLLOW_NONE:	//Do nothing
		return true;
	case WALL_FOLLOW_LEFT:
		error = -delta_IR_left;
		break;
	case WALL_FOLLOW_RIGHT:
		error = delta_IR_right;
		break;
	case WALL_FOLLOW_BOTH:
		error = delta_IR_right - delta_IR_left;
		break;
	default:
		return false;
	}

	preSelect = wall_follow_select;

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
void getNextMove(bool isWallLeft,bool isWallRight,bool  isWallFront)
{
	switch (e_wall_follow_select)
	{
	case WALL_FOLLOW_LEFT:
	{
		if (!isWallLeft)
		{
			eMove = TURN_LEFT;
		}
		else if (!isWallFront)
			eMove = FORWARD;
		else if (!isWallRight)
		{
			eMove = TURN_RIGHT;

		}
		else
			eMove = TURN_BACK;
		break;
	}
	case WALL_FOLLOW_RIGHT:
	{
		if (!isWallRight)
		{
			bluetooth_print("R\r\n");
			eMove = TURN_RIGHT;
		}
		else if (!isWallFront)
			eMove = FORWARD;
		else if (!isWallLeft)
		{
			bluetooth_print("L\r\n");
			eMove = TURN_LEFT;
		}
		else
		{
			bluetooth_print("B\r\n");
			eMove = TURN_BACK;
		}
		break;
	}
	}

	if ((eMove==TURN_LEFT)||(eMove==TURN_RIGHT)||(eMove==TURN_BACK))
	{
		bluetooth_print("IR: %5d %5d %5d %5d\r\n", IR_GetIrDetectorValue(0), IR_GetIrDetectorValue(1)
				, IR_GetIrDetectorValue(2), IR_GetIrDetectorValue(3));
		qei_setPosLeft(0);
		qei_setPosRight(0);
		CtrlStep=1;
	}
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
		float leftError=(float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IRLeft;
		float rightError=(float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IRRight;

		pid_Runtimeout(&Pid_process_callback, ui32_msLoop);
		ControlFlag = false;

		isWallLeft = (IRLeft<(IR_get_calib_value(IR_CALIB_MAX_LEFT)+100));
		isWallRight = (IRRight<(IR_get_calib_value(IR_CALIB_MAX_RIGHT)+100));
		isWallFront = ((IRFrontLeft<IR_get_calib_value(IR_CALIB_MIN_FRONT_LEFT))
				|| (IRFrontRight<IR_get_calib_value(IR_CALIB_MIN_FRONT_RIGHT)));


		switch(eMove)
		{
		case FORWARD:
		{
			LED1_OFF();LED2_OFF();LED3_OFF();
			pid_wallfollow(leftError,rightError, AVG_SPEED_FWD,e_wall_follow_select);
			getNextMove(isWallLeft,isWallRight,isWallFront);
			break;
		}
		case TURN_LEFT:
		{
			//LED1_ON();LED2_OFF();LED3_OFF();
			switch (CtrlStep)
			{
			case 1:

				if (qei_getPosRight()+qei_getPosLeft()<=2*TURN_STEP1_MIN)//turn left (ignore right corner)
				{
					if (isWallRight)
					{

						pid_wallfollow(leftError,rightError,AVG_SPEED_TURN,WALL_FOLLOW_RIGHT);
					}
					else
					{
						speed_set(MOTOR_RIGHT, AVG_SPEED_TURN);
						speed_set(MOTOR_LEFT, AVG_SPEED_TURN);
					}
				}
				else
				{
					CtrlStep++;
					qei_setPosRight(0);
					qei_setPosLeft(0);
				}
				break;
			case 2:

				if (qei_getPosRight()+qei_getPosLeft()<=TURN_STEP2)//ignore right corner
				{
					speed_set(MOTOR_RIGHT, TURN_SPEED_MAX);
					speed_set(MOTOR_LEFT, TURN_SPEED_MIN);
				}
				else
				{
					if (isWallRight && (!isWallFront))
					{
						getNextMove(isWallLeft,isWallRight,isWallFront);
					}
					else//keep rotating until we find right wall
					{
						speed_set(MOTOR_RIGHT, TURN_SPEED_MAX);
						speed_set(MOTOR_LEFT, TURN_SPEED_MIN);
					}
				}
				break;
			}
			break;
		}
		case TURN_RIGHT:
		{

			switch (CtrlStep)
			{
			case 1:
				LED1_ON();LED2_OFF();LED3_OFF();
				if (qei_getPosRight()+qei_getPosLeft()<=2*TURN_STEP1_MAX)//turn left (ignore right corner)
				{
					if (isWallLeft)
					{
						LED1_OFF();LED2_ON();LED3_OFF();
						pid_wallfollow(leftError,rightError,AVG_SPEED_TURN,WALL_FOLLOW_LEFT);
					}
					else
					{
						speed_set(MOTOR_RIGHT, AVG_SPEED_TURN);
						speed_set(MOTOR_LEFT, AVG_SPEED_TURN);
					}
				}
				else
				{
					CtrlStep++;
					qei_setPosRight(0);
					qei_setPosLeft(0);
				}
				break;
			case 2:
				LED1_OFF();LED2_OFF();LED3_ON();
				if (qei_getPosRight()+qei_getPosLeft()<=TURN_STEP2)//ignore right corner
				{
					speed_set(MOTOR_LEFT, TURN_SPEED_MAX);
					speed_set(MOTOR_RIGHT, TURN_SPEED_MIN);
				}
				else
				{
					if (isWallRight)
					{
						getNextMove(isWallLeft,isWallRight,isWallFront);
					}
					else
					{
						speed_set(MOTOR_LEFT, TURN_SPEED_MAX);
						speed_set(MOTOR_RIGHT, TURN_SPEED_MIN);
					}
				}
				break;
			}

			break;
		}
		case TURN_BACK:
		{
			LED1_ON();LED2_ON();LED3_ON();
			switch (CtrlStep)
			{
			case 1:
			{
				if (qei_getPosRight()+qei_getPosLeft()<=2*TURN_BACK_STEP1)
				{
					pid_wallfollow(leftError,rightError,AVG_SPEED_TURN,WALL_FOLLOW_RIGHT);
				}
				else
				{
					CtrlStep++;
					qei_setPosLeft(0);
					qei_setPosRight(0);
				}
				break;
			}
			case 2:
			{
				if (qei_getPosRight()<=4000)
				{
					if (isWallRight && (!isWallFront))
						getNextMove(isWallLeft,isWallRight,isWallFront);
					else
					{
						speed_set(MOTOR_RIGHT, TURN_SPEED_BACK-30);
						speed_set(MOTOR_LEFT, -TURN_SPEED_BACK-30);
					}
				}
				else
				{
					if (isWallRight && (!isWallFront))
						getNextMove(isWallLeft,isWallRight,isWallFront);
					else
					{
						speed_set(MOTOR_RIGHT, TURN_SPEED_BACK+30);
						speed_set(MOTOR_LEFT, -TURN_SPEED_BACK+30);
					}
				}
			}
			}
			break;
		}
		case BACKWARD:
		{

		}
		}
		PrintStep++;
		if (PrintStep > (200 / ui32_msLoop))
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

