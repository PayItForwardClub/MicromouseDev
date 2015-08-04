/*
 * WallFollow.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "WallFollow.h"

//#define TEST_TURNLEFT_MOVE1
//#define TEST_TURNLEFT_TURN
//#define TEST_TURNLEFT_MOVE2
//#define TEST_TURNRIGHT_MOVE1
//#define TEST_TURNRIGHT_TURN
//#define TEST_TURNRIGHT_MOVE2
//#define TEST_TURNBACK_FWD
//#define TEST_TURNBACK_TURN1
//#define TEST_TURNBACK_TURN2
//#define TEST_TURNBACK_BACKWARD


//#define _DEBUG_IR_
//#define _DEBUG_PID_
//#define _DEBUG_POS_

#define CELL_ENC 13500
#define AVG_SPEED_FWD 200
#define AVG_SPEED_BWD 120

PID_PARAMETERS pid_wall = {.Kp = 0.1, .Kd = 0.0, .Ki = 0.0009,
		.Ts = 0.020, .PID_Saturation = 200, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_posLeft = {.Kp = 0.06, .Kd = 0.0, .Ki = 0.2,
		.Ts = 0.020, .PID_Saturation = 300, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS pid_posRight = {.Kp = 0.035, .Kd = 0.0, .Ki = 0.12,
		.Ts = 0.020, .PID_Saturation = 250, .e_=0, .e__=0, .u_=0};

typedef enum{
	TURN_LEFT,
	TURN_RIGHT,
	TURN_BACK,
	FORWARD,
	BACKWARD,
	SIMPLE_FORWARD
}MOVE;

MOVE eMove=FORWARD;

static void pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;
static TIMER_ID pid_TimerID = INVALID_TIMER_ID;

static int32_t PrintStep=0;
static int32_t CtrlStep=1;

static int32_t robotX=0,robotY=0;
static int32_t posLeftTmp=0,posRightTmp=0;

static int32_t avrSpeedLeft,avrSpeedRight,avrSpeed=AVG_SPEED_FWD;
static int32_t turnPulse;
static int32_t fwdPulse,fwdPulse2;
static int32_t bwdPulseLeft,bwdPulseRight;

void pid_wallfollow_init()
{
	ui32_msLoop =  pid_wall.Ts * 1000;
	pid_Runtimeout(&pid_process_callback, ui32_msLoop);
}

static bool pid_wallfollow(float delta_IR_left, float delta_IR_right, float averageSpeed,
		WALL_FOLLOW_SELECT wall_follow_select)
{
	static float error, u;
	static WALL_FOLLOW_SELECT preSelect=WALL_FOLLOW_NONE;
	int32_t set_speed[2];

	if (preSelect!=WALL_FOLLOW_NONE)
		if (preSelect!=wall_follow_select)
			pid_reset(&pid_wall);

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

	u = pid_process(&pid_wall,error);
	set_speed[0] = averageSpeed + (int32_t)(u / 2);
	set_speed[1] = averageSpeed - (int32_t)(u / 2);

	speed_set(MOTOR_RIGHT, set_speed[0]);
	speed_set(MOTOR_LEFT, set_speed[1]);

	return true;
}

static void pid_process_callback(void)
{
	pid_TimerID = INVALID_TIMER_ID;
	ControlFlag = true;
}

static void setParams(MOVE move)
{
	switch (move)
	{
	case TURN_RIGHT:
		fwdPulse=9500;
		avrSpeedLeft=160;
		avrSpeedRight=40;
		turnPulse=9500;
		fwdPulse2=0;
		CtrlStep=1;
		break;
	case TURN_LEFT:
		fwdPulse=11000;
		avrSpeedRight=160;
		avrSpeedLeft=40;
		turnPulse=8000;
		fwdPulse2=0;
		CtrlStep=1;
		break;
	case TURN_BACK:
		fwdPulse = 7500;
		avrSpeedLeft=-70;
		avrSpeedRight=60;
		turnPulse = 8000;
		bwdPulseLeft = 7000;
		bwdPulseRight = 5000;
		CtrlStep=1;
		break;
	case FORWARD:

		break;
	}
}

static MOVE getMove(bool isWallLeft,bool isWallFront,bool isWallRight)
{
	if (((!isWallRight) && (!isWallLeft)) && (!isWallFront))
	{

		avrSpeed = 200;
		return SIMPLE_FORWARD;
	}
	if (e_wall_follow_select == WALL_FOLLOW_RIGHT)
	{
		if (!isWallRight)
		{
			setParams(TURN_RIGHT);
			return TURN_RIGHT;
		}
		else if (!isWallLeft)//we detect available left wall
			//before front wall is detected, so we must check for front wall in turn left
		{

			setParams(TURN_LEFT);
			return TURN_LEFT;
		}
		else if (isWallFront)
		{
			setParams(TURN_BACK);
			return TURN_BACK;
		}
	}
	else if (e_wall_follow_select == WALL_FOLLOW_LEFT)
	{
		if (!isWallLeft)
		{
			setParams(TURN_LEFT);
			return TURN_LEFT;
		}
		else if (!isWallRight)//we detect available left wall
			//before front wall is detected, so we must check for front wall in turn left
		{

			setParams(TURN_RIGHT);
			return TURN_RIGHT;
		}
		else if (isWallFront)
		{
			setParams(TURN_BACK);
			return TURN_BACK;
		}
	}
	//avrSpeed=AVG_SPEED_FWD/2;
	return FORWARD;
}
//clear pos truoc khi xai
bool move(int deltaLeft,int deltaRight,int velLeftMax, int velRightMax)
{
	static int origLeft, origRight;
	static bool done = true;

	int currentLeft=qei_getPosLeft();
	int currentRight=qei_getPosRight();

	if (done)
	{
		done=false;
		origLeft=currentLeft;
		origRight=currentRight;
	}

	if (abs(origLeft+deltaLeft-currentLeft)>500)
	{
		pid_posLeft.PID_Saturation = velLeftMax;
		speed_set(MOTOR_LEFT, pid_process(&pid_posLeft,origLeft+deltaLeft-currentLeft));
		pid_posRight.PID_Saturation = velRightMax;
		speed_set(MOTOR_RIGHT, pid_process(&pid_posRight,origRight+deltaRight-currentRight));
		done=false;
	}
	else
	{
		done = true;
		pid_reset(&pid_posLeft);
		pid_reset(&pid_posRight);
	}
#ifdef _DEBUG_MOVE
	bluetooth_print("move: %5d %5d %5d %5d\r\n",(int)left, (int)right,(int)pid_posLeft.u,(int)pid_posRight.u);
#endif

	return done;
}
void ClearPosition()
{
	qei_setPosLeft(0);
	qei_setPosRight(0);
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
		bool isWallLeft, isWallRight, isWallFrontLeft,isWallFrontRight;

		float leftError=(float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IRLeft;
		float rightError=(float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IRRight;

		pid_Runtimeout(&pid_process_callback, ui32_msLoop);
		ControlFlag = false;

		isWallLeft = IRLeft<IR_get_calib_value(IR_CALIB_MAX_LEFT);
		isWallRight = IRRight<IR_get_calib_value(IR_CALIB_MAX_RIGHT);
		isWallFrontLeft = IRFrontLeft<IR_get_calib_value(IR_CALIB_MAX_FRONT_LEFT);
		isWallFrontRight = IRFrontRight<IR_get_calib_value(IR_CALIB_MAX_FRONT_RIGHT);

		switch(eMove)
		{
		case SIMPLE_FORWARD:
		{
			LED1_ON();LED2_ON();LED3_ON();
			speed_set(MOTOR_RIGHT, avrSpeed);
			speed_set(MOTOR_LEFT, avrSpeed);
			eMove=getMove(isWallLeft,isWallFrontRight|isWallFrontLeft,isWallRight);
			break;
		}
		case FORWARD:
		{
			LED1_OFF();LED2_ON();LED3_OFF();

			if (avrSpeed<=AVG_SPEED_FWD-30)
				avrSpeed+=30;
			else if (avrSpeed>=AVG_SPEED_FWD+30)
				avrSpeed-=30;
			else
				avrSpeed=AVG_SPEED_FWD;

			if ((isWallLeft && (e_wall_follow_select==WALL_FOLLOW_LEFT)) ||
					(isWallRight && (e_wall_follow_select==WALL_FOLLOW_RIGHT)))
			{
				pid_wallfollow(leftError,rightError, avrSpeed,e_wall_follow_select);
			}
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}

			eMove=getMove(isWallLeft,isWallFrontRight|isWallFrontLeft,isWallRight);
			break;
		}
		case TURN_LEFT:
		{
			static int vt,vp;
			LED1_ON();LED2_OFF();LED3_OFF();
			switch (CtrlStep)
			{
			case 1:
				posLeftTmp=qei_getPosLeft();
				CtrlStep++;
				vt=1;
				vp=1;
			case 2://go straight
				if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) &&
						(IRFrontRight>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
						(IRFrontLeft>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
				{
					avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / AVG_SPEED_FWD)) / 2)
							+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
					if (isWallRight)
						pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_RIGHT);
					else
					{
						speed_set(MOTOR_RIGHT, avrSpeed);
						speed_set(MOTOR_LEFT, avrSpeed);
					}
					if ((!isWallRight) && (e_wall_follow_select==WALL_FOLLOW_RIGHT))
					{
						eMove=TURN_RIGHT;
						setParams(TURN_RIGHT);
					}
				}
				else
				{
					if ((!isWallFrontLeft) && (!isWallFrontRight) && (e_wall_follow_select==WALL_FOLLOW_RIGHT))
					{
						avrSpeed = (avrSpeed+AVG_SPEED_FWD)/2;
						eMove = FORWARD;
						pid_reset(&pid_wall);

					}
					else
					{
						CtrlStep++;
						ClearPosition();
					}

				}
				break;
			case 3://turn 90 degree
#ifdef TEST_TURNLEFT_MOVE1
				speed_Enable_Hbridge(false);
#endif
				if ((abs(qei_getPosLeft()) + abs(qei_getPosRight()) < 2*turnPulse/3) ||
						//truong hop nga 3
						// __| |
						// __  |
						//   | |
						((abs(qei_getPosLeft()) + abs(qei_getPosRight()) < turnPulse)
								&& (isWallFrontLeft|isWallFrontRight)))
				{
					speed_set(MOTOR_RIGHT, avrSpeedRight);
					speed_set(MOTOR_LEFT, -avrSpeedLeft);
					if((abs(qei_getPosRight())>(turnPulse*0.8*vp/8)) && (vp<9))
					{
						if (avrSpeedRight>=10)
							avrSpeedRight-=10;
						vp++;

					}
					if((abs(qei_getPosLeft())>(turnPulse*0.2*vt/8)) && (vt<9))
					{
						if (avrSpeedLeft>=2)
							avrSpeedLeft-=2;
						vt++;
					}
				}
				else
				{
					ClearPosition();
					CtrlStep++;
				}
				break;
			case 4:
#ifdef TEST_TURNLEFT_TURN
				speed_Enable_Hbridge(false);
#endif
				if (move(fwdPulse2,fwdPulse2,AVG_SPEED_BWD,AVG_SPEED_BWD))
				{
#ifdef TEST_TURNLEFT_MOVE2
				speed_Enable_Hbridge(false);
#endif
					eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
					pid_reset(&pid_wall);
					ClearPosition();
				}
				break;
			}
			break;
		}
		case TURN_RIGHT:
		{
			static int vt,vp;

			LED1_OFF();LED2_OFF();LED3_ON();
			switch (CtrlStep)
			{
			case 1:
				qei_setPosLeft(0);
				posLeftTmp=qei_getPosLeft();
				CtrlStep=2;
				vt=1;
				vp=1;
			case 2://go straight
				if (abs(qei_getPosLeft()-posLeftTmp)<fwdPulse &&
						(IRFrontRight>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
						(IRFrontLeft>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
				{
					avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / AVG_SPEED_FWD)) / 2)
							+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
					if (isWallLeft)
						pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_LEFT);
					else
					{
						speed_set(MOTOR_RIGHT, avrSpeed);
						speed_set(MOTOR_LEFT, avrSpeed);
					}
					if ((!isWallLeft) && (e_wall_follow_select==WALL_FOLLOW_LEFT))
					{
						setParams(TURN_LEFT);
						eMove = TURN_LEFT;
					}
				}
				else
				{
					if ((!isWallFrontLeft) && (!isWallFrontRight) && (e_wall_follow_select==WALL_FOLLOW_LEFT))
					{
						avrSpeed = (avrSpeed+AVG_SPEED_FWD)/2;
						eMove = FORWARD;
						pid_reset(&pid_wall);
					}
					CtrlStep=3;
					ClearPosition();
				}
				break;
			case 3://turn 90 degree
#ifdef TEST_TURNRIGHT_MOVE1
				speed_Enable_Hbridge(false);
#endif
				if ((abs(qei_getPosLeft()) + abs(qei_getPosRight()) < 2*turnPulse/3) ||
						//truong hop nga 3
						// | |__
						// |  __
						// | |
						((abs(qei_getPosLeft()) + abs(qei_getPosRight()) < turnPulse)
								&& (isWallFrontLeft|isWallFrontRight)))
				{
					speed_set(MOTOR_LEFT, avrSpeedLeft);
					speed_set(MOTOR_RIGHT, -avrSpeedRight);
					if((abs(qei_getPosLeft())>(turnPulse*0.8*vp/8)) && (vp<9))
					{
						if (avrSpeedLeft>=10)
							avrSpeedLeft-=10;
						vp++;
					}
					if((abs(qei_getPosRight())>(turnPulse*0.2*vt/8)) && (vt<9))
					{
						if (avrSpeedLeft>=2)
							avrSpeedLeft-=2;
						vt++;
					}
				}
				else
				{
					ClearPosition();
					CtrlStep=4;

				}
				break;
			case 4://go straight
#ifdef TEST_TURNRIGHT_TURN
				speed_Enable_Hbridge(false);
#endif
				if (move(fwdPulse2,fwdPulse2,AVG_SPEED_BWD,AVG_SPEED_BWD))
				{
#ifdef TEST_TURNRIGHT_MOVE2
				speed_Enable_Hbridge(false);
#endif
					eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
					pid_reset(&pid_wall);
					ClearPosition();
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
				ClearPosition();
				posLeftTmp = qei_getPosLeft();
				CtrlStep++;
			}
			case 2://go forward a litte bit
			{
				if ((move(fwdPulse,fwdPulse,AVG_SPEED_FWD,AVG_SPEED_FWD))||
						(IRFrontRight<IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))||
						(IRFrontLeft<IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
				{
					CtrlStep++;
					ClearPosition();
				}
				break;
			}
			case 3:
			{
#ifdef TEST_TURNBACK_FWD
				speed_Enable_Hbridge(false);
#endif
				if ((abs(qei_getPosLeft())+abs(qei_getPosRight()))<turnPulse)
				{
					speed_set(MOTOR_RIGHT, avrSpeedRight);
					speed_set(MOTOR_LEFT, avrSpeedLeft);
				}
				else
				{
					CtrlStep++;
					ClearPosition();
				}
				break;
			}
			case 4:
			{
#ifdef TEST_TURNBACK_TURN1
				speed_Enable_Hbridge(false);
#endif
				LED1_OFF();LED2_OFF();LED3_OFF();
				if (((abs(qei_getPosLeft())+abs(qei_getPosRight()))<turnPulse) &&
						(isWallFrontLeft|isWallFrontRight))
				{
					speed_set(MOTOR_RIGHT, -avrSpeedLeft);
					speed_set(MOTOR_LEFT, -avrSpeedRight);
				}
				else
				{
					CtrlStep++;
					ClearPosition();
				}
				break;
			}

			case 5:
			{
#ifdef TEST_TURNBACK_TURN2
				speed_Enable_Hbridge(false);
#endif
				bluetooth_print("3 %d %d\r\n", (int32_t)qei_getPosLeft(), (int32_t)qei_getPosRight());
				if (move(-bwdPulseLeft,-bwdPulseRight,AVG_SPEED_BWD+20,AVG_SPEED_BWD-20))
				{
#ifdef TEST_TURNBACK_BACKWARD
					speed_Enable_Hbridge(false);
#endif
					eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
					pid_reset(&pid_wall);
					ClearPosition();
					speed_set(MOTOR_RIGHT, AVG_SPEED_FWD);
					speed_set(MOTOR_LEFT, AVG_SPEED_FWD);
				}
				break;
			}
			}
			break;
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

#ifdef _DEBUG_POS_

			if (PrintStep == 0)
			{
				bluetooth_print("pos: %3d.%03d\r\n", (int32_t)pid_params.u, (int32_t)(pid_params.u * 1000) % 1000);
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

