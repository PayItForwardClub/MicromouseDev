/*
 * WallFollow.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "WallFollow.h"

//#define TEST_FORWARD_MOVE
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
#define _DEBUG_POS_

#define CELL_ENC 13500
#define AVG_SPEED_FWD 200
#define AVG_SPEED_FWD_SLOW 120
#define AVG_SPEED_BWD 120

PID_PARAMETERS pid_wall = {.Kp = 0.1, .Kd = 0.0, .Ki = 0.1,
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
	SIMPLE_FORWARD,
	NONE
}MOVE;

typedef enum{
	UP_DIR=0,
	RIGHT_DIR=1,
	DOWN_DIR=2,
	LEFT_DIR=3
}DIRECTION;

static DIRECTION currentDir=UP_DIR;
static MOVE eMove=FORWARD;

static void pid_process_callback(void);
static void pid_StopTimeout(void);
static TIMER_ID pid_Runtimeout(TIMER_CALLBACK_FUNC CallbackFcn, uint32_t msTime);
static WALL_FOLLOW_SELECT e_wall_follow_select = WALL_FOLLOW_NONE;
static bool ControlFlag = false;
static uint32_t ui32_msLoop = 0;
static TIMER_ID pid_TimerID = INVALID_TIMER_ID;

static int32_t PrintStep=0;
static int32_t CtrlStep=1;
static int32_t moveStage=1;
static int32_t robotX=0,robotY=0;
static int32_t posLeftTmp=0,posRightTmp=0;
static int32_t encLeftTmp=0;

//static int32_t avrSpeedLeft,avrSpeedRight;
static int32_T avrSpeed, avrSpeedTmp;

static bool isWallLeft, isWallRight, isWallFrontLeft,isWallFrontRight;
static bool rqTurnLeft=false,rqTurnRight=false;
static float leftError, rightError;

static void clearPosition()
{
	encLeftTmp=0;
	//encRightTmp=0;
	qei_setPosLeft(9000);//head of robot
	qei_setPosRight(9000);
}
static void initPos()
{
	robotX=0;
	robotY=0;
	clearPosition();
}

static void pid_Wallfollow_init()
{
	ui32_msLoop =  pid_wall.Ts * 1000;
	pid_Runtimeout(&pid_process_callback, ui32_msLoop);

}
void wallFollow_init()
{
	pid_Wallfollow_init();
	initPos();
	eMove=FORWARD;
	currentDir=UP_DIR;
	avrSpeed=AVG_SPEED_FWD;
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
//*****************************************************************************
//
//! Update robot position when moving in straight line
//!
//! \param none
//!
//!
//! \return none
//
//*****************************************************************************
static void forwardUpdate()
{
	if (qei_getPosLeft()-encLeftTmp>CELL_ENC)
	{
		encLeftTmp += CELL_ENC;
		switch (currentDir)
		{
		case 0:
			robotY++;
			break;
		case 1:
			robotX++;
			break;
		case 2:
			robotY--;
			break;
		case 3:
			robotX--;
			break;
		}
#ifdef _DEBUG_POS_
		bluetooth_print("pos %d %d %d",robotX,robotY,currentDir);
#endif
	}
}
//*****************************************************************************
//
//! Update robot position, modify it to get better precision
//!
//! \param none
//!
//!
//! \return none
//
//*****************************************************************************
static void updatePos()
{
	switch(eMove)
	{
	case FORWARD:
	{
		forwardUpdate();
		break;
	}
	case TURN_LEFT:
	{
		switch (CtrlStep)
		{
		case 2:
			forwardUpdate();
			break;
		case 4:
			currentDir=(currentDir+3)%4;
			clearPosition();
			qei_setPosLeft(16500);//head of robot
			qei_setPosRight(16500);
			forwardUpdate();
#ifdef _DEBUG_POS_
		bluetooth_print("pos %d %d %d",robotX,robotY,currentDir);
#endif
			break;
		}
		break;
	}
	case TURN_RIGHT:
	{
		switch (CtrlStep)
		{
		case 2:
			forwardUpdate();
			break;
		case 4:
			currentDir=(currentDir+1)%4;
			clearPosition();
			qei_setPosLeft(16500);
			qei_setPosRight(16500);
			forwardUpdate();
#ifdef _DEBUG_POS_
		bluetooth_print("pos %d %d %d",robotX,robotY,currentDir);
#endif
			break;
		}
		break;
	}
	case TURN_BACK:
	{
		switch (CtrlStep)
		{
		case 2:
			forwardUpdate();
			break;
		case 4:
			currentDir=(currentDir+3)%4;
#ifdef _DEBUG_POS_
		bluetooth_print("pos %d %d %d",robotX,robotY,currentDir);
#endif
			break;
		case 6:
			currentDir=(currentDir+3)%4;
#ifdef _DEBUG_POS_
		bluetooth_print("pos %d %d %d",robotX,robotY,currentDir);
#endif
			break;
		}
	}
	}

}
static int getRobotX()
{
	return robotX;
}
static int getRobotY()
{
	return robotY;
}
static DIRECTION getCurrentDir()
{
	return currentDir;
}
//add your algorithm here
//using getCurrentDir and getRobotX, getRobotY
static MOVE getMove(bool wallLeft,bool wallFront,bool wallRight)
{
	if ((!wallLeft) && (!wallFront) && (!wallRight))
	{
		avrSpeed = AVG_SPEED_FWD;
		return SIMPLE_FORWARD;
	}
	if (e_wall_follow_select == WALL_FOLLOW_RIGHT)
	{
		if (!wallRight)
		{
			return TURN_RIGHT;
		}
		else if (!wallFront)
		{
			return FORWARD;
		}
		else if (!wallLeft)
		{
			return TURN_LEFT;
		}
		else
		{
			return TURN_BACK;
		}
	}
	else if (e_wall_follow_select == WALL_FOLLOW_LEFT)
	{
		if (!wallLeft)
		{

			return TURN_LEFT;
		}
		else if (!wallFront)
		{
			return FORWARD;
		}
		else if (!wallRight)
		{

			return TURN_RIGHT;
		}
		else
		{
			return TURN_BACK;
		}
	}
	return FORWARD;
}

//*****************************************************************************
//
//! Move robot a little bit (error +-500 pulses). Robot velocity is equal 0 after moving.
//! Make sure this function return true before calling it again.
//!
//! \param deltaLeft distance left motor will go
//! \param deltaRight distance right motor will go
//! \param velLeftMax max left velocity
//! \param velRightMax max right velocity
//!
//! \return true if done
//
//*****************************************************************************
static bool move(int deltaLeft,int deltaRight,int velLeftMax, int velRightMax)
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
	//bluetooth_print("move: %5d %5d\r\n",origLeft,currentLeft);
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

//*****************************************************************************
//
//! Control two motor to make robot turn right 90 degree
//!
//! \param fwdPulse is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param avrSpeedLeft is the speed of left motor.
//! \param avrSpeedRight is the speed of right motor.
//! \param turnPulse is the total pulse of two encoder after turn
//!
//! \return true if finish
//!			false if not
//
//*****************************************************************************
static bool TurnRight(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse)
{
	static int vt,vp;
	LED1_OFF();LED2_OFF();LED3_ON();
//	bluetooth_print("RS %d\r\n",CtrlStep);
	switch (CtrlStep)
	{
	case 1:
		posLeftTmp=qei_getPosLeft();
		CtrlStep=2;
		vt=1;
		vp=1;
		avrSpeedTmp=avrSpeed;
	case 2://go straight

		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) ||
				((isWallFrontLeft|isWallFrontRight)&&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT))))
		{
			if (qei_getPosLeft()<fwdPulse+posLeftTmp)
				avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
				+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			else
				avrSpeed = (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			if (isWallLeft)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_LEFT);
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}
		}
		else
		{
#ifdef TEST_TURNRIGHT_MOVE1
		speed_Enable_Hbridge(false);
#endif
			pid_reset(&pid_wall);
			updatePos();
			CtrlStep++;
		}
		break;
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4://turn 90 degree
		if (abs(qei_getPosLeft()-posLeftTmp) + abs(qei_getPosRight()-posRightTmp) < turnPulse)
		{
			speed_set(MOTOR_LEFT, avrSpeedLeft);
			speed_set(MOTOR_RIGHT, -avrSpeedRight);
			if((abs(qei_getPosLeft()-posLeftTmp)>(turnPulse*0.8*vp/8)) && (vp<9))
			{
				if (avrSpeedLeft>=10)
					avrSpeedLeft-=10;
				vp++;
			}
			if((abs(qei_getPosRight()-posRightTmp)>(turnPulse*0.2*vt/8)) && (vt<9))
			{
				if (avrSpeedLeft>=2)
					avrSpeedLeft-=2;
				vt++;
			}
		}
		else
		{
#ifdef TEST_TURNRIGHT_TURN
		speed_Enable_Hbridge(false);
#endif
			updatePos();
			CtrlStep=1;
			pid_reset(&pid_wall);
			return true;
		}
		break;
	}
	return false;
}
//*****************************************************************************
//
//! Control two motor to make robot turn left 90 degree
//!
//! \param fwdPulse is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param avrSpeedLeft is the speed of left motor.
//! \param avrSpeedRight is the speed of right motor.
//! \param turnPulse is the total pulse of two encoder after turn
//!
//! \return true if finish
//!			false if not
//
//*****************************************************************************
static bool TurnLeft(int fwdPulse,int avrSpeedLeft,int avrSpeedRight,int turnPulse)
{
	static int vt,vp;
	LED1_ON();LED2_OFF();LED3_OFF();
//	bluetooth_print("LS %d\r\n",CtrlStep);
	switch (CtrlStep)
	{
	case 1:
		posLeftTmp=qei_getPosLeft();
		CtrlStep++;
		vt=1;
		vp=1;
		avrSpeedTmp=avrSpeed;
	case 2://go straight
		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) ||
				((isWallFrontLeft|isWallFrontRight)&&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT))))
		{
			if (qei_getPosLeft()<fwdPulse+posLeftTmp)
				avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
					+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			else
				avrSpeed = (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;

			if (isWallRight)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_RIGHT);
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}
		}
		else
		{
#ifdef TEST_TURNLEFT_MOVE1
		speed_Enable_Hbridge(false);
#endif
			pid_reset(&pid_wall);
			updatePos();
			CtrlStep++;
		}
		break;
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4://turn 90 degree

		if (abs(qei_getPosLeft()-posLeftTmp) + abs(qei_getPosRight()-posRightTmp) < turnPulse)

		{
			speed_set(MOTOR_RIGHT, avrSpeedRight);
			speed_set(MOTOR_LEFT, -avrSpeedLeft);
			if((abs(qei_getPosRight()-posRightTmp)>(turnPulse*0.8*vp/8)) && (vp<9))
			{
				if (avrSpeedRight>=10)
					avrSpeedRight-=10;
				vp++;

			}
			if((abs(qei_getPosLeft()-posLeftTmp)>(turnPulse*0.2*vt/8)) && (vt<9))
			{
				if (avrSpeedLeft>=2)
					avrSpeedLeft-=2;
				vt++;
			}
		}
		else
		{
#ifdef TEST_TURNLEFT_TURN
		speed_Enable_Hbridge(false);
#endif
			updatePos();
			CtrlStep=1;
			pid_reset(&pid_wall);
			return true;
		}
		break;
	}
	return false;

}
//*****************************************************************************
//
//! Control two motor to make robot turn back 180 degree.
//!
//! \param fwdPulse is the distance robot will go straight before turn right
//!, the robot will stand between the next cell of maze.
//! \param avrSpeedLeft is the speed of left motor.
//! \param avrSpeedRight is the speed of left motor.
//! \param NumPulse is the total pulse of two encoder after turn
//!
//! \return true if finish
//!			false if not
//
static bool TurnBack(int fwdPulse, int avrSpeedLeft,int avrSpeedRight,int turnPulse
)
{
	LED1_ON();LED2_ON();LED3_ON();
	switch (CtrlStep)
	{
	case 1:
	{
		posLeftTmp = qei_getPosLeft();
		avrSpeedTmp = avrSpeed;
		CtrlStep++;
	}
	case 2://go forward a litte bit
	{

		if ((abs(qei_getPosLeft()-posLeftTmp)<fwdPulse) &&
				(IR_GetIrDetectorValue(3)>IR_get_calib_value(IR_CALIB_BASE_FRONT_RIGHT))&&
				(IR_GetIrDetectorValue(0)>IR_get_calib_value(IR_CALIB_BASE_FRONT_LEFT)))
		{
			avrSpeed = ((abs(fwdPulse + posLeftTmp - qei_getPosLeft()) / (fwdPulse / avrSpeedTmp)) / 2)
					+ (abs(avrSpeedLeft) + abs(avrSpeedRight)) / 2;
			if (isWallRight)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_RIGHT);
			else if (isWallLeft)
				pid_wallfollow(leftError,rightError,avrSpeed,WALL_FOLLOW_LEFT);
			else
			{
				speed_set(MOTOR_RIGHT, avrSpeed);
				speed_set(MOTOR_LEFT, avrSpeed);
			}
		}
		else
		{
			pid_reset(&pid_wall);
			updatePos();
			CtrlStep++;
		}
		break;
	}
	case 3:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 4://turing 90 degree
	{
#ifdef TEST_TURNBACK_FWD
		speed_Enable_Hbridge(false);
#endif
		if ((abs(qei_getPosLeft()-posLeftTmp)+abs(qei_getPosRight()-posRightTmp))<turnPulse)
		{
			speed_set(MOTOR_RIGHT, avrSpeedRight);
			speed_set(MOTOR_LEFT, avrSpeedLeft);
		}
		else
		{
			updatePos();
			CtrlStep++;
		}
		break;
	}
	case 5:
		posLeftTmp=qei_getPosLeft();
		posRightTmp=qei_getPosRight();
		CtrlStep++;
	case 6://turning another 90 degree
	{
#ifdef TEST_TURNBACK_TURN1
		speed_Enable_Hbridge(false);
#endif

		if ((abs(qei_getPosLeft()-posLeftTmp)+abs(qei_getPosRight()-posRightTmp))<turnPulse)
		{
			speed_set(MOTOR_RIGHT, -avrSpeedLeft);
			speed_set(MOTOR_LEFT, -avrSpeedRight);
		}
		else
		{
#ifdef TEST_TURNBACK_TURN2
		speed_Enable_Hbridge(false);
#endif
			updatePos();
			CtrlStep=1;
			return true;
		}
		break;
	}
	}
	return false;
}
//*****************************************************************************
//
//! Control robot to go straight forward by following wall
//!
//!
//! \return true if left/right wall is detected
//!			false if no left/right wall is detected
//
static bool Forward()
{
	LED1_OFF();LED2_ON();LED3_OFF();
	updatePos();
	//bluetooth_print("%d\r\n",avrSpeed);
	if ((!isWallLeft) || (!isWallRight))
		return true;

	if (avrSpeed<AVG_SPEED_FWD-30)
		avrSpeed+=30;
	else
		avrSpeed=AVG_SPEED_FWD;

	if (isWallRight)
	{
		pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_RIGHT);
	}
	else if (isWallLeft)
	{
		pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_LEFT);
	}
	else
	{
		speed_set(MOTOR_RIGHT, avrSpeed);
		speed_set(MOTOR_LEFT, avrSpeed);
	}

	return false;
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
		static int i;
		pid_Runtimeout(&pid_process_callback, ui32_msLoop);
		ControlFlag = false;

		leftError=(float)IR_get_calib_value(IR_CALIB_BASE_LEFT) - (float)IR_GetIrDetectorValue(1);
		rightError=(float)IR_get_calib_value(IR_CALIB_BASE_RIGHT) - (float)IR_GetIrDetectorValue(2);
		isWallLeft = IR_GetIrDetectorValue(1)<IR_get_calib_value(IR_CALIB_MAX_LEFT);
		isWallRight = IR_GetIrDetectorValue(2)<IR_get_calib_value(IR_CALIB_MAX_RIGHT);
		isWallFrontLeft = IR_GetIrDetectorValue(0)<IR_get_calib_value(IR_CALIB_MAX_FRONT_LEFT);
		isWallFrontRight = IR_GetIrDetectorValue(3)<IR_get_calib_value(IR_CALIB_MAX_FRONT_RIGHT);


		switch(eMove)
		{
		case SIMPLE_FORWARD:
			speed_set(MOTOR_RIGHT,AVG_SPEED_FWD);
			speed_set(MOTOR_LEFT,AVG_SPEED_FWD);
			if (isWallFrontRight|isWallFrontLeft|isWallRight|isWallLeft)
				eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
			break;
		case FORWARD:
			switch (moveStage)
			{
			case 1:
				if (Forward())
					moveStage++;
				if (isWallFrontLeft| isWallFrontRight)
					eMove=getMove(isWallLeft,isWallFrontLeft|isWallFrontRight,isWallRight);
				break;
			case 2:
				posLeftTmp=qei_getPosLeft();
				moveStage++;
				i=1;
				avrSpeedTmp=avrSpeed;
			case 3://slow down
				forwardUpdate();
				if (!isWallLeft)
				{
					rqTurnLeft=true;
				}
				if (!isWallRight)
				{
					rqTurnRight=true;
				}
				if ((abs(qei_getPosLeft()-posLeftTmp)<5000)
						&& (!isWallFrontLeft) && (!isWallFrontRight))
				{
					if (abs(qei_getPosLeft()-posLeftTmp)>i*1000)
					{
						avrSpeed -= 10;
						i++;
					}
					if (isWallRight)
					{
						pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_RIGHT);
					}
					else if (isWallLeft)
					{
						pid_wallfollow(leftError,rightError, avrSpeed,WALL_FOLLOW_LEFT);
					}
					else
					{
						speed_set(MOTOR_RIGHT, avrSpeed);
						speed_set(MOTOR_LEFT, avrSpeed);
					}
				}
				else
				{
#ifdef TEST_FORWARD_MOVE
					speed_Enable_Hbridge(false);
#endif

					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					if (eMove==FORWARD)
						avrSpeed=AVG_SPEED_FWD;
					rqTurnLeft=false;
					rqTurnRight=false;
					moveStage=1;
				}
				break;
			}
			break;

		case TURN_LEFT:
			switch (moveStage)
			{
			case 1:
				if (TurnLeft(4000,30,200,8500))
				{
					moveStage++;
				}
				break;
			case 2:
				posLeftTmp=qei_getPosLeft();
				moveStage++;
			case 3:
				//go straight again to check left/right wall
				forwardUpdate();
				if (abs(qei_getPosLeft()-posLeftTmp)<2000)
				{
					if (!isWallLeft)
						rqTurnLeft=true;
					if (!isWallRight)
						rqTurnRight=true;

					if (isWallRight)
						pid_wallfollow(leftError,rightError,AVG_SPEED_FWD_SLOW,WALL_FOLLOW_RIGHT);
					else if (isWallLeft)
						pid_wallfollow(leftError,rightError,AVG_SPEED_FWD_SLOW,WALL_FOLLOW_LEFT);
					else
					{
						speed_set(MOTOR_RIGHT, AVG_SPEED_FWD_SLOW);
						speed_set(MOTOR_LEFT, AVG_SPEED_FWD_SLOW);
					}
				}
				else
				{
#ifdef TEST_TURNLEFT_MOVE2
					speed_Enable_Hbridge(false);
#endif
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					rqTurnLeft=false;
					rqTurnRight=false;
					moveStage=1;
					pid_reset(&pid_wall);
				}
			}
			break;

		case TURN_RIGHT:
			switch (moveStage)
			{
			case 1:
				if (TurnRight(4000,200,30,8500))
				{
					moveStage++;
				}
				break;
			case 2:
				posLeftTmp=qei_getPosLeft();
				moveStage++;
			case 3:
				//go straight again to check left/right wall
				forwardUpdate();
				if (abs(qei_getPosLeft()-posLeftTmp)<2000)
				{
					if (!isWallLeft)
						rqTurnLeft=true;
					if (!isWallRight)
						rqTurnRight=true;

					if (isWallRight)
						pid_wallfollow(leftError,rightError,AVG_SPEED_FWD_SLOW,WALL_FOLLOW_RIGHT);
					else if (isWallLeft)
						pid_wallfollow(leftError,rightError,AVG_SPEED_FWD_SLOW,WALL_FOLLOW_LEFT);
					else
					{
						speed_set(MOTOR_RIGHT, AVG_SPEED_FWD_SLOW);
						speed_set(MOTOR_LEFT, AVG_SPEED_FWD_SLOW);
					}
				}
				else
				{
#ifdef TEST_TURNRIGHT_MOVE2
					speed_Enable_Hbridge(false);
#endif
					eMove=getMove(!rqTurnLeft,isWallFrontLeft|isWallFrontRight,!rqTurnRight);
					rqTurnLeft=false;
					rqTurnRight=false;
					moveStage=1;
					pid_reset(&pid_wall);
				}
			}
			break;
		case TURN_BACK:
			switch (moveStage)
			{
			case 1:
				if (TurnBack(7000,-80,60,9000))
				{
					//rotate more if we still detect front wall: do it yourself ;D
					moveStage++;
				}
				break;
			case 2:
				if (move(-7000,-7000,AVG_SPEED_BWD,AVG_SPEED_BWD))
				{
#ifdef TEST_TURNBACK_BACKWARD
					speed_Enable_Hbridge(false);
#endif
					clearPosition();
					avrSpeed = AVG_SPEED_FWD_SLOW;
					eMove=FORWARD;
					moveStage = 1;
				}
			}
			break;
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

