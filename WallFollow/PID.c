/*
 * PID.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "PID.h"

//static PID_PARAMETERS pid_parameter;

void pid_set_k_params(PID_PARAMETERS* pid_parameter,float Kp,float Ki, float Kd)
{
	pid_parameter->Kp = Kp;
	pid_parameter->Ki = Ki;
	pid_parameter->Kd = Kd;
}
float pid_process(PID_PARAMETERS* pid_parameter,float error)
{
	pid_parameter->e__ = pid_parameter->e_;
	pid_parameter->e_ = pid_parameter->e;
	pid_parameter->e = error;
	pid_parameter->u_ = pid_parameter->u;
	pid_parameter->u = pid_parameter->u_ + pid_parameter->Kp * (pid_parameter->e - pid_parameter->e_)
			+ pid_parameter->Ki * pid_parameter->Ts * pid_parameter->e
			+ (pid_parameter->Kd / pid_parameter->Ts) * (pid_parameter->e - (2 * pid_parameter->e_) + pid_parameter->e__);

	if (pid_parameter->u > pid_parameter->PID_Saturation)
	{
		pid_parameter->u = pid_parameter->PID_Saturation;
	}
	else if (pid_parameter->u < (-pid_parameter->PID_Saturation))
	{
		pid_parameter->u = -pid_parameter->PID_Saturation;
	}

	return pid_parameter->u;
}
void pid_reset(PID_PARAMETERS* pid_parameter)
{
	pid_parameter->e = 0;
	pid_parameter->e_ = 0;
	pid_parameter->e__ = 0;
	pid_parameter->u = 0;
	pid_parameter->u_ = 0;
}



