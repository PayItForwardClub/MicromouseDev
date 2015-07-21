/*
 * PID.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "../include.h"
#include "PID.h"

static PID_PARAMETERS pid_parameter;

void pid_init(void)
{
	memset(&pid_parameter, 0, sizeof(PID_PARAMETERS));

}

void pid_set_parameters(PID_PARAMETERS pid_param)
{
	pid_parameter = pid_param;
}
void pid_set_k_params(float Kp,float Ki, float Kd)
{
	pid_parameter.Kp = Kp;
	pid_parameter.Ki = Ki;
	pid_parameter.Kd = Kd;
}
float pid_process(float error)
{
	pid_parameter.e__ = pid_parameter.e_;
	pid_parameter.e_ = pid_parameter.e;
	pid_parameter.e = error;
	pid_parameter.u_ = pid_parameter.u;
	pid_parameter.u = pid_parameter.u_ + pid_parameter.Kp * (pid_parameter.e - pid_parameter.e_)
			+ pid_parameter.Ki * pid_parameter.Ts * pid_parameter.e
			+ (pid_parameter.Kd / pid_parameter.Ts) * (pid_parameter.e - 2 * pid_parameter.e_ + pid_parameter.e__);

	return pid_parameter.u;
}

float pid_get_error()
{
	return pid_parameter.e;
}
