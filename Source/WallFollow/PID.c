/*
 * PID.c
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#include "include.h"
#include "PID.h"

static PID_PARAMETERS pid_parameter;

void pid_init(void)
{
	uint8_t au8_tempBuf[128];
	memset(&pid_parameter, 0, sizeof(PID_PARAMETERS));

	param_Get(PARAM_ID_PID_INFO, au8_tempBuf);
	
	memcpy(&pid_parameter, au8_tempBuf, sizeof(PID_PARAMETERS));
}

void pid_set_parameters(PID_PARAMETERS pid_param)
{
	uint8_t au8_tempBuf[128];
	
	memset(au8_tempBuf, 0, 150);
	
	pid_parameter = pid_param;
	
	memcpy(au8_tempBuf, (void *)&pid_param, sizeof(PID_PARAMETERS));
	param_Set(PARAM_ID_PID_INFO, au8_tempBuf);
}

float pid_process(float error)
{
	pid_parameter.e__ = pid_parameter.e_;
	pid_parameter.e_ = pid_parameter.e;
	pid_parameter.e = error;
	pid_parameter.u_ = pid_parameter.u;
	pid_parameter.u = pid_parameter.u_ + pid_parameter.Kp * (pid_parameter.e - pid_parameter.e_)
			+ pid_parameter.Ki * pid_parameter.Ts * pid_parameter.e
			+ (pid_parameter.Kd / pid_parameter.Ts) * (pid_parameter.e - (2 * pid_parameter.e_) + pid_parameter.e__);

	if (pid_parameter.u > pid_parameter.PID_Saturation)
	{
		pid_parameter.u = pid_parameter.PID_Saturation;
	}
	else if (pid_parameter.u < (-pid_parameter.PID_Saturation))
	{
		pid_parameter.u = -pid_parameter.PID_Saturation;
	}

	return pid_parameter.u;
}

void pid_get_parameters(PID_PARAMETERS *pid_param)
{
	*pid_param = pid_parameter;
}
