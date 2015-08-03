/*
 * PID.h
 *
 *  Created on: Jul 6, 2015
 *      Author: NHH
 */

#ifndef WALLFOLLOW_PID_H_
#define WALLFOLLOW_PID_H_

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float u;
	float u_;
	float e;
	float e_;
	float e__;
	float Ts;
	float PID_Saturation;
} PID_PARAMETERS;

extern void pid_set_parameters(PID_PARAMETERS pid_param);
//extern bool pid_wallfollow(float delta_IR_left, float delta_IR_right, float averageSpeed);
float pid_process(PID_PARAMETERS* pid_parameter,float error);

extern float pid_get_error();
extern void pid_reset();
extern void pid_set_k_params(PID_PARAMETERS* pid_parameters,float Kp,float Ki, float Kd);

extern void pid_get_parameters(PID_PARAMETERS *pid_param);

#endif /* WALLFOLLOW_PID_H_ */
