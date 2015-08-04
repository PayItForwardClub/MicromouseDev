/*
 * STR_Indirect.c
 *
 * Code generation for function 'STR_Indirect'
 *
 * C source code generated on: Tue Apr 29 23:32:21 2014
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "STR_Indirect.h"
#include "define.h"
#include "speed_control.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
real_T STR_Indirect(MOTOR_SELECT Motor_Sel, real_T Theta[4], real_T uc, real_T y)
{
  static real_T u;
  static real_T u_[2] = {0, 0};
  static real_T uc_[2] = {0, 0}, y_[2] = {0, 0};

	if (Theta[2] != 0)
	{
		if (Motor_Sel == MOTOR_RIGHT)
		{
			u = (((-(Theta[3] / Theta[2]) * u_[0] + ((real_T)(0.999999956715774) / Theta[2]) * uc) + ((real_T)(3.91619188305815e-08) /
				Theta[2]) * uc_[0]) - (((real_T)(-4.12230724487735e-09) - Theta[0]) / Theta[2]) * y) - (((real_T)(4.24835425529272e-18) -
			Theta[1]) / Theta[2]) * y_[0];
		}
		else
		{
			u = (((-(Theta[3] / Theta[2]) * u_[1] + ((real_T)(0.999999956715774) / Theta[2]) * uc) + ((real_T)(3.91619188305815e-08) /
				Theta[2]) * uc_[1]) - (((real_T)(-4.12230724487735e-09) - Theta[0]) / Theta[2]) * y) - (((real_T)(4.24835425529272e-18) -
			Theta[1]) / Theta[2]) * y_[1];
		}

		if (u > (real_T)90.0) {
			u = (real_T)90.0;
		} else {
			if (u < (real_T)(-90.0)) {
				u = (real_T)(-90.0);
			}
		}
	}
	else
	{
		u = 0;
	}
  if (Motor_Sel == MOTOR_RIGHT)
  {
	  uc_[0] = uc;
	  u_[0] = u;
	  y_[0] = y;
  }
  else
  {
	  uc_[1] = uc;
	  u_[1] = u;
	  y_[1] = y;
  }

  return (u);
}
/* End of code generation (STR_Indirect.c) */
