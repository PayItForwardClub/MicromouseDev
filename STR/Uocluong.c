/*
 * Uocluong.c
 *
 * Code generation for function 'Uocluong'
 *
 * C source code generated on: Tue Apr 29 18:48:34 2014
 *
 */

/* Include files */
#include "Uocluong.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void Uocluong(real_T u_, real_T y, real_T Theta[4], real_T Theta_[4])
{
  volatile real_T PHI[4];
  real_T b_y;
  int32_T k;
  real_T c_y[4];
  int32_T i0;
  real_T e;
  real_T b_P_[4];
  real_T c_P_[16];
  real_T d_P_[16];
  real_T d0;
  int32_T i1;
  real_T P[16];
  static real_T u[2] = {0, 0};
  static real_T y_[2] = {0, 0};
  static real_T P_[16] = {
			10000, 0, 0, 0,
			0, 10000, 0, 0,
			0, 0, 10000, 0,
			0, 0, 0, 10000
  };

  PHI[0] = -y_[0];
  PHI[1] = -y_[1];
  PHI[2] = u[0];
  PHI[3] = u[1];
  y_[1] = y_[0];
  y_[0] = y;
  b_y = 0.0;
  for (k = 0; k < 4; k++) {
    b_y += PHI[k] * Theta_[k];
    c_y[k] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      c_y[k] += PHI[i0] * P_[i0 + (k << 2)];
    }
  }

  e = y - b_y;
  b_y = 0.0;
  for (k = 0; k < 4; k++) {
    b_y += c_y[k] * PHI[k];
    b_P_[k] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      b_P_[k] += P_[k + (i0 << 2)] * PHI[i0];
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (k = 0; k < 4; k++) {
      c_P_[k + (i0 << 2)] = b_P_[k] * PHI[i0];
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (k = 0; k < 4; k++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        d0 += c_P_[i0 + (i1 << 2)] * P_[i1 + (k << 2)];
      }

      d_P_[i0 + (k << 2)] = P_[i0 + (k << 2)] - d0 / (0.985 + b_y);
    }
  }

  b_y = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (k = 0; k < 4; k++) {
      P[k + (i0 << 2)] = 1.0050251256281406 * d_P_[k + (i0 << 2)];
    }

    c_y[i0] = 0.0;
    for (k = 0; k < 4; k++) {
      c_y[i0] += PHI[k] * P_[k + (i0 << 2)];
    }

    b_y += c_y[i0] * PHI[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    d0 = 0.0;
    for (k = 0; k < 4; k++) {
      d0 += P_[i0 + (k << 2)] * PHI[k];
    }

    b_P_[i0] = d0 / (0.985 + b_y);
  }

  for (i0 = 0; i0 < 4; i0++) {
    Theta[i0] = Theta_[i0] + b_P_[i0] * e;
  }
  for (i0 = 0; i0 < 4; i0++) {
    Theta_[i0] = Theta[i0];
  }
  for (i0 = 0; i0 < 16; i0++) {
    P_[i0] = P[i0];
  }
  u[1] = u[0];
  u[0] = u_;
}

void Uocluong2(real_T u_, real_T y, real_T Theta[4], real_T Theta_[4])
{
  volatile real_T PHI[4];
  real_T b_y;
  int32_T k;
  real_T c_y[4];
  int32_T i0;
  real_T e;
  real_T b_P_[4];
  real_T c_P_[16];
  real_T d_P_[16];
  real_T d0;
  int32_T i1;
  real_T P[16];
  static real_T u[2] = {0, 0};
  static real_T y_[2] = {0, 0};
  static real_T P_[16] = {
			10000, 0, 0, 0,
			0, 10000, 0, 0,
			0, 0, 10000, 0,
			0, 0, 0, 10000
  };

  PHI[0] = -y_[0];
  PHI[1] = -y_[1];
  PHI[2] = u[0];
  PHI[3] = u[1];
  y_[1] = y_[0];
  y_[0] = y;
  b_y = 0.0;
  for (k = 0; k < 4; k++) {
    b_y += PHI[k] * Theta_[k];
    c_y[k] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      c_y[k] += PHI[i0] * P_[i0 + (k << 2)];
    }
  }

  e = y - b_y;
  b_y = 0.0;
  for (k = 0; k < 4; k++) {
    b_y += c_y[k] * PHI[k];
    b_P_[k] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      b_P_[k] += P_[k + (i0 << 2)] * PHI[i0];
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (k = 0; k < 4; k++) {
      c_P_[k + (i0 << 2)] = b_P_[k] * PHI[i0];
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (k = 0; k < 4; k++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 4; i1++) {
        d0 += c_P_[i0 + (i1 << 2)] * P_[i1 + (k << 2)];
      }

      d_P_[i0 + (k << 2)] = P_[i0 + (k << 2)] - d0 / (0.985 + b_y);
    }
  }

  b_y = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (k = 0; k < 4; k++) {
      P[k + (i0 << 2)] = 1.0050251256281406 * d_P_[k + (i0 << 2)];
    }

    c_y[i0] = 0.0;
    for (k = 0; k < 4; k++) {
      c_y[i0] += PHI[k] * P_[k + (i0 << 2)];
    }

    b_y += c_y[i0] * PHI[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    d0 = 0.0;
    for (k = 0; k < 4; k++) {
      d0 += P_[i0 + (k << 2)] * PHI[k];
    }

    b_P_[i0] = d0 / (0.985 + b_y);
  }

  for (i0 = 0; i0 < 4; i0++) {
    Theta[i0] = Theta_[i0] + b_P_[i0] * e;
  }
  for (i0 = 0; i0 < 4; i0++) {
    Theta_[i0] = Theta[i0];
  }
  for (i0 = 0; i0 < 16; i0++) {
    P_[i0] = P[i0];
  }
  u[1] = u[0];
  u[0] = u_;
}
/* End of code generation (Uocluong.c) */
