#include "Estimation_position.h"
#include "STR_Position.h"
#include "include.h"

#define lamda_predict		((double)100)
//#define lamda						((double)0.98)// The higher lamda the more stable, the lower the faster from 0.95 to 1

double adb_Error1[5] = {0, 0, 0, 0, 0};//e, e_, e__, ...
int32_t ai32_SetPoint1[4] = {0, 0, 0, 0};//SP, SP_, SP__, SP___
int32_t ai32_Position1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double adb_output1[8] = {0, 0, 0, 0, 0, 0, 0, 0};

extern double theta1[6][6], P1[6][6];

double adb_Error2[5] = {0, 0, 0, 0, 0};//e, e_, e__, ...
int32_t ai32_SetPoint2[4] = {0, 0, 0, 0};//SP, SP_, SP__, SP___
int32_t ai32_Position2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
double adb_output2[8] = {0, 0, 0, 0, 0, 0, 0, 0};

extern double theta2[6][6], P2[6][6];

unsigned int i,j,k;

void Multiply_Matrix(double x[6][6], unsigned char m, unsigned char n,  double y[6][6], unsigned char p, unsigned char q,  double z[6][6])
{
	static double Temp_mul = 0;
    for ( i = 0 ; i < m ; i++ )
    {
      for ( j = 0 ; j < q ; j++ )
      {
        for ( k = 0 ; k < p ; k++ )
        {
          Temp_mul = Temp_mul + x[i][k]*y[k][j];
        }
        z[i][j] = Temp_mul;
        Temp_mul = 0;
      }
    }
}
void Add_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]+y[i][j];
}
void Sub_Matrix(double x[6][6], double y[6][6],unsigned char m, unsigned char n,  double z[6][6])
{
    for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]-y[i][j];
}
void Multiply_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
	for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]*y;
}
void Divide_Matrix_Vector(double x[6][6],unsigned char m, unsigned char n,double y, double z[6][6])
{
	for(i=0;i<m;i++)
       for(j=0;j<n;j++)
            z[i][j]=x[i][j]/y;
}
void Tranpose_Matrix(double x[6][6],unsigned char m, unsigned char n,  double y[6][6])
{
   for(i=0;i<n;i++)
      for(j=0;j<m;j++)
           y[i][j]=0;
  for(i=0;i<n;i++)
  {
      for(j=0;j<m;j++)
      {
           y[i][j]=x[j][i];
      }
  }
}

void Estimation_Algorithm(bool Select)
{
	static double phiT[6][6], theta_temp, phiTPphi, epsilon, lamda;
	static double temp2[6][6], temp[6][6], denominator[6][6], numerator[6][6], L[6][6];
	static double phi[6][6];
	
	if (!Select)
	{
		phi[0][0] = -ai32_Position1[3];
		phi[1][0] = -ai32_Position1[4];
		phi[2][0] = -ai32_Position1[5];
		phi[3][0] = adb_output1[3];
		phi[4][0] = adb_output1[4];
		phi[5][0] = adb_output1[5];
	}
	else
	{
		phi[0][0] = -ai32_Position2[3];
		phi[1][0] = -ai32_Position2[4];
		phi[2][0] = -ai32_Position2[5];
		phi[3][0] = adb_output2[3];
		phi[4][0] = adb_output2[4];
		phi[5][0] = adb_output2[5];
	}
	Tranpose_Matrix(phi,6,1,phiT);
	/*epsilon(k) = Frequency(k) - phiT(k) * theta(k-1)  Prediction error*/
	if (!Select)
	{
		Multiply_Matrix(phiT,1,6,theta1,6,1,temp);//temp = phiT(k)*theta(k-1)
		epsilon = ai32_Position1[0] - temp[0][0];
	}
	else
	{
		Multiply_Matrix(phiT,1,6,theta2,6,1,temp);//temp = phiT(k)*theta(k-1)
		epsilon = ai32_Position2[0] - temp[0][0];
	}
	
	/*L(k) = (P(k-1)*phi(k))/(lamda + phiT(k)*P(k-1)*phi(k))*/
	if (!Select)
	{
		Multiply_Matrix(P1,6,6,phi,6,1,numerator);//(P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P1,6,6,temp);//temp = phiT(k)*P(k-1)
	}
	else
	{
		Multiply_Matrix(P2,6,6,phi,6,1,numerator);//(P(k-1)*phi(k))
		Multiply_Matrix(phiT,1,6,P2,6,6,temp);//temp = phiT(k)*P(k-1)		
	}
	Multiply_Matrix(temp,1,6,phi,6,1,denominator);//phiT(k)*P(k-1)*phi(k)
	phiTPphi = denominator[0][0];
	denominator[0][0] = denominator[0][0] + 1;//(1 + phiT(k)*P(k-1)*phi(k))
	Divide_Matrix_Vector(numerator,6,1,denominator[0][0],L);
	/*lamda*/
	lamda = epsilon * epsilon;
	lamda = lamda / (1 + phiTPphi);
	lamda = lamda / lamda_predict;
	lamda = 1 - lamda;
	if(lamda>1){lamda = 1;}
	if(lamda<0.95){lamda = 0.95;}
	/*P(k) = (P(k-1) - (P(k-1)*phi(k)*phiT(k)*P(k-1))/(lamda + phiT(k)*P(k-1)*phi(k)))/lamda*/
	Multiply_Matrix(L,6,1,temp,1,6,temp2);
	if (!Select)
	{
		Sub_Matrix(P1,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P1);
		theta_temp = theta1[3][0];
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta1,temp,6,1,theta1);
	}
	else
	{
		Sub_Matrix(P2,temp2,6,6,temp);
		Divide_Matrix_Vector(temp,6,6,lamda,P2);
		theta_temp = theta2[3][0];
		Multiply_Matrix_Vector(L,6,1,epsilon,temp);
		Add_Matrix(theta2,temp,6,1,theta2);
	}
	/*theta(k) = theta(k-1) + L(k)*epsilon(k)*/
	if (!Select)
	{
		/*Recursion*/
		for (i = 7; i > 0; i--)
		{
			ai32_Position1[i] = ai32_Position1[i - 1];
			adb_output1[i] = adb_output1[i-1];
		}
		
		ai32_SetPoint1[3] = ai32_SetPoint1[2];
		ai32_SetPoint1[2] = ai32_SetPoint1[1];
		ai32_SetPoint1[1] = ai32_SetPoint1[0];
		adb_Error1[2] = adb_Error1[1];
		adb_Error1[1] = adb_Error1[0];

		if (theta1[3][0] == 0) 
		{
			theta1[3][0] = theta_temp;
		}
	}
	else
	{
		/*Recursion*/
		for (i = 7; i > 0; i--)
		{
			ai32_Position2[i] = ai32_Position2[i - 1];
			adb_output2[i] = adb_output2[i-1];
		}
		
		ai32_SetPoint2[3] = ai32_SetPoint2[2];
		ai32_SetPoint2[2] = ai32_SetPoint2[1];
		ai32_SetPoint2[1] = ai32_SetPoint2[0];
		adb_Error2[2] = adb_Error2[1];
		adb_Error2[1] = adb_Error2[0];
		if (theta2[3][0] == 0) 
		{
			theta2[3][0] = theta_temp;
		}
	}
}
