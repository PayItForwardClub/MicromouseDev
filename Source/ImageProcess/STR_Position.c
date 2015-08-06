#include "Estimation_position.h"
#include "STR_Position.h"
#include "include.h"

double theta1[6][6], P1[6][6];
double theta2[6][6], P2[6][6];
double s0,s1,s2,t0,t1,t2,r1,r2;

void Initial_Condition_Position(void)
{
	theta1[0][0]= -1;			//a1
	theta1[1][0]= 0;				//a2
	theta1[2][0]= 0;				//a3
	theta1[3][0]= 1;				//b1
	theta1[4][0]= 0;				//b2
	theta1[5][0]= 0;				//b3

	P1[0][0] = 1;
	P1[1][1] = 1;
	P1[2][2] = 1;
	P1[3][3] = 1;
	P1[4][4] = 1;
	P1[5][5] = 1;
	
	theta2[0][0]= -1;			//a1
	theta2[1][0]= 0;				//a2
	theta2[2][0]= 0;				//a3
	theta2[3][0]= 1;				//b1
	theta2[4][0]= 0;				//b2
	theta2[5][0]= 0;				//b3

	P2[0][0] = 1;
	P2[1][1] = 1;
	P2[2][2] = 1;
	P2[3][3] = 1;
	P2[4][4] = 1;
	P2[5][5] = 1;
}

void Model_Position(bool Select)
{
	if (!Select)
	{
		adb_Error1[0] = ai32_SetPoint1[0] - ai32_Position1[0];

		r1 = theta1[4][0] / theta1[3][0];
		r2 = theta1[5][0] / theta1[3][0];
		t0 = 0.999999952160871 / theta1[3][0];
		t1 = 4.37168138727063e-08 / theta1[3][0];
		t2 = 5.24488179668768e-20 / theta1[3][0];
		s0 = (-4.12230724487712e-09 - theta1[0][0]) / theta1[3][0];
		s1 = (4.24835425529119e-18 - theta1[1][0]) / theta1[3][0];
		s2 = (3.62206672972073e-41 - theta1[2][0]) / theta1[3][0];
			
		ai32_Position1[0] = qei_Get_Position(false);
	//	ai32_Position[0] = QEIPositionGet(QEI0_BASE);
		
		adb_output1[0] = - (r1 * adb_output1[1]) - (r2 * adb_output1[2])
					+ (t0 * ai32_SetPoint1[0]) + (t1 * ai32_SetPoint1[1]) + (t2 * ai32_SetPoint1[2])
					- (s0 * ai32_Position1[0]) - (s1 * ai32_Position1[1])- (s2 * ai32_Position1[2]);
		if (adb_output1[0] > 90)
			adb_output1[0] = 90;
		else if (adb_output1[0] < -90)
			adb_output1[0] = -90;
	}
	else
	{
		adb_Error2[0] = ai32_SetPoint2[0] - ai32_Position2[0];

		r1 = theta2[4][0] / theta2[3][0];
		r2 = theta2[5][0] / theta2[3][0];
		t0 = 0.999999952160871 / theta2[3][0];
		t1 = 4.37168138727063e-08 / theta2[3][0];
		t2 = 5.24488179668768e-20 / theta2[3][0];
		s0 = (-4.12230724487712e-09 - theta2[0][0]) / theta2[3][0];
		s1 = (4.24835425529119e-18 - theta2[1][0]) / theta2[3][0];
		s2 = (3.62206672972073e-41 - theta2[2][0]) / theta2[3][0];
			
		ai32_Position2[0] = qei_Get_Position(true);
	//	ai32_Position[0] = QEIPositionGet(QEI0_BASE);
		
		adb_output2[0] = - (r1 * adb_output2[1]) - (r2 * adb_output2[2])
					+ (t0 * ai32_SetPoint2[0]) + (t1 * ai32_SetPoint2[1]) + (t2 * ai32_SetPoint2[2])
					- (s0 * ai32_Position2[0]) - (s1 * ai32_Position2[1])- (s2 * ai32_Position2[2]);
		if (adb_output2[0] > 90)
			adb_output2[0] = 90;
		else if (adb_output2[0] < -90)
			adb_output2[0] = -90;
	}
}
