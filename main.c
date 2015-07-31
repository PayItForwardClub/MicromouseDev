#include "include.h"

#define ESTIMATE_MOTOR_MODEL

extern volatile float BatteryVoltage;
uint8_t IR_Calib_Step = 0;
static uint32_t IR_vals[4];

PID_PARAMETERS pid_wall = {.Kp = 0.1, .Kd = 0.0, .Ki = 0.0005,
		.Ts = 0.020, .PID_Saturation = 200,
//PID_PARAMETERS pid_wall = {.Kp = 0.3, .Kd = 0.0, .Ki = 0.0005,
//		.Ts = 0.020, .PID_Saturation = 200,
};
void ButtonLeftHandler(void)
{
	switch (system_GetState())
	{
		case SYSTEM_INITIALIZE:
			speed_Enable_Hbridge(false);
			system_SetState(SYSTEM_CALIB_SENSOR);
			IR_Calib_Step = 0;
			break;
		case SYSTEM_CALIB_SENSOR:
			speed_Enable_Hbridge(false);
			system_SetState(SYSTEM_SAVE_CALIB_SENSOR);
			break;
		case SYSTEM_SAVE_CALIB_SENSOR:

#ifdef ESTIMATE_MOTOR_MODEL

			system_SetState(SYSTEM_ESTIMATE_MOTOR_MODEL);
			speed_Enable_Hbridge(true);
			speed_set(MOTOR_LEFT, 200);
			speed_set(MOTOR_RIGHT, 400);
#else
			loadMotorModel();
			system_SetState(SYSTEM_WAIT_TO_RUN);
#endif
			break;
		case SYSTEM_ESTIMATE_MOTOR_MODEL:
#ifdef ESTIMATE_MOTOR_MODEL
			system_SetState(SYSTEM_SAVE_MOTOR_MODEL);
#else
			system_SetState(SYSTEM_WAIT_TO_RUN);
#endif
			speed_Enable_Hbridge(false);
			break;
		case SYSTEM_SAVE_MOTOR_MODEL:
#ifdef ESTIMATE_MOTOR_MODEL
			saveMotorModel();
#endif
		case SYSTEM_WAIT_TO_RUN:

			SysCtlDelay(SysCtlClockGet()/3);
			system_SetState(SYSTEM_RUN_SOLVE_MAZE);
		case SYSTEM_RUN_SOLVE_MAZE:
		case SYSTEM_RUN_IMAGE_PROCESSING:
			speed_Enable_Hbridge(true);
//			system_SetState(SYSTEM_WAIT_TO_RUN);
//			speed_Enable_Hbridge(false);
			break;
		default:
			break;
	}
}

void ButtonRightHandler(void)
{
	if (system_GetState() == SYSTEM_CALIB_SENSOR)
	{
		switch(IR_Calib_Step)
		{
			case 0:
				LED1_ON();
				LED2_OFF();
				LED3_OFF();
				IR_set_calib_value(IR_CALIB_BASE_LEFT);
				break;
			case 1:
				IR_set_calib_value(IR_CALIB_BASE_RIGHT);
				break;
			case 2:
				IR_set_calib_value(IR_CALIB_BASE_FRONT_LEFT);
				IR_set_calib_value(IR_CALIB_BASE_FRONT_RIGHT);
				LED1_OFF();
				LED2_ON();
				LED3_OFF();
				break;
			case 3:
				IR_set_calib_value(IR_CALIB_MAX_LEFT);
				LED1_OFF();
				LED2_OFF();
				LED3_ON();
				break;
			case 4:
				IR_set_calib_value(IR_CALIB_MAX_RIGHT);
				LED1_ON();
				LED2_ON();
				LED3_OFF();
				break;
			case 5:
				IR_set_calib_value(IR_CALIB_MIN_FRONT_LEFT);
				IR_set_calib_value(IR_CALIB_MIN_FRONT_RIGHT);
				LED1_OFF();
				LED2_OFF();
				LED3_OFF();
				break;
		}
		IR_Calib_Step++;
		IR_Calib_Step %= 6;
	}
}

void main(void){
	system_SetState(SYSTEM_INITIALIZE);
	Config_System();
	EEPROMConfig();
	Timer_Init();
	speed_control_init();
	pid_init();
	pid_Wallfollow_init(pid_wall);
	HostCommInit();
	qei_init(20);
	buzzer_init();
//	BattSense_init();
	LED_Display_init();
	Button_init();
	IRDetector_init();

	ButtonRegisterCallback(BUTTON_LEFT, &ButtonLeftHandler);
	ButtonRegisterCallback(BUTTON_RIGHT, &ButtonRightHandler);

//	buzzer_on(2000, 500);
//	buzzer_low_batterry_notify();
//	system_SetState(SYSTEM_ESTIMATE_MOTOR_MODEL);
//	speed_set(MOTOR_LEFT,1200);
//	speed_set(MOTOR_RIGHT, 100);
//	speed_Enable_Hbridge(true);
//	bluetooth_print("AT\r\n");

//	bluetooth_print("AT+NAME=NHH\r\n");

//	bluetooth_print("AT+CMODE=0\r\n");
//	bluetooth_print("AT+PSWD=1234\r\n");
//	bluetooth_print("AT+UART=115200,0,0\r\n");
	pid_Wallfollow_set_follow(WALL_FOLLOW_RIGHT);
	qei_setPosLeft(0);
	qei_setPosRight(0);
	while (1)
	{
	//	bluetooth_print("Left:%d, right:%d\r\n",qei_getPosLeft(),qei_getPosRight());
	//	bluetooth_print("Right:%d",qei_getPosRight());
	//	qei_setPosLeft(0);
	//	qei_setPosRight(0);
	//	SysCtlDelay(SysCtlClockGet()/15);



		system_Process_System_State();

//		HostComm_process();

//		IR_vals[0] = IR_GetIrDetectorValue(0);
//		IR_vals[1] = IR_GetIrDetectorValue(1);
//		IR_vals[2] = IR_GetIrDetectorValue(2);
//		IR_vals[3] = IR_GetIrDetectorValue(3);
	}

}
