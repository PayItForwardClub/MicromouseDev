#include "include.h"

extern volatile float BatteryVoltage;
static uint8_t IR_Calib_Step = 0;
static uint32_t IR_vals[4];

void ButtonHandler(void)
{
	switch (system_GetState())
	{
		case SYSTEM_INITIALIZE:
			speed_Enable_Hbridge(false);
			system_SetState(SYSTEM_CALIB_SENSOR);
			IR_Calib_Step = 0;
			//LED1_ON();
			//LED2_ON();
			//LED3_ON();
			break;
		case SYSTEM_CALIB_SENSOR:

			speed_Enable_Hbridge(false);
			system_SetState(SYSTEM_SAVE_CALIB_SENSOR);
		case SYSTEM_SAVE_CALIB_SENSOR:
			system_SetState(SYSTEM_ESTIMATE_MOTOR_MODEL);
			speed_Enable_Hbridge(true);
			speed_set(MOTOR_LEFT,500);
			speed_set(MOTOR_RIGHT, 500);
			break;
		case SYSTEM_ESTIMATE_MOTOR_MODEL:
//			system_SetState(SYSTEM_SAVE_MOTOR_MODEL);
			system_SetState(SYSTEM_WAIT_TO_RUN);
			speed_Enable_Hbridge(false);
			break;
		case SYSTEM_WAIT_TO_RUN:
			speed_Enable_Hbridge(true);
			system_SetState(SYSTEM_RUN_SOLVE_MAZE);
			break;
		case SYSTEM_RUN_SOLVE_MAZE:
		case SYSTEM_RUN_IMAGE_PROCESSING:
			system_SetState(SYSTEM_WAIT_TO_RUN);
			speed_Enable_Hbridge(false);
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
				IR_set_calib_value(IR_CALIB_BASE_RIGHT);
				break;
			case 1:
				IR_set_calib_value(IR_CALIB_BASE_FRONT_LEFT);
				IR_set_calib_value(IR_CALIB_BASE_FRONT_RIGHT);
				LED1_OFF();
				LED2_ON();
				LED3_OFF();
				break;
			case 2:
				IR_set_calib_value(IR_CALIB_MAX_LEFT);
				LED1_ON();
				LED2_ON();
				LED3_OFF();
				break;
			case 3:
				IR_set_calib_value(IR_CALIB_MAX_RIGHT);
				LED1_OFF();
				LED2_OFF();
				LED3_ON();
				break;
			case 4:
				IR_set_calib_value(IR_CALIB_MAX_FRONT_LEFT);
				IR_set_calib_value(IR_CALIB_MAX_FRONT_RIGHT);
				LED1_ON();
				LED2_OFF();
				LED3_ON();
				break;
		}
		IR_Calib_Step++;
		IR_Calib_Step %= 4;
	}
}

void main(void)
{
	PID_PARAMETERS pid_param = {.Kp = 1.0, .Kd = 0.0, .Ki = 0.0,
			.Ts = 20
	};

	system_SetState(SYSTEM_INITIALIZE);
	Config_System();
	speed_control_init();
	pid_Wallfollow_init(pid_param);
	HostCommInit();
	qei_init(20);
	buzzer_init();
	BattSense_init();
	LED_Display_init();
	Button_init();
	IRDetector_init();
	pid_init();

	Timer_Init();

	ButtonRegisterCallback(BUTTON_LEFT, &ButtonHandler);
	ButtonRegisterCallback(BUTTON_RIGHT, &ButtonRightHandler);

//	buzzer_on(2000, 500);
//	buzzer_low_batterry_notify();
//	system_SetState(SYSTEM_ESTIMATE_MOTOR_MODEL);
//	speed_set(MOTOR_LEFT,1200);
//	speed_set(MOTOR_RIGHT, 100);
//	speed_Enable_Hbridge(true);
//	bluetooth_print("AT\r\n");
//	bluetooth_print("AT+NAME=NHH\r\n");
//	bluetooth_print("AT+PSWD=3393\r\n");
//	bluetooth_print("AT+UART=115200,0,0\r\n");
	pid_Wallfollow_set_follow(WALL_FOLLOW_RIGHT);
	while (1)
	{
		system_Process_System_State();
		HostComm_process();

//		IR_vals[0] = IR_GetIrDetectorValue(0);
//		IR_vals[1] = IR_GetIrDetectorValue(1);
//		IR_vals[2] = IR_GetIrDetectorValue(2);
//		IR_vals[3] = IR_GetIrDetectorValue(3);
	}

}
