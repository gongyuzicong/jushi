#include "pid.h"

PID_LOC_SPEED pidSpeed_loc;
PID_INC_SPEED pidSpeed_inc;


void PID_LOC_SPEED_INIT(void)
{
	pidSpeed_loc.SetSpeed = 0.0;
	pidSpeed_loc.ActualSpeed = 0.0;
	pidSpeed_loc.err = 0.0;
	pidSpeed_loc.err_last = 0.0;
	pidSpeed_loc.voltage = 0.0;
	pidSpeed_loc.integral = 0.0;
	pidSpeed_loc.Kp = 0.2;
	pidSpeed_loc.Ki = 0.015;
	pidSpeed_loc.Kd = 0.2;
}

float PID_LOC_realize(float speed)
{
	pidSpeed_loc.SetSpeed = speed;
	pidSpeed_loc.err = pidSpeed_loc.SetSpeed - pidSpeed_loc.ActualSpeed;
	pidSpeed_loc.integral += pidSpeed_loc.err;
	pidSpeed_loc.voltage = pidSpeed_loc.Kp * pidSpeed_loc.err + pidSpeed_loc.Ki * pidSpeed_loc.integral + pidSpeed_loc.Kd * (pidSpeed_loc.err - pidSpeed_loc.err_last);
	pidSpeed_loc.err_last = pidSpeed_loc.err;
	pidSpeed_loc.ActualSpeed = pidSpeed_loc.voltage * 1.0;

	return pidSpeed_loc.ActualSpeed;
	
}

void PID_LOC_TEST(void)
{
	u16 count = 0;
	
	printf("PID_LOC_START\r\n");
	PID_LOC_SPEED_INIT();
	
	while(count < 1000)
	{
		float speed = PID_LOC_realize(200.0);
		printf("%f\r\n", speed);
		count++;
	}
	
}

void PID_INC_SPEED_INIT(void)
{
	pidSpeed_inc.SetSpeed = 0.0;
	pidSpeed_inc.ActualSpeed = 0.0;
	pidSpeed_inc.err = 0.0;
	pidSpeed_inc.err_next = 0.0;
	pidSpeed_inc.err_last = 0.0;
	pidSpeed_inc.Kp = 0.2;
	pidSpeed_inc.Ki = 0.015;
	pidSpeed_inc.Kd = 0.2;
}

float PID_INC_realize(float speed)
{
	float incrementSpeed = 0.0;
	
	pidSpeed_inc.SetSpeed = speed;
	pidSpeed_inc.err = pidSpeed_inc.SetSpeed - pidSpeed_inc.ActualSpeed;
	incrementSpeed = pidSpeed_inc.Kp * (pidSpeed_inc.err - pidSpeed_inc.err_next) + pidSpeed_inc.Ki * pidSpeed_inc.err + pidSpeed_inc.Kd * (pidSpeed_inc.err - 2 * pidSpeed_inc.err_next + pidSpeed_inc.err_last);
	pidSpeed_inc.ActualSpeed += incrementSpeed;
	pidSpeed_inc.err_last = pidSpeed_inc.err_next;
	pidSpeed_inc.err_next = pidSpeed_inc.err;

	return pidSpeed_inc.ActualSpeed;
	
}


void PID_INC_TEST(void)
{
	u16 count = 0;
	float speed = 0.0;
	
	PID_INC_SPEED_INIT();

	while(count < 1000)
	{
		speed = PID_INC_realize(200.0);
		printf("%f\r\n", speed);
		count++;
	}
	
}


