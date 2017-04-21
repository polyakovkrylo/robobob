#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "FreeRTOS.h"
#include "task.h"

#define LEFT_DIRECTION						1
#define RIGHT_DIRECTION						-1

void turn(int angularSpeed) {
	motor_set(DM_MOTOR0, angularSpeed);
	if(sign(motor_get_speed(DM_MOTOR2)) == sign(motor_get_speed(DM_MOTOR1))) {
		motor_set(DM_MOTOR1, angularSpeed);
		motor_set(DM_MOTOR2, angularSpeed);
	}
	vTaskDelay(5);
}

void start(int speed, int direction = 0) {
	switch(direction) {
		case LEFT_DIRECTION:
		motor_set(DM_MOTOR1, speed);
		motor_set(DM_MOTOR0, -speed);
		break;

		case RIGHT_DIRECTION:
		motor_set(DM_MOTOR0, speed);
		motor_set(DM_MOTOR2, -speed);
		break;

		default:
		motor_set(DM_MOTOR1, speed);
		motor_set(DM_MOTOR2, -speed);
		break;
	}
	vTaskDelay(5);
}

void stop() {
	motor_set(DM_MOTOR0, 0);
	motor_set(DM_MOTOR1, 0);
	motor_set(DM_MOTOR2, 0);
	vTaskDelay(5);
}

#endif
