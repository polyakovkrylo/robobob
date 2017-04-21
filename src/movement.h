#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#define LEFT_DIRECTION						1
#define RIGHT_DIRECTION						-1

void turn(int angularSpeed) {
	motor_set(DM_MOTOR0, angularSpeed);
	if(sign(motor_get_speed(DM_MOTOR2)) == sign(motor_get_speed(DM_MOTOR1))) {
		motor_set(DM_MOTOR1, angularSpeed);
		motor_set(DM_MOTOR2, angularSpeed);
	}
}

void start(int speed) {
	if(sign(motor_get_speed(DM_MOTOR1)) != sign(speed)){
		motor_set(DM_MOTOR1, 0);
		vTaskDelay(10);
	}
	if(sign(motor_get_speed(DM_MOTOR2)) != -sign(speed)){
			motor_set(DM_MOTOR2, 0);
			vTaskDelay(10);
	}
	motor_set(DM_MOTOR1, speed);
	motor_set(DM_MOTOR2, -speed);
}

void stop() {
	motor_set(DM_MOTOR0, 0);
	motor_set(DM_MOTOR1, 0);
	motor_set(DM_MOTOR2, 0);
}

#endif
