#ifndef MOVEMENT_H_
#define MOVEMENT_H_


#define LEFT_DIRECTION						1
#define RIGHT_DIRECTION						-1

bool isStarted = false;

void turn(int angularSpeed) {
	motor_set(DM_MOTOR0, angularSpeed);
	if(sign(motor_get_speed(DM_MOTOR2)) == sign(motor_get_speed(DM_MOTOR1))) {
		motor_set(DM_MOTOR1, angularSpeed);
		motor_set(DM_MOTOR2, angularSpeed);
	}
}

void start(int speed, int direction) {
	switch(direction) {
		case RIGHT_DIRECTION:
		motor_set(DM_MOTOR1, speed);
		motor_set(DM_MOTOR0, -speed);
		// for turning while moving
		motor_set(DM_MOTOR2, speed/5);
		break;

		case LEFT_DIRECTION:
		motor_set(DM_MOTOR0, speed);
		motor_set(DM_MOTOR2, -speed);
		// for turning while moving
		motor_set(DM_MOTOR1, -speed/5);
		break;

		default:
		motor_set(DM_MOTOR1, speed);
		motor_set(DM_MOTOR2, -speed);
		break;
	}

	isStarted = (speed > 0) ? true : false;
}

void stop() {
	start(0,0);
}

#endif
