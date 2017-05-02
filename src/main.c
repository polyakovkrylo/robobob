//Code by Hector Munoz and Vladimir Poliakov.

#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

#include "dorobo32.h"
#include "trace.h"
#include "adc.h"
#include "digital.h"
#include "fft.h"
#include "motor.h"

#include "rmath.h"
#include "movement.h"
#include "targeting.h"
#include "obstacle.h"

enum States{
	NoTarget,
	Adjusting,
	OnCourse,
	ObstacleDetected,
	CollisionDetected,
	Avoiding,
	ChangingPosition
};

// System state
int currentState = NoTarget;

void setState(int state);
static void irTask(void *pvParameters);
static void mainTask(void *pvParameters);
static void distanceTask(void *pvParameters);

int main() {
	// Initialization
	dorobo_init();
	trace_init();

	initCollisionSensors();

	// Create tasks
	xTaskCreate(mainTask, "mainTask", 256, NULL, 2, NULL);
	xTaskCreate(irTask, "irTask", 256, NULL, 2, NULL);
	xTaskCreate(distanceTask, "distanceTask", 256, NULL, 2, NULL);

	vTaskStartScheduler();	//start the freertos scheduler
	return 0;				//should not be reached!
}

void setState(int state) {
	tracef("new state: %d", state);
	currentState = state;

	// Do entering routine
	switch(state) {
	case NoTarget:
		resetBlindCounter();
		stop();
		vTaskDelay(20);
		turn(45 * -lastSeen);
		break;

	case Adjusting:
      		turn(0);
    		break;

   	case OnCourse:
      		turn(0);
      		start(85,0);
    		break;

	case ObstacleDetected:
		stop();
		vTaskDelay(20);
		currentObstacleDistanceMeter = triggeredDistanceMeter;
		start(85  , currentObstacleDistanceMeter);
		setState(Avoiding);
		break;

	case CollisionDetected:
		stop();
		start(-60,collisionSide);	// Move back from collision
		break;

	case ChangingPosition:
		resetBlindCounter();
		stop();
		vTaskDelay(10);
		start(70, 0);
		turn(40*-lastSeen);
		break;
  }
}

static void mainTask(void *pvParameters){
	motor_init();
	setState(NoTarget);

	while(1){
		switch(currentState){
		case NoTarget:
			if(max(irLeft, irRight) > IR_TARGET_VALUE){
				setState(Adjusting);
				continue;
			}
			if(isTargetLost() && !triggeredDistanceMeter){
				setState(ChangingPosition);
			}
			break;

		case Adjusting:
			if(isTargetLost()) {
				setState(NoTarget);
				continue;
			}

			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if((abs(irLeft - irRight) < IR_MIN_DIFF)){
				setState(OnCourse);
				continue;
			}

			int angSp = 15;
			if(irLeft < irRight){
				angSp = -angSp;
			}
			if(isStarted) {
				angSp *= 5;
			}
			turn(angSp);

			break;

		case OnCourse:
			if(isTargetLost()) {
				setState(NoTarget);
				continue;
			}

			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if(abs(irLeft - irRight) > IR_MIN_DIFF){
				setState(Adjusting);
			}
			break;

		case CollisionDetected:
			vTaskDelay(150);
			stop();
			vTaskDelay(20);
			turn(collisionSide*60);
			vTaskDelay(80);
			collisionSide=0;
			setState(OnCourse);
			break;

		case Avoiding:
			updateBlindCounter();
			if(currentObstacleDistanceMeter != triggeredDistanceMeter) {
				setState(ObstacleDetected);
			}

			if(max(distanceLeft, distanceRight) < DIST_SENSOR_OBSTACLE_VALUE) {
				stop();
				vTaskDelay(20);
				setState(OnCourse);
			}
			break;

		case ChangingPosition:

			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if(max(irLeft, irRight) > IR_TARGET_VALUE){
				stop();
				vTaskDelay(10);
				setState(Adjusting);
				continue;
			}

			if(isTargetLost())
				setState(NoTarget);

			break;
		}

		vTaskDelay(20);
	}
}

/*
 * Collision sensor interruption
 */
void EXTI4_15_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_14)) {
		collisionSide=RIGHT_DIRECTION;
		setState(CollisionDetected);
	} else if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)) {
		collisionSide=LEFT_DIRECTION;
		setState(CollisionDetected);
	}

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

static void distanceTask(void *pvParameters) {
	// ADC initialization
	adc_init();
	while(1){
		readDistanceSensors();
		updateDistanceAverage();
		checkForTriggeredSide();
		vTaskDelay(10);
	}
}

static void irTask(void *pvParameters) {
	// FFT initialization
	ft_init();
	while(1){
		readIrSensors();
		updateIrAverage();
		setLastSeen();
		updateBlindCounter();
		vTaskDelay(10);
	}
}
