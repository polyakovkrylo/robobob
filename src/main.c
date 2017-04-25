//Code by Hector Munoz and Vladimir Poliakov.

//possible improvement: entering while(1) inside robobob thread, ask for every measurement at the begining
//and save it in a variable with a name for more easily handling.
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

#define COLLISSION_SENSOR_LEFT				DD_PIN_PD14
#define COLLISSION_SENSOR_RIGHT				DD_PIN_PD15
#define IR_LEFT								DD_PIN_PC8
#define IR_RIGHT							DD_PIN_PC9
#define DISTANCE_METER_RIGHT				DA_ADC_CHANNEL0
#define DISTANCE_METER_LEFT					DA_ADC_CHANNEL1

#define DIST_SENSOR_MAX						3600
#define DIST_SENSOR_MIN						250
#define DIST_SENSOR_OBSTACLE_VALUE			600
#define DIST_ARRAY_SIZE						5

enum States{
	NoTarget,
	Adjusting,
	OnCourse,
	ObstacleDetected,
	CollisionDetected,
	Avoiding
};

// System state
int currentState = NoTarget;

// Distance meters
const int lDistanceMeter = 1;
const int rDistanceMeter = 2;
int triggeredDistanceMeter = 0;
int currentObstacleDistanceMeter = 0;
int avDistanceValues[3]={0};
int lDistanceValues[DIST_ARRAY_SIZE] = {0};
int rDistanceValues[DIST_ARRAY_SIZE] = {0};
int it_currentValueIndex = 0;

// Collision Sensors
int collisionSide = 0;

void setState(int state);
static void irTask(void *pvParameters);
static void mainTask(void *pvParameters);
static void distanceTask(void *pvParameters);

int main() {
	// Initialization
	dorobo_init();
	trace_init();
	adc_init();
//	digital_init();
	motor_init();

	// Pin configuration
//	digital_configure_pin(COLLISSION_SENSOR_LEFT, DD_CFG_INPUT_PULLUP);
//	digital_configure_pin(COLLISSION_SENSOR_RIGHT, DD_CFG_INPUT_PULLUP);

	GPIO_InitTypeDef GPIO_InitStructLeft;
	GPIO_InitStructLeft.Pin = GPIO_PIN_14;
	GPIO_InitStructLeft.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructLeft.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructLeft);

	GPIO_InitTypeDef GPIO_InitStructRight;
	GPIO_InitStructRight.Pin = GPIO_PIN_15;
	GPIO_InitStructRight.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructRight.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructRight);

	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Set initial state
	setState(NoTarget);

	// Create tasks
	xTaskCreate(mainTask, "mainTask", 256, NULL, 2, NULL);
	xTaskCreate(irTask, "irTask", 256, NULL, 2, NULL);
	xTaskCreate(distanceTask, "distanceTask", 256, NULL, 2, NULL);

	vTaskStartScheduler();	//start the freertos scheduler
	return 0;								//should not be reached!
}

void setState(int state) {
	tracef("new state: %d", state);
	currentState = state;

	// Do entering routine
	switch(state) {
	case NoTarget:
		resetBlindCounter();
		stop();
		turn(35 * lastSeen);
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

		currentObstacleDistanceMeter = triggeredDistanceMeter;

		if(currentObstacleDistanceMeter == lDistanceMeter)
			start(85, LEFT_DIRECTION);
		else
			start(85  , RIGHT_DIRECTION);
		setState(Avoiding);
		break;

	case CollisionDetected:
		stop();
		start(-60,collisionSide);	// Move back to opposite side
		break;
  }
}

/*
 * Right Collision sensor interruption
 */
//void EXTI4_14_IRQHandler(void)
//{
//	if(digital_get_pin(COLLISSION_SENSOR_LEFT)==DD_LEVEL_HIGH){
//		collisionSide=RIGHT_DIRECTION;
//		setState(CollisionDetected);
//	}
//
//	else if(digital_get_pin(COLLISSION_SENSOR_RIGHT)==DD_LEVEL_HIGH){
//		collisionSide=LEFT_DIRECTION;
//		setState(CollisionDetected);
//	}
//}

/*
 * Right Collision sensor interruption
 */
void EXTI4_15_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_14)){
		collisionSide=RIGHT_DIRECTION;
		setState(CollisionDetected);
	}

	else if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)){
		collisionSide=LEFT_DIRECTION;
		setState(CollisionDetected);
	}
//	tracef("left %d, right %d", digital_get_pin(COLLISSION_SENSOR_LEFT), digital_get_pin(COLLISSION_SENSOR_RIGHT));
//	collisionSide=RIGHT_DIRECTION;
//	setState(CollisionDetected);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

static void distanceTask(void *pvParameters) {
	// Read sensors

	while(1){
//		// If switch is pushed, go to CollisionDetected state
//		if(currentState!=CollisionDetected){
//			if(digital_get_pin(COLLISSION_SENSOR_LEFT)==DD_LEVEL_LOW){
//				collisionSide=RIGHT_DIRECTION;
//				setState(CollisionDetected);
//			}
//
//			else if(digital_get_pin(COLLISSION_SENSOR_RIGHT)==DD_LEVEL_LOW){
//				collisionSide=LEFT_DIRECTION;
//				setState(CollisionDetected);
//			}
//		}
		// Read distance sensors
		rDistanceValues[it_currentValueIndex] = adc_get_value(DISTANCE_METER_RIGHT);
		lDistanceValues[it_currentValueIndex] = adc_get_value(DISTANCE_METER_LEFT);

		// Calculate average
		int rSum = 0;
		int lSum = 0;
		for (int i = 0; i < DIST_ARRAY_SIZE; i++) {
			lSum += lDistanceValues[i];
			rSum += rDistanceValues[i];
		}
		avDistanceValues[lDistanceMeter] = lSum / DIST_ARRAY_SIZE;
		avDistanceValues[rDistanceMeter] = rSum / DIST_ARRAY_SIZE;

		// Check if obstacle was detected
		if(max(avDistanceValues[rDistanceMeter],avDistanceValues[lDistanceMeter]) > DIST_SENSOR_OBSTACLE_VALUE) {
			if(avDistanceValues[rDistanceMeter] > avDistanceValues[lDistanceMeter]) {
				triggeredDistanceMeter = rDistanceMeter;
			} else {
				triggeredDistanceMeter = lDistanceMeter;
			}
		} else {
			triggeredDistanceMeter = 0;
		}

		// Update the iterator
		it_currentValueIndex = (++it_currentValueIndex) % DIST_ARRAY_SIZE; 

		vTaskDelay(10);
	}
}

static void irTask(void *pvParameters) {

	// FFT initialization
	ft_init();

	while(1){
		// Read right IR
		ft_start_sampling(IR_LEFT);
		while(!ft_is_sampling_finished()){
			vTaskDelay(10);
		}
		irLeft = ft_get_transform(DFT_FREQ125);

		// Read right IR
		ft_start_sampling(IR_RIGHT);
		while(!ft_is_sampling_finished()){
			vTaskDelay(10);
		}
		irRight = ft_get_transform(DFT_FREQ125);

		// Set last seen  direction
		if(irRight>irLeft){
			lastSeen = RIGHT_DIRECTION;
		}
		else if(irRight<irLeft){
			lastSeen = LEFT_DIRECTION;
		}

		updateBlindCounter();
		vTaskDelay(20);
	}
}

static void mainTask(void *pvParameters){

	while(1){
		switch(currentState){
		case NoTarget:
			if(max(irLeft, irRight) > IR_TARGET_VALUE)
				setState(Adjusting);
			break;

		case Adjusting:
			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if((abs(irLeft-irRight) < IR_MIN_DIFF)){
				setState(OnCourse);
				continue;
			}
				
			int angSp = 15;
			if(irLeft<irRight){
				angSp = -angSp;
			}
			if(isStarted) {
				angSp *= 5;
			}
			turn(angSp);

			if(isTargetLost())
				setState(NoTarget);
			break;

		case OnCourse:
			tracef("blind counter: %d", blindCounter);
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
			setState(OnCourse);
			break;

		case Avoiding:
			if(currentObstacleDistanceMeter != triggeredDistanceMeter) {
				setState(ObstacleDetected);
			}

			if(avDistanceValues[currentObstacleDistanceMeter] < DIST_SENSOR_OBSTACLE_VALUE) {
				stop();
				setState(OnCourse);
			}
			break;
		}

		vTaskDelay(20);
	}
}
