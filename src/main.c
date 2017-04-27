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

#define COLLISSION_SENSOR_LEFT				DD_PIN_PD14
#define COLLISSION_SENSOR_RIGHT				DD_PIN_PD15
#define IR_LEFT								DD_PIN_PC8
#define IR_RIGHT							DD_PIN_PC9
#define DISTANCE_METER_RIGHT				DA_ADC_CHANNEL0
#define DISTANCE_METER_LEFT					DA_ADC_CHANNEL1

#define DIST_SENSOR_MAX						3600
#define DIST_SENSOR_MIN						250
#define DIST_SENSOR_OBSTACLE_VALUE			800
#define DIST_ARRAY_SIZE						5
#define IR_ARRAY_SIZE						2

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

// Distance meters
const int lDistanceMeter = 1;
const int rDistanceMeter = 2;
const int rIr=1;
const int lIr=2;
int triggeredDistanceMeter = 0;
int currentObstacleDistanceMeter = 0;
int avDistanceValues[3]={0};
int avIrValues[3]={0};
int lDistanceValues[DIST_ARRAY_SIZE] = {0};
int rDistanceValues[DIST_ARRAY_SIZE] = {0};
int irRightValues[IR_ARRAY_SIZE]={0};
int irLeftValues[IR_ARRAY_SIZE]={0};
int it_distanceValues = 0;
int it_irValues=0;

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
	motor_init();

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
		turn(45 * lastSeen);
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

		if(currentObstacleDistanceMeter == lDistanceMeter)
			start(85, LEFT_DIRECTION);
		else
			start(85  , RIGHT_DIRECTION);
		setState(Avoiding);
		break;

	case CollisionDetected:
		stop();
		start(-60,collisionSide);	// Move back from collision
		break;
			
	case ChangingPosition:
		stop();
		vTaskDelay(10);
		start(70, 0);
		turn(30*lastSeen);
		resetBlindCounter();
		break;
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
	// Read sensors

	while(1){
		// Read distance sensors
		rDistanceValues[it_distanceValues] = adc_get_value(DISTANCE_METER_RIGHT);
		lDistanceValues[it_distanceValues] = adc_get_value(DISTANCE_METER_LEFT);

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
		it_distanceValues = (++it_distanceValues) % DIST_ARRAY_SIZE; 

		vTaskDelay(10);
	}
}

static void irTask(void *pvParameters) {

	// FFT initialization
	ft_init();
	while(1){

		// Read right IR
		ft_start_sampling(IR_RIGHT);
		while(!ft_is_sampling_finished()){
			vTaskDelay(10);
		}
		irRightValues[it_irValues] = ft_get_transform(DFT_FREQ125);
		tracef("level right %d ", ft_get_transform(DFT_FREQ125));
		
		// Read left IR
		ft_start_sampling(IR_LEFT);
		while(!ft_is_sampling_finished()){
			vTaskDelay(10);
		}
		irLeftValues[it_irValues] = ft_get_transform(DFT_FREQ125);
		tracef("level left %d", ft_get_transform(DFT_FREQ125));
		
		int rSum=0;
		int lSum=0;
		for(int i=0; i<IR_ARRAY_SIZE;i++){
			rSum+=irRightValues[i];
			lSum+=irLeftValues[i];
		}

		avIrValues[lIr] = lSum / DIST_ARRAY_SIZE;
		avIrValues[rIr] = rSum / DIST_ARRAY_SIZE;
		tracef("level %d , %d", avIrValues[rIr], avIrValues[lIr]);
		// Update iterator
		it_irValues = (++it_irValues) % IR_ARRAY_SIZE;
		
		// Set last seen  direction
		if(avIrValues[rIr]>avIrValues[lIr]){
			lastSeen = RIGHT_DIRECTION;
		}
		else if(avIrValues[rIr]<avIrValues[lIr]){
			lastSeen = LEFT_DIRECTION;
		}

		updateBlindCounter();
		vTaskDelay(10);
	}
}

static void mainTask(void *pvParameters){

	setState(NoTarget);
	while(1){
		switch(currentState){
		case NoTarget:
			if(max(avIrValues[lIr], avIrValues[rIr]) > IR_TARGET_VALUE){
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

			if((abs(avIrValues[lIr]-avIrValues[rIr]) < IR_MIN_DIFF)){
				setState(OnCourse);
				continue;
			}
				
			int angSp = 15;
			if(avIrValues[lIr]<avIrValues[rIr]){
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

//			if(isFalseTarget()) {
//				setState(FalseTarget);
//				continue;
//			}

			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if(abs(avIrValues[lIr] - avIrValues[rIr]) > IR_MIN_DIFF){
				setState(Adjusting);
			}

			break;

		case CollisionDetected:
			vTaskDelay(150);
			stop();
			vTaskDelay(20);
			turn(collisionSide*60);
			vTaskDelay(40);
			collisionSide=0;
			setState(OnCourse);
			break;

		case Avoiding:
			if(currentObstacleDistanceMeter != triggeredDistanceMeter) {
				setState(ObstacleDetected);
			}

			if(avDistanceValues[currentObstacleDistanceMeter] < DIST_SENSOR_OBSTACLE_VALUE) {
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
			if(max(avIrValues[lIr], avIrValues[rIr]) > IR_TARGET_VALUE){
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
