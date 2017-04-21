//Code by Hector Munoz and Vladimir Poliakov.

//possible improvement: entering while(1) inside robobob thread, ask for every measurement at the begining
//and save it in a variable with a name for more easily handling.
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "dorobo32.h"
#include "trace.h"
#include "adc.h"
#include "digital.h"
#include "fft.h"
#include "task.h"
#include "motor.h"

#include "rmath.h"
#include "movement.h"
#include "targeting.h"

#define COLLISSION_SENSOR_LEFT		DD_PIN_PD14
#define COLLISSION_SENSOR_RIGHT		DD_PIN_PD15
#define IR_LEFT										DD_PIN_PC8
#define IR_RIGHT									DD_PIN_PC9
#define DISTANCE_METER_RIGHT			DA_ADC_CHANNEL0
#define DISTANCE_METER_LEFT				DA_ADC_CHANNEL1

#define DIST_SENSOR_MAX							3600
#define DIST_SENSOR_MIN							300
#define DIST_SENSOR_OBSTACLE_VALUE	400
#define DIST_ARRAY_SIZE							10

enum States{
	NoTarget,
	Grabbing,
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
const int triggeredDistanceMeter = 0;
int avDistanceValues[3] = 0;
int lDistanceValues[DIST_ARRAY_SIZE] = 0;
int rDistanceValues[DIST_ARRAY_SIZE] = 0;
int it_currentValueIndex = 0;

// Collision Sensors
int triggeredCollisionSensor = 0;

void setState(int state);
static void irTask(void *pvParameters);
static void mainTask(void *pvParameters);
static void distanceTask(void *pvParameters);

int main() {
	// Initialization
  dorobo_init();
  trace_init();
  adc_init();
  digital_init();

	// Pin configuration
  digital_configure_pin(COLLISSION_SENSOR_LEFT, DD_CFG_INPUT_PULLUP);
  digital_configure_pin(COLLISSION_SENSOR_RIGHT, DD_CFG_INPUT_PULLUP);
  // digital_configure_pin(IR_LEFT, DD_CFG_INPUT_NOPULL);
  // digital_configure_pin(IR_RIGHT, DD_CFG_INPUT_NOPULL);

  xTaskCreate(mainTask, "mainTask", 256, NULL, 2, NULL);
  xTaskCreate(irTask, "irTask", 256, NULL, 4, NULL);
	xTaskCreate(distanceTask, "distanceTask", 256, NULL, 3, NULL);

	vTaskStartScheduler();	//start the freertos scheduler
	return 0;								//should not be reached!
}

void setState(int state) {
	tracef("new state: %d", state);
	currentState = state;

  // Do entering routine
  switch(state) {

		case NoTarget:
			stop();
      turn(50 * lastSeen);
		break;

		case Grabbing:
      turn(0);
    break;

    case onCourse:
      turn(0);
      start(60);
    break;

		case ObstacleDetected:
			stop();
			if(triggeredDistanceMeter == lDistanceMeter)
				start(30, LEFT_DIRECTION);
			else
				start(30, RIGHT_DIRECTION);
			setState(Avoiding);
		break;

		case CollisionDetected:
			stop();
			turn(90 * triggeredCollisionSensor);
			vTaskDelay(30);
			start(60);
			vTaskDelay(100);
			setState(Avoiding);
		break;
  }
}

static void distanceTask(void *pvParameters) {
	// Read sensors
	rDistanceValues[it_currentValueIndex] = adc_get_value(DISTANCE_METER_RIGHT);
	lDistanceValues[it_currentValueIndex] = adc_get_value(DISTANCE_METER_LEF);

	// Calculate average
	int lSum = 0, rSum = 0;
	for (int i = 0; i < DIST_ARRAY_SIZE; i++) {
		lSum += lDistanceValues[i];
		rSum += rDistanceValues[i];
	}
	avDistanceValues[lDistanceMeter] = lSum / DIST_ARRAY_SIZE;
	avDistanceValues[rDistanceMeter] = lSum / DIST_ARRAY_SIZE;

	// Check if obstacle was detected
	if(avDistanceValues[lDistanceMeter] > OBSTACLE_VALUE) {
		triggeredDistanceMeter = lDistanceMeter;
	} else if(avDistanceValues[rDistanceMeter] > OBSTACLE_VALUE) {
		triggeredDistanceMeter = rDistanceMeter;
	} else {
		triggeredDistanceMeter = 0;
	}

	it = (++it) % DIST_ARRAY_SIZE; //moving the iterator
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

		vTaskDelay(20);
	}
}

static void mainTask(void *pvParameters){
	// Motor initialization
	motor_init();

	while(1){
		switch(currentState){

		case NoTarget:
			if(max(irLeft, irRight) > IR_TARGET_VALUE)
				setState(Grabbing);
		break;

		case Grabbing:
			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if((abs(irLeft-irRight) < IR_MIN_DIFF)){
				setState(OnCourse);
			}
			else if(irLeft>irRight){
				turn(40);
			}
			else {
				turn(-40);
			}

			if(isTargetLost())
				setState(NoTarget);
		break;

		case OnCourse:
			if(triggeredDistanceMeter != 0) {
				setState(ObstacleDetected);
				continue;
			}

			if(abs(irLeft - irRight) > IR_MIN_DIFF){
				setState(Grabbing);
			}

			if(isTargetLost()) {
				setState(NoTarget)
			}
		break;

		case Avoiding:
			updateBlindCounter();
			if(avDistanceValues[triggeredDistanceMeter] < OBSTACLE_VALUE) {
				stop();
				setState(Grabbing);
			}
		break;

		}

		vTaskDelay(20);
	}
}
