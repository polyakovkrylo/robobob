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

enum States{
	NoTarget,
	Grabbing,
	OnCourse,
	ObstacleDetected,
	Avoiding,
	ReturningOnCourse
};

// System state
int currentState = NoTarget;

// Distance meters
int lDistanceMeter = 0;


static void irTask(void *pvParameters);
static void mainTask(void *pvParameters);
void setState(int state);

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
  xTaskCreate(irTask, "irTask", 256, NULL, 3, NULL);

	vTaskStartScheduler();	//start the freertos scheduler
	return 0;								//should not be reached!
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

void setState(int state) {
	tracef("new state: %d", state);
	currentState = state;

  // Do entering routine
  switch(state) {

		case NoTarget:
			stop();
      turn(50 * lastSeen);
		break;

		case onCourse:
      turn(0);
    break;

    case onCourse:
      turn(0);
      start(60);
    break;
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
			// Move it to different task !!!!!!!!!!
			if((adc_get_value(DISTANCE_METER_RIGHT)>DIST_SENSOR_OBSTACLE_VALUE)||
				(digital_get_pin(COLLISSION_SENSOR_RIGHT)==DD_LEVEL_LOW)||
				(digital_get_pin(COLLISSION_SENSOR_LEFT)==DD_LEVEL_LOW)) {
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
			start(60);
			//tracef("distance sensor: %d", adc_get_value(DISTANCE_METER_RIGHT));
			if((adc_get_value(DISTANCE_METER_RIGHT)>DIST_SENSOR_OBSTACLE_VALUE)||
				(digital_get_pin(COLLISSION_SENSOR_RIGHT)==DD_LEVEL_LOW)||
				(digital_get_pin(COLLISSION_SENSOR_LEFT)==DD_LEVEL_LOW)) {
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

		case ObstacleDetected:
			if(digital_get_pin(COLLISSION_SENSOR_RIGHT)==DD_LEVEL_LOW){
				stop();
				turn(90);
			}
			else if(digital_get_pin(COLLISSION_SENSOR_LEFT)==DD_LEVEL_LOW){
				stop();
				turn(-90);
			}
			setState(Grabbing);
		break;
		}
		vTaskDelay(20);
	}
}
