//Code by Hector Munoz and Vladimir Poliakov.

//possible improvement: entering while(1) inside robobob thread, ask for every measurement at the begining
//and save it in a variable with a name for more easily handling.
#include <stdlib.h>
#include "dorobo32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "adc.h"
#include "digital.h"
#include "fft.h"
#include "task.h"
#include "motor.h"

#define left_collision_pin			DD_PIN_PD14
#define right_collision_pin			DD_PIN_PD15
#define IR_LEFT						DD_PIN_PC8
#define IR_RIGHT					DD_PIN_PC9
#define distance_meter_pin			DA_ADC_CHANNEL0

#define TARGET_VALUE			300
#define MIN_TARGET_VALUE		100
#define MIN_DIFF				400
#define OBSTACLE_VALUE			800
//#define ( MAX( a, b ) ( ( a > b) ? a : b ) )

//3600 min, you crashed
//300 no threat
//ir 1500 max


//start(int speed) - speed form -100 to 100
//turn(int angularSpeed) - speed from -100 to 100
//stop()

enum {
	NoTarget,
	Grabbing,
	OnCourse,
	ObstacleDetected,
	Avoiding,
	ReturningOnCourse
}States;

static void irTask(void *pvParameters);
static void mainTask(void *pvParameters);

int irLeft = 0;
int irRight = 0;
int last_seen=1;
int lDistanceMeter = 0;
int lButton = DD_LEVEL_HIGH;
int rButton = DD_LEVEL_HIGH;

int currentState= NoTarget;
int blindCounter = 0;

int max(int x, int y){

	if(x>=y)
		return x;
	else
		return y;
}

int min(int x, int y){

	if(x>=y)
		return y;
	else
		return x;
}

int abs(int x) {
	if(x > 0)
		return x;
	else
		return -x;
}

int sign(int x) {
	if(x > 0)
		return 1;
	else if(x < 0)
		return -1;
	else
		return 0;
}

int main() {
    dorobo_init();
    trace_init();
    adc_init();
    digital_init();
    ft_init();
    motor_init();

    digital_configure_pin(left_collision_pin, DD_CFG_INPUT_PULLUP);
    digital_configure_pin(right_collision_pin, DD_CFG_INPUT_PULLUP);
    digital_configure_pin(IR_LEFT, DD_CFG_INPUT_NOPULL);
    digital_configure_pin(IR_RIGHT, DD_CFG_INPUT_NOPULL);

    xTaskCreate(mainTask, "mainTask", 256, NULL, 2, NULL);
    xTaskCreate(irTask, "irTask", 256, NULL, 3, NULL);

	vTaskStartScheduler();	//start the freertos scheduler
	return 0;				//should not be reached!
}

void setState(int state) {
	tracef("new state: %d", state);
	currentState = state;
}

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

static void irTask(void *pvParameters) {
	while(1){
		ft_start_sampling(IR_LEFT);
		while(!ft_is_sampling_finished()){
			vTaskDelay(10);
		}
		irLeft = ft_get_transform(DFT_FREQ125);

		//right infra red
		ft_start_sampling(IR_RIGHT);
		while(!ft_is_sampling_finished()){
			vTaskDelay(10);
		}
		irRight = ft_get_transform(DFT_FREQ125);
		vTaskDelay(40);

		if(irRight>irLeft){
			last_seen=-1;
		}
		else if(irRight<irLeft){
			last_seen=1;
		}
	}
}


static void mainTask(void *pvParameters){
	while(1){


		switch(currentState){

		case NoTarget:
			if((max(irLeft, irRight)>TARGET_VALUE))
				setState(Grabbing);
			else
				turn(50*last_seen);
			break;

		case Grabbing:
			tracef("distance sensor: %d", adc_get_value(distance_meter_pin));

			if((adc_get_value(distance_meter_pin)>OBSTACLE_VALUE)||(digital_get_pin(right_collision_pin)==DD_LEVEL_LOW)||(digital_get_pin(left_collision_pin)==DD_LEVEL_LOW)){

				setState(ObstacleDetected);
			}
			else if((abs(irLeft-irRight) < MIN_DIFF)){
				turn(0);
				setState(OnCourse);
			}

			else if(irLeft>irRight){
				turn(40);
			}
			else if(irLeft<irRight){
				turn(-40);
			}
			if(min(irLeft, irRight) < MIN_TARGET_VALUE){
				blindCounter++;
				if(blindCounter > 30){
					stop();
					blindCounter = 0;
					setState(NoTarget);
				}
			} else {
				blindCounter = 0;
			}

		break;

		case OnCourse:
			start(60);
			//tracef("distance sensor: %d", adc_get_value(distance_meter_pin));
			if((digital_get_pin(right_collision_pin)==DD_LEVEL_LOW)||(digital_get_pin(left_collision_pin)==DD_LEVEL_LOW)){
				setState(ObstacleDetected);
				//setState(Grabbing);
			}
			else if(adc_get_value(distance_meter_pin)>OBSTACLE_VALUE){
				setState(Avoiding);
			}
			else if(abs(irLeft - irRight) > MIN_DIFF){
				setState(Grabbing);
			}

			if(min(irLeft, irRight) < MIN_TARGET_VALUE){
				blindCounter++;
				if(blindCounter > 30) {
					blindCounter = 0;
					stop();
					setState(NoTarget);
				}
			} else {
				blindCounter = 0;
			}
		break;

		case ObstacleDetected:

			if(digital_get_pin(right_collision_pin)==DD_LEVEL_LOW){
				stop();
				turn(90);
			}
			else if(digital_get_pin(left_collision_pin)==DD_LEVEL_LOW){
				stop();
				turn(-90);
			}
			setState(Grabbing);
		break;

		case Avoiding:
			tracef("DISTANCEEEEEEEEEEE: %d", adc_get_value(distance_meter_pin));

		    turn (90);
		    if(adc_get_value(distance_meter_pin)<OBSTACLE_VALUE) {
		    	turn(0);
		    	setState(Grabbing);
		    }
		break;

		}
		vTaskDelay(30);
	}
}


/*static void mainTask(void *pvParameters) {
	while(1) {


		switch(currentState) {
		case NoTarget:
			turn(20);
			if(max(irLeft, irRight) > TARGET_VALUE){
				setState(Grabbing);
				turn(0);
			}
			break;

		case Grabbing:
			if(abs(irLeft - irRight) < MIN_DIFF) {
				turn(0);
				vTaskDelay(30);
			}
			if(abs(irLeft - irRight) < MIN_DIFF) {
				//setState(OnCourse);
			}
			turn((irLeft - irRight) / 10);
			break;

		case OnCourse:
			start(100);
			if(abs(irLeft - irRight) > MIN_DIFF)
				setState(Grabbing);
			break;
		}
		vTaskDelay(40);
	}
}*/

//static void robobob(void *pvParameters){
//	traces("starting main task");
//	int actual_state=no_target;
//	int offset;
//	while(1){
//		switch (actual_state){
//		case no_target:
//			if(ir_max<TARGET_VALUE){ //if((ir_max<TARGET_VALUE)){
//				actual_state=grabbing;
//			} else {
//				turn(25);
//			}
//			break;
//
//		case grabbing:
//			turn(25);
//			if((ir_left-ir_right) > MINIMUN_DIF){
//				actual_state=on_course;
//				turn(0);
//			}
//			break;
//
//		case on_course:
//			start(100);
//			if((right_collision==DD_LEVEL_LOW)||(left_collision==DD_LEVEL_LOW || distance_meter>OBSTACLE_VALUE)){
//				actual_state=collision_detected;
//			}
//			else if(distance_meter>OBSTACLE_VALUE){
//				actual_state=obsatcle_detected;
//			}
//			break;
//
//		case obstacle_detected:
//			//stop();
//			actual_state=avoiding;
//			break;
//
//		case avoiding:
//			//turn(25);
//			//start(25);
//			//turn(25);
//			if(distance_meter<OBSTACLE_VALUE){
//				actual_state=on_course;
//			}
//			break;
//		}
//
///*
//		tracef("ADC value: %d", adc_get_value(distance_meter_pin));
//		if(digital_get_pin(left_collision_pin)==DD_LEVEL_LOW){
//			traces("switch pressed!!!!!!!");
//		}
//		ft_start_sampling(ir_left_pin);
//		while(!ft_is_sampling_finished()){}
//		tracef("IR sensor: %d", ft_get_transform(DFT_FREQ125));
//		led_green_toggle();
//		vTaskDelay(500);				//delay the task for 20 ticks (1 ticks = 50 ms)*/
//	}
//}
//static void checking(void *pvParameters){
//	//adquisition of values from sensors:
///*
//	//left infra red
//	ft_start_sampling(ir_left_pin);
//	while(!ft_is_sampling_finished()){}
//	ir_left = ft_get_transform(DFT_FREQ125);
//
//	//right infra red
//	ft_start_sampling(ir_right_pin);
//	while(!ft_is_sampling_finished()){}
//	ir_right = ft_get_transform(DFT_FREQ125);
//
//	//distance meter
//	distance_meter = adc_get_value(distance_meter_pin);
//
//	//maximun infra red
//	ir_max=max(ir_left, ir_right);
//
//	//collision detector left
//	left_collision=digital_get_pin(left_collision_pin);
//
//	//collision detector right
//	right_collision=digital_get_pin(right_collision_pin);
//
//
//	*/
//	//test//////////////////////////////////
//	//led_green_toggle();
//	traces("task1");
//	while(1){
//		tracef("ADC value: %d", adc_get_value(distance_meter_pin));
//			if(digital_get_pin(left_collision_pin)==DD_LEVEL_LOW){
//				traces("switch pressed!!!!!!!");
//			}
//			ft_start_sampling(ir_left_pin);
//			while(!ft_is_sampling_finished()){}
//			tracef("IR sensor: %d", ft_get_transform(DFT_FREQ125));
//			led_green_toggle();
//			traces("checked!!!!!!!");
//			vTaskDelay(300);				//delay the task for 20 ticks (1 ticks = 50 ms)
//
//			//taskYIELD();
//
//	}
//
//
//}



