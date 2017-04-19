//Code by Hector Munoz, sensor testing full program. (init and usage).
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


#define left_collision_pin			DD_PIN_PD14
#define right_collision_pin			DD_PIN_PD15
#define ir_left_pin					DD_PIN_PC8
#define ir_right_pin				DD_PIN_PC9
#define distance_meter_pin			DA_ADC_CHANNEL0


#define no_target				0
#define grabbing				1
#define on_course				2
#define collision_detected		3
#define avoiding				4
#define obsitcle_detected		5

#define TARGET_VALUE			1500	
#define MINIMUN_DIF				25
#define OBSTACLE_VALUE			3300
//#define ( MAX( a, b ) ( ( a > b) ? a : b ) )

//3600 min, you crashed
//300 no threat
//ir 1500 max


//start(int speed) - speed form -100 to 100
//turn(int angularSpeed) - speed from -100 to 100
//stop()

static void robobob(void *pvParameters);
//static void checking(void *pvParameters);


int main(){
    dorobo_init();	
    trace_init();
    adc_init();
    digital_init();
    ft_init();
    digital_configure_pin(left_collision_pin, DD_CFG_INPUT_PULLUP);
    digital_configure_pin(right_collision_pin, DD_CFG_INPUT_PULLUP);
    digital_configure_pin(ir_left_pin, DD_CFG_INPUT_NOPULL);
    digital_configure_pin(ir_right_pin, DD_CFG_INPUT_NOPULL);
    
	xTaskCreate(robobob, "ROBOBOB", 512, NULL, 2, NULL);
	//xTaskCreate(checking, "Checking for measurements", 512, NULL, 2, NULL);
	
	vTaskStartScheduler();	//start the freertos scheduler
	return 0;				//should not be reached!
}

static void robobob(void *pvParameters){
	int actual_state=no_target, rotation_time=0;
	int ir_left, ir_right, ir_max, ir_min, distance_meter, left_collision, right_collision;
	
	
	
	while(1){
		//adquisition of values from sensors:
		
		//left infra red
		ft_start_sampling(ir_left_pin);
		while(!ft_is_sampling_finished()){}
		ir_left = ft_get_transform(DFT_FREQ125);
		
		//right infra red
		ft_start_sampling(ir_right_pin);
		while(!ft_is_sampling_finished()){}
		ir_right = ft_get_transform(DFT_FREQ125);
		
		//distance meter
		distance_meter = adc_get_value(distance_meter_pin);
				
		//maximun infra red
		ir_max=max(ir_left, ir_right);
		
		//collision detector left
		left_collision=digital_get_pin(left_collision_pin);
		
		//collision detector right
		right_collision=digital_get_pin(right_collision_pin);
				
		switch (actual_state){
			case no_target:
				if(ir_max<TARGET_VALUE){ //if((ir_max<TARGET_VALUE)||(rotation_time>10)){
					actual_state=grabbing;
					rotation_time=0;
				}
			break;
			
			case grabbing:
				turn(25);
				if(rotation_time>3){
					actual_state=no_target;
				}
				else if((ir_left-ir_right) > MINIMUN_DIF){
					actual_state=on_course;
				}
				rotation_time++;
			break;
			
			case on_course:
				start(25);
				if((ir_left==0)&&(ir_right==0)){	//target lost
					actual_state=no_target;
				}
				else if((right_collision==DD_LEVEL_LOW)||(left_collision==DD_LEVEL_LOW)){
					actual_state=collision_detected;
				}
				else if(distance_meter>OBSTACLE_VALUE){
					actual_state=obstacle_detected;
				}
			break;
			
			case collision_detected:
				stop();
				actual_state=avoiding;
			break;
			
			case avoiding:
				turn(25);
				start(25);
				//turn(25);
				if(distance_meter<OBSTACLE_VALUE){
					actual_state=on_course;
				}
			break;
			
			case obsatcle_detected:
				actual_state=avoiding;
			break;
		}
/*
		tracef("ADC value: %d", adc_get_value(distance_meter));

		if(digital_get_pin(left_collision_pin)==DD_LEVEL_LOW){
			traces("switch pressed!!!!!!!");
		}

		ft_start_sampling(ir_left);
		while(!ft_is_sampling_finished()){}
		tracef("IR sensor: %d", ft_get_transform(DFT_FREQ125));

		led_green_toggle();*/
		vTaskDelay(20);				//delay the task for 20 ticks (1 ticks = 50 ms)
	}
}

int max(int x, int y){
	
	if (x>=y){
		return x;
	}
	else{
		return y;
	}
}

