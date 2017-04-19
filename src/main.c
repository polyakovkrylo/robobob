//Code by Hector, sensor testing full program. (init and usage).

#include <stdlib.h>
#include "dorobo32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "adc.h"
#include "digital.h"
#include "fft.h"


//3600 min, you crashed

//300 no threat

static void blinky(void *pvParameters);



int main()
{
    dorobo_init();			//Call dorobo_init() function to initialize HAL, Clocks, Timers etc.	
    trace_init();
    adc_init();
    digital_init();
    ft_init();
    digital_configure_pin(DD_PIN_PD14, DD_CFG_INPUT_PULLUP);
    digital_configure_pin(DD_PIN_PD15, DD_CFG_INPUT_NOPULL);
	xTaskCreate(blinky, "BLINKYTASK", 512, NULL, 2, NULL);	//create blinky task

	vTaskStartScheduler();	//start the freertos scheduler

	return 0;				//should not be reached!
}

static void blinky(void *pvParameters) 
{


	while (1) 
	{

		tracef("ADC value: %d", adc_get_value(DA_ADC_CHANNEL0));

		if(digital_get_pin(DD_PIN_PD14)==DD_LEVEL_LOW){
			traces("switch pressed!!!!!!!");
		}

		ft_start_sampling(DD_PIN_PD15);
		while(!ft_is_sampling_finished()){}
		tracef("IR sensor: %d", ft_get_transform(DFT_FREQ125));

		led_green_toggle();
		vTaskDelay(100);				//delay the task for 20 ticks (1 ticks = 50 ms)
	}
}
