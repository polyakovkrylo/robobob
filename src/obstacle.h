#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "adc.h"
#include "trace.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

#define DISTANCE_METER_RIGHT				DA_ADC_CHANNEL0
#define DISTANCE_METER_LEFT					DA_ADC_CHANNEL1

#define DIST_SENSOR_MAX								3600
#define DIST_SENSOR_MIN								250
#define DIST_SENSOR_OBSTACLE_VALUE		800
#define DIST_ARRAY_SIZE								5

// Distance meters
const int distanceRight = 0;
const int distanceLeft = 0;

int triggeredDistanceMeter = 0;
int currentObstacleDistanceMeter = 0;

int lDistanceValues[DIST_ARRAY_SIZE] = {0};
int rDistanceValues[DIST_ARRAY_SIZE] = {0};

int it_distanceValues = 0;

// Collision Sensors
int collisionSide = 0;

void initCollisionSensors()
{
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
}

void readDistanceSensors()
{
  rDistanceValues[it_distanceValues] = adc_get_value(DISTANCE_METER_RIGHT);
  lDistanceValues[it_distanceValues] = adc_get_value(DISTANCE_METER_LEFT);
}

void updateDistanceAverage()
{
  int rSum = 0;
  int lSum = 0;
  for (int i = 0; i < DIST_ARRAY_SIZE; i++) {
    lSum += lDistanceValues[i];
    rSum += rDistanceValues[i];
  }
  distanceLeft = lSum / DIST_ARRAY_SIZE;
  distanceRight = rSum / DIST_ARRAY_SIZE;
  it_distanceValues = (++it_distanceValues) % DIST_ARRAY_SIZE;
}

checkForTriggeredSide()
{
  if(max(distanceRight, distanceLeft) > DIST_SENSOR_OBSTACLE_VALUE) {
    if(distanceLeft > distanceRight) {
      triggeredDistanceMeter = LEFT_DIRECTION;
    } else {
      triggeredDistanceMeter = RIGHT_DIRECTION;
    }
  } else {
    triggeredDistanceMeter = 0;
  }
}

#endif
