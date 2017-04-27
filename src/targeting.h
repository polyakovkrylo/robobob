#ifndef TARGETING_H_
#define TARGETING_H_

#define IR_LEFT								DD_PIN_PC8
#define IR_RIGHT							DD_PIN_PC9

#define IR_MAX                  1710
#define IR_TARGET_VALUE         800
#define IR_MIN_DIFF             200

#define IR_ARRAY_SIZE						2

#define BLIND_COUNTER_MAX       30

// IR average values
int irLeft = 0;
int irRight = 0;

// Arrray of instant measurements
int irRightValues[IR_ARRAY_SIZE]={0};
int irLeftValues[IR_ARRAY_SIZE]={0};
// Instant measurements array iterator
int it_irValues=0;

int lastSeen = LEFT_DIRECTION;
int blindCounter = 0;

void readIrSensors()
{
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
}

void updateIrAverage()
{
  int rSum=0;
  int lSum=0;
  for(int i=0; i<IR_ARRAY_SIZE;i++){
    rSum+=irRightValues[i];
    lSum+=irLeftValues[i];
  }

  irLeft = lSum / DIST_ARRAY_SIZE;
  irRight = rSum / DIST_ARRAY_SIZE;
  tracef("level %d , %d", irLeft, irRight);
  // Update iterator
  it_irValues = (++it_irValues) % IR_ARRAY_SIZE;
}

void setLastSeen()
{
  if(irRight > irLeft]) {
    lastSeen = RIGHT_DIRECTION;
  }
  else if(irRight < irLeft) {
    lastSeen = LEFT_DIRECTION;
  }
}

void updateBlindCounter()
{
  if(min(irLeft, irRight) < IR_TARGET_VALUE) {
    blindCounter++;
  } else {
    blindCounter = 0;
  }
}

void resetBlindCounter()
{
	blindCounter = 0;
}

bool isTargetLost()
{
	return (blindCounter > BLIND_COUNTER_MAX);
}

#endif
