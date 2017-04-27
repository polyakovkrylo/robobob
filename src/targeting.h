#ifndef TARGETING_H_
#define TARGETING_H_

#define IR_MAX									1710
#define IR_TARGET_VALUE							800
#define IR_MIN_DIFF								150

#define BLIND_COUNTER_MAX           			25
#define FALSE_TARGET_COUNTER_MAX        		10

// IR Sensors
int irLeft = 0;
int irRight = 0;

int lastSeen = LEFT_DIRECTION;
int blindCounter = 0;

int falseTargetCounter = 0;

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

bool isFalseTarget() {
	if(max(irLeft, irRight) < (IR_MAX - IR_MIN_DIFF))
		falseTargetCounter++;
	return falseTargetCounter > FALSE_TARGET_COUNTER_MAX;
}

void resetFalseTargetCounter() {
	falseTargetCounter = 0;
}

#endif
