#ifndef TARGETING_H_
#define TARGETING_H_

#define IR_MAX											1710
#define IR_TARGET_VALUE							300
#define IR_MIN_DIFF									400

#define BLIND_COUNTER_MAX           30

// IR Sensors
int irLeft = 0;
int irRight = 0;
int lastSeen = LEFT_DIRECTION;
int blindCounter = 0;

void updateBlindCounter()
{
  if(min(irLeft, irRight) < IR_TARGET_VALUE) {
    blindCounter++;
  } else {
    blindCounter = 0;
  }
}

bool isTargetLost()
{
  updateBlindCounter();
  if(blindCounter > BLIND_COUNTER_MAX)
    return true;
  else
    return false;
}

#endif
