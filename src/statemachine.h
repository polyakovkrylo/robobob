#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

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

void setState(int state) {
	tracef("new state: %d", state);
	currentState = state;
}

#endif
