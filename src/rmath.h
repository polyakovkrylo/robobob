#ifndef RMATH_H_
#define RMATH_H_

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

#endif
