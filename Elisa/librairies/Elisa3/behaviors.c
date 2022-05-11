#include "behaviors.h"

void initBehaviors() {

	srand(TCNT3);	// initialize random seed (used in obstacle avoidance)

}

void dispersion(signed int *pwmLeft, signed int *pwmRight){
	
	int largeThreshold = 140;
	int shortThreshold = 80;
	int max_speed = MAX_MOTORS_PWM/6;
	int testPause = 0;

	for(int i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
		int number=proximityResultLinear[i];
		char k[10];
		int i = 0;
		
		while(number >= 1){
			k[i] = number%10 + '0';
			i++;
			number /= 10;
		}
		for(int j = i-1; j >= 0; j--){
			usart0Transmit(k[j], 0);
		}
		usart0Transmit('\n', 0);
	}

	if(proximityResultLinear[7] > shortThreshold || proximityResultLinear[0] > shortThreshold || proximityResultLinear[1] > shortThreshold || proximityResultLinear[2] > shortThreshold){
		*pwmLeft = max_speed*0.8;
		*pwmRight = max_speed;
		testPause = 1;
	}else if (proximityResultLinear[6] > shortThreshold){
		*pwmLeft = max_speed*0.8;
		*pwmRight = max_speed;
		testPause = 1;
	}else if(proximityResultLinear[7] > largeThreshold || proximityResultLinear[0] > largeThreshold || proximityResultLinear[1] > largeThreshold){
		*pwmLeft = max_speed*0.8;
		*pwmRight = max_speed;
	}else if(proximityResultLinear[6] > largeThreshold){
		*pwmLeft = max_speed;
		*pwmRight = max_speed*0.8;
	}else if(proximityResultLinear[2] > largeThreshold){
		*pwmLeft = max_speed*0.8;
		*pwmRight = max_speed;
	}else{
		*pwmLeft = max_speed;
		*pwmRight = max_speed;
	}
	needPause = testPause;
}

void wallFollow(signed int *pwmLeft, signed int *pwmRight){

	for(int i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
		int number=proximityResultLinear[i];
		char k[10];
		int i = 0;
		
		while(number >= 1){
			k[i] = number%10 + '0';
			i++;
			number /= 10;
		}
		for(int j = i-1; j >= 0; j--){
			usart0Transmit(k[j], 0);
		}
		usart0Transmit('\n', 0);
	}

	int max_speed = MAX_MOTORS_PWM/7;
	*pwmLeft = max_speed;
	*pwmRight = max_speed;

	if(proximityResultLinear[0] > 80){
		*pwmLeft = max_speed/2;
		*pwmRight = -max_speed/2;
	}else{
		if(proximityResultLinear[6] > 60){
			*pwmLeft = max_speed;
			*pwmRight = max_speed;
		}else if(proximityResultLinear[5] > 80){
			*pwmLeft = max_speed*0.8;
			*pwmRight = max_speed;
		}else if(proximityResultLinear[7] > 80){		
			*pwmLeft = max_speed;
			*pwmRight = max_speed*0.8;
		}else{			
			*pwmLeft = max_speed*0.8;
			*pwmRight = max_speed;
		}
		
	}
}


void avoidance(signed int *pwmLeft, signed int *pwmRight){

	for(int i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
		int number=proximityResultLinear[i];
		char k[10];
		int i = 0;
		
		while(number >= 1){
			k[i] = number%10 + '0';
			i++;
			number /= 10;
		}
		for(int j = i-1; j >= 0; j--){
			usart0Transmit(k[j], 0);
		}
		usart0Transmit('\n', 0);
	}

	_Bool front = proximityResultLinear[0] > 80;
	_Bool back = proximityResultLinear[4] > 80;
	_Bool left = (int) (proximityResultLinear[6]*2 + proximityResultLinear[7] + proximityResultLinear[5])/4 > 80;
	_Bool right = (int) (proximityResultLinear[2]*2 + proximityResultLinear[3] + proximityResultLinear[1])/4 > 80;

	int speed = MAX_MOTORS_PWM/4;

	if(front){
		*pwmLeft = speed;
		*pwmRight = -speed;
	}else if(back){
		*pwmLeft = -speed;
		*pwmRight = speed;
	}else if(right){
		*pwmLeft = 0;
		*pwmRight = speed;
	}else if(left){
		*pwmLeft = speed;
		*pwmRight = 0;
	}else{
		*pwmLeft = speed;
		*pwmRight = speed;
	}	
}

void followLeader(signed int *pwmLeft, signed int *pwmRight){

	for(int i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
		int number=proximityResultLinear[i];
		char k[10];
		int i = 0;
		
		while(number >= 1){
			k[i] = number%10 + '0';
			i++;
			number /= 10;
		}
		for(int j = i-1; j >= 0; j--){
			usart0Transmit(k[j], 0);
		}
		usart0Transmit('\n', 0);
	}

	double braitenberg_coefficients[8][2] = { {10, 8}, {7, -1.5}, {5, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, 5}, {-1.5, 7} };
	double speed[2]={0,0};

	signed int front = proximityResultLinear[0];
	signed int left_front = proximityResultLinear[7];
	signed int right_front = proximityResultLinear[1];
	signed int left = proximityResultLinear[6];
	signed int right = proximityResultLinear[2];

	signed int back = (int) (proximityResultLinear[4]*2 + proximityResultLinear[3] + proximityResultLinear[5])/4;
	signed int spd = MAX_MOTORS_PWM/8;
	signed int spd_low = MAX_MOTORS_PWM/14;

	if(front > 200){
		*pwmLeft = spd*0.7;
		*pwmRight = spd*0.7;
	}else if(front > 80){
		*pwmLeft = spd;
		*pwmRight = spd;
	}/*else if(left > 80){
		*pwmLeft = -spd;
		*pwmRight = spd;
	}else if(right > 80){
		*pwmLeft = spd;
		*pwmRight = -spd;
	}*/else if(left_front > 80){
		*pwmLeft = 0;
		*pwmRight = spd;
	}else if(right_front > 80){
		*pwmLeft = spd;
		*pwmRight = 0;
	}else if(back > 80){
		*pwmLeft = spd_low;
		*pwmRight = spd_low;
	}else{
		for (int i = 0; i < 2; i++) {
			speed[i] = 0.0;
			for (int j = 0; j < 8; j++) {
				speed[i] += braitenberg_coefficients[j][i] *7* (1.0 - (proximityResultLinear[j] / IRRANGEVALUE));
			}
		}

		*pwmLeft = speed[0];
		*pwmRight = speed[1];
	}

	if (*pwmRight>(MAX_MOTORS_PWM/2)) *pwmRight=(MAX_MOTORS_PWM/2);
	if (*pwmLeft>(MAX_MOTORS_PWM/2)) *pwmLeft=(MAX_MOTORS_PWM/2);
	if (*pwmRight<-(MAX_MOTORS_PWM/2)) *pwmRight=-(MAX_MOTORS_PWM/2);
	if (*pwmLeft<-(MAX_MOTORS_PWM/2)) *pwmLeft=-(MAX_MOTORS_PWM/2);

}

void aggregationBehavior(signed int *pwmLeft, signed int *pwmRight){

	unsigned int i=0;
	signed int desL=*pwmLeft, desR=*pwmRight;

	double braitenberg_coefficients[8][2] = { {10, 8}, {7, -1.5}, {5, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, 5}, {-1.5, 7} };

	double speed[2]={0,0};

	int max = 0;

	// consider small values to be noise thus set them to zero in order to not influence
	// the resulting force
	for(i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
		int number=proximityResultLinear[i];
		char k[10];
		int i = 0;
		
		while(number >= 1){
			k[i] = number%10 + '0';
			i++;
			number /= 10;
		}
		for(int j = i-1; j >= 0; j--){
			usart0Transmit(k[j], 0);
		}
		usart0Transmit('\n', 0);
	}

	signed int front = 0;
	signed int back = 0;
	signed int left = 0;
	signed int right = 0;

	front = proximityResultLinear[0];
	back = proximityResultLinear[4];
	left = (int) (proximityResultLinear[6]*2 + proximityResultLinear[7] + proximityResultLinear[5])/4;
	right = (int) (proximityResultLinear[2]*2 + proximityResultLinear[3] + proximityResultLinear[1])/4;

	signed int spd = MAX_MOTORS_PWM/8;

	int detectionCeil = 80;
	int state = -1;

	if(back > detectionCeil){
		*pwmLeft = spd;
		*pwmRight = spd;
		state = 0;
	}else if(right > detectionCeil){
		*pwmLeft = -spd;
		*pwmRight = spd;
		state = 1;
	}else if(left > detectionCeil){
		*pwmLeft = 0;
		*pwmRight = spd;
		state = 2;
	}else if(front > detectionCeil){
		*pwmLeft = spd;
		*pwmRight = 0;
		state = 3;
	}else{
		for (i = 0; i < 2; i++) {
			speed[i] = 0.0;
			for (int j = 0; j < 8; j++) {
				speed[i] += braitenberg_coefficients[j][i] *8* (1.0 - (proximityResultLinear[j] / IRRANGEVALUE));
			}
		}

		*pwmLeft = speed[0];
		*pwmRight = speed[1];
	}

	// force the values to be in the pwm maximum range
	if (*pwmRight>(MAX_MOTORS_PWM/2)) *pwmRight=(MAX_MOTORS_PWM/2);
	if (*pwmLeft>(MAX_MOTORS_PWM/2)) *pwmLeft=(MAX_MOTORS_PWM/2);
	if (*pwmRight<-(MAX_MOTORS_PWM/2)) *pwmRight=-(MAX_MOTORS_PWM/2);
	if (*pwmLeft<-(MAX_MOTORS_PWM/2)) *pwmLeft=-(MAX_MOTORS_PWM/2);

	if(state == lastState && state != -1){
		countStop++;
	}else{
		countStop = 0;
	}

	if(countStop > 100){
		*pwmRight = 0;
		*pwmLeft = 0;
	}
	lastState = state;

}

void obstacleAvoidance(signed int *pwmLeft, signed int *pwmRight) {

	// Obstacle avoidance using all the proximity sensors based on a simplified 
	// force field method.
	//
	// General schema of the robot and related parameters:
	// 
	//		forward
	//
	//			0
	//		7		1
	//	velL	x	 velR
	//	  |		|	  |
	//	  6	  y_0	  2
	//
	//
	//		5		3
	//			4
	//
	// The follwoing table shows the weights (simplified respect to the trigonometry)
	// of all the proximity sensors for the resulting repulsive force:
	//
	//		0		1		2		3		4		5		6		7
	//	x	-1		-0.5	0		0.5		1		0.5		0		-0.5
	//	y	0		0.5		1		0.5		0		-0.5	-1		-0.5

	unsigned int i=0;
	signed int long res=0;
	signed int sumSensorsX=0, sumSensorsY=0;
	signed int desL=*pwmLeft, desR=*pwmRight;

	// consider small values to be noise thus set them to zero in order to not influence
	// the resulting force
	for(i=0; i<8; i++) {
		if(proximityResultLinear[i] < NOISE_THR) {
			proximityResultLinear[i] = 0;
		}
	}

	// sum the contribution of each sensor (based on the previous weights table);
	// give more weight to prox2 and prox6 (side proximities) in order to get more stability in narrow aisles;
	// add some noise to the sum in order to escape from dead-lock positions
	sumSensorsX = -proximityResultLinear[0] - (proximityResultLinear[1]>>1) + (proximityResultLinear[3]>>1) + proximityResultLinear[4] + (proximityResultLinear[5]>>1) - (proximityResultLinear[7]>>1) + ((rand()%60)-30);
	//sumSensorsX = -proximityResultLinear[0]  + proximityResultLinear[4];	
	sumSensorsY = (proximityResultLinear[1]>>1) + (proximityResultLinear[2]>>2) + (proximityResultLinear[3]>>1) - (proximityResultLinear[5]>>1) - (proximityResultLinear[6]>>2) - (proximityResultLinear[7]>>1)+ ((rand()%60)-30);
	//sumSensorsY = (proximityResultLinear[1]>>1) + (proximityResultLinear[3]>>1) - (proximityResultLinear[5]>>1) - (proximityResultLinear[7]>>1) + (rand()%30);

	// modify the velocity components based on sensor values
	if(desL >= 0) {
		res = (signed long int)desL + (((signed long int)(desL) * (signed long int)((signed long int)sumSensorsX - (signed long int)sumSensorsY))>>6); //7);
		*pwmLeft = (signed int)res;
	} else {
		res = (signed long int)desL - (((signed long int)(desL) * (signed long int)((signed long int)sumSensorsX + (signed long int)sumSensorsY))>>6); //7);
		*pwmLeft = (signed int)res;
	}
	if(desR >=0) {
		res = (signed long int)desR + (((signed long int)(desR) * (signed long int)((signed long int)sumSensorsX + (signed long int)sumSensorsY))>>6); //7);
		*pwmRight = (signed int)res;
	} else {
		res = (signed long int)desR - (((signed long int)(desR) * (signed long int)((signed long int)sumSensorsX - (signed long int)sumSensorsY))>>6); //7);
		*pwmRight = (signed int)res;
	}

	if(currentSelector == 10) {	// force a little bit the upwards direction in vertical motion
		if(desL!=0 && desR!=0) {
			if(currentAngle<270 && currentAngle>90) {
				*pwmLeft += 10;
				*pwmRight -= 10;
			} else {
				*pwmLeft -= 10;
				*pwmRight += 10;
			}
		}
	}

	// force the values to be in the pwm maximum range
	if (*pwmRight>(MAX_MOTORS_PWM/2)) *pwmRight=(MAX_MOTORS_PWM/2);
	if (*pwmLeft>(MAX_MOTORS_PWM/2)) *pwmLeft=(MAX_MOTORS_PWM/2);
	if (*pwmRight<-(MAX_MOTORS_PWM/2)) *pwmRight=-(MAX_MOTORS_PWM/2);
	if (*pwmLeft<-(MAX_MOTORS_PWM/2)) *pwmLeft=-(MAX_MOTORS_PWM/2);

}


char cliffDetected() {

	// tell whether a cliff is detected or not
	if(proximityResult[8]<CLIFF_THR || proximityResult[9]<CLIFF_THR || proximityResult[10]<CLIFF_THR || proximityResult[11]<CLIFF_THR) {
	//if(proximityResult[8]<(proximityOffset[8]>>1) || proximityResult[9]<(proximityOffset[9]>>1) || proximityResult[10]<(proximityOffset[10]>>1) || proximityResult[11]<(proximityOffset[11]>>1)) {
		return 1;
	} else {
		return 0;
	}


}

void enableObstacleAvoidance() {
	obstacleAvoidanceEnabled=1;
}

void disableObstacleAvoidance() {
	obstacleAvoidanceEnabled=0;
}

void enableCliffAvoidance() {
	cliffAvoidanceEnabled=1;
}

void disableCliffAvoidance() {
	cliffAvoidanceEnabled=0;
}

void enableAggregation() {
	aggregationEnabled=1;
}

void disableAggregation() {
	aggregationEnabled=0;
}

void enableFollowLeader() {
	followLeaderEnabled=1;
}

void disableFollowLeader() {
	followLeaderEnabled=0;
}

void enableAvoidance() {
	avoidanceEnabled=1;
}

void disableAvoidance() {
	avoidanceEnabled=0;
}

void enableWallFollow(){
	wallFollowEnabled=1;
}

void disableWallFollow(){
	wallFollowEnabled=0;
}

void enableDispersion(){
	dispersionEnabled=1;
}

void disableDispersion(){
	dispersionEnabled=0;
}

int getPause(){
	return needPause;
}