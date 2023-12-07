#include <Arduino.h>
#include <Servo.h>
#include "ARMcalib.h"
#include "Position.h"

//#define stdPosition
#define SQUARE(x) (x*x)
#define mm(x) (x/1000);

#define STEP 1
#define SERVO1_PIN 9	//BASE
#define SERVO2_PIN 10	//SHOULDER
#define SERVO3_PIN 11	//ELBOW
#define SERVO4_PIN 12	//GRABBER

int c_step = 1; // Configurable step

typedef struct{
	int state, new_state;

	// tes - time entering state
	// tis - time in state
	unsigned long tes, tis;
} fsm_t;

uint32_t interval, last_cycle;
// input variables

// Output variables
uint16_t cmd;
uint16_t cmd_micros;

// State Machines

// Other Variables


Servo Base;
Servo Shoulder;
Servo Elbow;
Servo Grabber;


float angBase;
float angShoulder;
float angElbow;
int microsBase;
int microsShoulder;
int microsElbow;
int thB, thS, thE;
Position p(0,0,0);
float point[3];

int s1dm(float deg);
int s2dm(float deg);
int s3dm(float deg);
int s4dm(float deg);

bool inverseKinematics(float p[3], float& base, float& shoulder, float& elbow);
void doCommand1(char b, float* p);
void doCommand2(char b, float* p, int* step);

void setup(){
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);

	interval = 40;
	Base.attach(SERVO1_PIN, 500, 2500);
	Shoulder.attach(SERVO2_PIN, 966, 2200);
	Elbow.attach(SERVO3_PIN, 1055, 1944);
	Grabber.attach(SERVO4_PIN, 722, 1955);

	#ifdef stdPosition
	Base.writeMicroseconds(1500);
	Shoulder.writeMicroseconds(1500);
	Elbow.writeMicroseconds(1500);
	Grabber.writeMicroseconds(1500);
	#endif

	//p.setxyz(140, 0, 140);
	point[0]=5;
	point[1]=5;
	point[2]=5;

	thB = 90;
	thS = 90;
	thE = 90;

	cmd = 90;
}

void loop(){
	uint8_t b;

	if(Serial.available()){
		b = Serial.read();
      	doCommand2(b, point, &c_step);
	}


	unsigned long now = millis();
	if (now - last_cycle > interval)
	{
		last_cycle = now;
		
		//	Read Inputs
		// 0: 1000
		// 90: 2000
		
		
		//analogWrite(SERVO1_PIN, cmd);

		//	Update State Machine Timers
		unsigned long cur_time = millis();
		//operation.tis 	 = cur_time - operation.tes;

		//	Calculate next states
		//		State Machine operation		- define main operation of the system	
		//		State Machine supDecipher	- helps to differentiate between a sup short press and a sup long press
		//		State Machine ledControl	- define control state of the leds
		

		//	Update States
		//set_state(operation, operation.new_state);
		

		//	Set Actions

		//	Set Outputs
		inverseKinematics(point, angBase, angShoulder, angElbow);
		Base.writeMicroseconds(s1dm(angBase));
		Shoulder.writeMicroseconds(s1dm(angShoulder));
		Elbow.writeMicroseconds(s1dm(angElbow));
		
		//	Serial Log
		Serial.print("   x: ");
		Serial.print(point[0]);
		Serial.print("   y: ");
		Serial.print(point[1]);
		Serial.print("   z: ");
		Serial.print(point[2]);
		Serial.print("   base: ");
		Serial.print(angBase);
		Serial.print("   shoulder: ");
		Serial.print(angShoulder);
		Serial.print("   elbow: ");
		Serial.print(angElbow);

		Serial.println();
	}
}


int s1dm(float deg){
	return min1 + ((max1-min1)*deg/180);
}

int s2dm(float deg){
	return min2 + ((max2-min2)*deg/180);
}

int s3dm(float deg){
	return min3 + ((max3-min3)*deg/180);
}

int s4dm(float deg){
	return min4 + ((max4-min4)*deg/180);
}

float map(float input, int min_in, int max_in, int min_out, int max_out){
	return max_out + ((max_in - input)/(max_in-min_in))*max_out;
}

float pitagoras(float a, float b){
	return sqrt(pow(a,2)+pow(b,2));
}

bool inverseKinematics(float *point, float& base, float& shoulder, float& elbow){
	//float qa = atan((p[0]-CAL_L_0)/-p[1]);
	float p[3];
	p[0] = point[0]/1000;
	p[1] = point[1]/1000;
	p[2] = point[2]/1000;

	float qa = atan2((p[0]-CAL_L_0),-p[1]);
	float qb = asin(CAL_D_5/sqrt(pow(p[0]-CAL_L_0, 2) + pow(p[1],2)));
	float q1 = qa + qb; // angle of base servo in radians


	// TODO arrumar isso- lembrar que os links do professor são diferentes
	// l1 dele é o nosso l2
	// l2 dele é o nosso l3
	// y é para cima
	// x é para frente
	// z é de lado a lado
	j3 = acos((SQUARE(p[0]) + SQUARE(p[1]) - SQUARE(CAL_L_2) - SQUARE(CAL_L_3))/(2*CAL_L_2*CAL_L_3))
	j2 = atan2(y,x)-atan2(l2-sin(j3), l1 + (l2 * cos(j3)));
	j1 = atan2 (y, x)

	float P_0W[3] = { //Positon x,y,z of the wrist
		p[0] + (CAL_L_5*sin(q1)-CAL_L_0),
		p[1] + (-CAL_L_5*cos(q1)),
		p[2] + (0)
	};

	float r = pitagoras(p[0],p[1])-CAL_L_1;
	float ze = p[2] - CAL_H_1;
	//float alpha = atan(ze/r);
	float alpha = atan2(ze,r);
	float s = pitagoras(r, ze);

	float gama = acos((pow(CAL_L_2,2)+pow(CAL_L_3,2)-pow(s,2))/(2*CAL_L_2*CAL_L_3));
	float beta = acos((pow(s,2)+pow(CAL_L_2,2)-pow(CAL_L_3,2))/(2*s*CAL_L_3));

	float q_3o = PI - gama;
	float q2 = PI - alpha - beta; // Angle of shoulder servo in radians

	float e = sqrt(
		pow(CAL_L_3O,2)+pow(CAL_L_2,2)-(2*CAL_L_3O*cos(q_3o))
	);

	float phi = acos((pow(e,2)+pow(CAL_L_3I,2)-pow(CAL_L_4,2))/(2*e*CAL_L_3I));
	float psi = asin(CAL_L_3O*sin(q_3o/e));
	float q3 = psi + phi + (PI/2) - q2; // Angle of elbow servo in radians

	base = q1*180/PI;
	shoulder = q2*180/PI;
	elbow = q3*180/PI;

	return true;
}

void doCommand1(char b, float* p){
	Serial.println(b);
	/* if (b == '-'){
			cmd = (cmd-STEP < 500) ? 500 : cmd-STEP;
		} 
      	if (b == '+'){
			cmd = (cmd+STEP > 2500) ? 2500 : cmd+STEP;
		} */
	if (b == '-'){
		cmd = (cmd-STEP < 0) ? 0 : cmd-STEP;
	} 
	if (b == '+'){
		cmd = (cmd+STEP > 180) ? 180 : cmd+STEP;
	}
	/* if (b == 'u'){
		x = (x-STEP < -80) ? -80 : x-STEP;
	} 
	if (b == 'j'){
		x = (x+STEP > 80) ? 80 : x+STEP;
	}
	if (b == 'i'){
		y = (y-STEP < 0) ? 0: y-STEP;
	} 
	if (b == 'k'){
		y = (y+STEP > 250) ? 250 : y+STEP;
	}
	if (b == 'o'){
		z = (z-STEP < 0) ? 0: z-STEP;
	} 
	if (b == 'l'){
		z = (z+STEP > 145) ? 145 : z+STEP;
	} */
	if (b == 'e'){
		thB = (thB-STEP < 0) ? 0: thB-STEP;
	} 
	if (b == 'd'){
		thB = (thB+STEP > 180) ? 180 : thB+STEP;
	}
	if (b == 'r'){
		thS = (thS-STEP < 33) ? 33: thS-STEP;
	} 
	if (b == 'f'){
		thS = (thS+STEP > 135) ? 135 : thS+STEP;
	}
	if (b == 't'){
		thE = (thE-STEP < 20) ? 20: thE-STEP;
	} 
	if (b == 'g'){
		thE = (thE+STEP > 128) ? 128 : thE+STEP;
	}
	if (b == '1'){
		//pp.setxyz(140, 0, 140);
		point[0]=180;
		point[1]=0;
		point[2]=140;
	}
	if (b == '2'){
		//pp.setxyz(140, -50, 140);
		point[0]=180;
		point[1]=-70;
		point[2]=140;
	}
	if (b == '3'){
		//pp.setxyz(140, 50, 140);
		point[0]=180;
		point[1]=70;
		point[2]=140;
	}
	if (b == '4'){
		//pp.setxyz(140, 0, 140);
		point[0]=180;
		point[1]=0;
		point[2]=170;
	}
	/* if (b == '1'){
		arm.gotoPoint(0,100,50);
	}
	if (b == '2'){
		arm.gotoPoint(0,300,50);
	}
	if (b == '3'){
		arm.gotoPoint(-120,100,50);
	}
	if (b == '4'){
		arm.gotoPoint(120,100,50);
	}
	if (b == '5'){
		arm.gotoPoint(0,120,200);
	}
	if (b == '6'){
		arm.gotoPoint(0,120,20);
	} */
}

void doCommand2(char b, float* p, int* step){
	if (b == '-'){
		c_step = (c_step-STEP < 1) ? 1 : c_step-STEP;
		Serial.print("c_step: ");
		Serial.println(c_step);
	}
	if (b == '+'){
		c_step = (c_step+STEP > 100) ? 100 : c_step-STEP;
		Serial.print("c_step: ");
		Serial.println(c_step);
	} 

	if (b == 'h'){
		p[0] = (p[0] - c_step < -50) ? -50 : p[0]-c_step;
	}
	if (b == 'y'){
		p[0] = (p[0] + c_step > 50) ? 50 : p[0]+c_step;
	}

	if (b == 'j'){
		p[1] = (p[1] - c_step < -50) ? -50 : p[1]-c_step;
	}
	if (b == 'u'){
		p[1] = (p[1] + c_step > 50) ? 50 : p[1]+c_step;
	}

	if (b == 'k'){ // Z
		p[2] = (p[2] - c_step < 0) ? 0 : p[2]-c_step;
	}
	if (b == 'i'){
		p[2] = (p[2] + c_step > 200) ? 200 : p[2]+c_step;
	}

	if (b == '1'){
		//pp.setxyz(140, 0, 140);
		point[0]=180;
		point[1]=0;
		point[2]=140;
	}
	if (b == '2'){
		//pp.setxyz(140, -50, 140);
		point[0]=180;
		point[1]=-70;
		point[2]=140;
	}
	if (b == '3'){
		//pp.setxyz(140, 50, 140);
		point[0]=180;
		point[1]=70;
		point[2]=140;
	}
	if (b == '4'){
		//pp.setxyz(140, 0, 140);
		point[0]=180;
		point[1]=0;
		point[2]=170;
	}
}