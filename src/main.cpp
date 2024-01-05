#include <Arduino.h>
#include <Servo.h>
#include "ARMcalib.h"
#include "Position.h"

#define stdPosition
//#define CTRL_BY_MICROS
//#define CTRL_BY_ANGLE
#define CTRL_BY_CARSTESIAN

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

bool inverseKinematics(float *point, float& base, float& shoulder, float& elbow);
void doCommand1(char b, float* p);
void doCommand2(char b, float* p, int* step);
void doCommand3(char b, float* p, int* step);

void setup(){
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);

	interval = 40;
	Base.attach(SERVO1_PIN, 500, 2500);
	Shoulder.attach(SERVO2_PIN, 966, 2500);
	Elbow.attach(SERVO3_PIN, 500, 2500);
	Grabber.attach(SERVO4_PIN, 722, 1955);

	#ifdef stdPosition
	microsBase = 1500;
	microsShoulder = 1500;
	microsElbow = 1500;
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
		#ifdef CTRL_BY_CARSTESIAN
      	doCommand2(b, point, &c_step);
		#endif
		#ifdef CTRL_BY_ANGLE
		doCommand1(b, point);
		#endif
		#ifdef CTRL_BY_MICROS
		doCommand3(b, point, &c_step);
		#endif
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
		#ifdef CTRL_BY_MICROS
		Base.writeMicroseconds(microsBase);
		Shoulder.writeMicroseconds(microsShoulder);
		Elbow.writeMicroseconds(microsElbow);
		#endif

		#ifndef CTRL_BY_CARSTESIAN
		#ifdef CTRL_BY_ANGLE
		angBase = thB;
		angShoulder = thS;
		angElbow = thE;
		Base.writeMicroseconds(s1dm(thB));
		Shoulder.writeMicroseconds(s1dm(thS));
		Elbow.writeMicroseconds(s1dm(thE));
		#endif
		#endif
		#ifdef CTRL_BY_CARSTESIAN
		inverseKinematics(point, angBase, angShoulder, angElbow);
		Base.writeMicroseconds(s1dm(angBase));
		Shoulder.writeMicroseconds(s1dm(angShoulder));
		Elbow.writeMicroseconds(s1dm(angElbow));
		#endif
		
		//	Serial Log
		Serial.print("Step: ");
		Serial.print(c_step);
		#ifdef CTRL_BY_CARSTESIAN
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
		#endif

		Serial.print("   base: ");
		Serial.print(angBase);
		Serial.print("   shoulder: ");
		Serial.print(angShoulder);
		Serial.print("   elbow: ");
		Serial.print(angElbow);

		Serial.print("   micros B: ");
		Serial.print(microsBase);
		Serial.print("   micros S: ");
		Serial.print(microsShoulder);
		Serial.print("   micros E: ");
		Serial.print(microsElbow);

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
	float l = CAL_L_2;
	float p[3];
	p[0] = point[0]/1000;
	p[1] = point[1]/1000;
	p[2] = point[2]/1000;
	float r = sqrt(SQUARE(p[0]) + SQUARE(p[1]));
	float q1 = atan2(p[0],p[1]);
	float z = p[2];
	float hp = z - CAL_H_1;
	float alpha = atan2(r, hp);
	float S = r/cos(alpha);
	float q3_ = acos(1-(SQUARE(S)/(2*SQUARE(l))));
	float Beta = (PI-q3_)/2;
	float q2 = PI - Beta - alpha;
	float q3 = PI/2 - q3_;


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
		c_step = (c_step+STEP > 100) ? 100 : c_step+STEP;
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

void doCommand3(char b, float* p, int* step){
	if (b == '-'){
		c_step = (c_step-STEP < 1) ? 1 : c_step-STEP;
		Serial.print("c_step: ");
		Serial.println(c_step);
	}
	if (b == '+'){
		c_step = (c_step+STEP > 100) ? 100 : c_step+STEP;
		Serial.print("c_step: ");
		Serial.println(c_step);
	} 

	if (b == 'h'){
		microsBase = (microsBase - c_step < 500) ? 500 : microsBase-c_step;
	}
	if (b == 'y'){
		microsBase = (microsBase + c_step > 2500) ? 2500 : microsBase+c_step;
	}

	if (b == 'j'){
		microsShoulder = (microsShoulder - c_step < 500) ? 500 : microsShoulder-c_step;
	}
	if (b == 'u'){
		microsShoulder = (microsShoulder + c_step > 2500) ? 2500 : microsShoulder+c_step;
	}

	if (b == 'k'){ // Z
		microsElbow = (microsElbow - c_step < 500) ? 500 : microsElbow-c_step;
	}
	if (b == 'i'){
		microsElbow = (microsElbow + c_step > 2500) ? 2500 : microsElbow+c_step;
	}
}