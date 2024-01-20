#include <Arduino.h>
#include <Servo.h>
#include "ARMcalib.h"
#include "Position.h"

#define stdPosition
//#define CTRL_BY_MICROS
//#define CTRL_BY_ANGLE
#define CTRL_BY_CARSTESIAN

#define SQUARE(x) (x*x)
#define CUBE(x) (x*x*x)
#define mm(x) (x/1000)

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

int microsBase;		//used for control by microseconds
int microsShoulder;	//used for control by microseconds
int microsElbow;	//used for control by microseconds
int microsGrabber;  //used for control by microseconds

float angBase;		//used for control by angle
float angShoulder;	//used for control by angle
float angElbow;		//used for control by angle
int thB, thS, thE;	//used for control by angle

int openClose = 0;

Position p(0,0,0);
float point[3];

int s1dm(float rad);
int s2dm(float rad);
int s3dm(float rad);
int s4dm(int openClose);

bool inverseKinematics(float *point, float& base, float& shoulder, float& elbow);
void doCommand1(char b, float* p);
void doCommand2(char b, float* p, int* step, int* openClose);
void doCommand3(char b, float* p, int* step);

void setup(){
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);

	interval = 40;
	Base.attach(SERVO1_PIN, min1, max1);
	Shoulder.attach(SERVO2_PIN, min2, max2);
	Elbow.attach(SERVO3_PIN, min3, max3);
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
	point[0]=85;
	point[1]=0;
	point[2]=70;

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
      	doCommand2(b, point, &c_step, &openClose);
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
		Grabber.writeMicroseconds(microsGrabber);
		#endif
		#ifdef CTRL_BY_ANGLE
		angBase = thB;
		angShoulder = thS;
		angElbow = thE;
		Base.writeMicroseconds(s1dm(thB*PI/180));
		Shoulder.writeMicroseconds(s1dm(thS*PI/180));
		Elbow.writeMicroseconds(s1dm(thE*PI/180));
		#endif
		#ifdef CTRL_BY_CARSTESIAN
		inverseKinematics(point, angBase, angShoulder, angElbow);
		
		microsBase = s1dm(angBase);
		microsShoulder = s2dm(angShoulder);
		microsElbow = s3dm(angElbow);
		microsGrabber = s4dm(openClose);

		Base.writeMicroseconds(microsBase);
		Shoulder.writeMicroseconds(microsShoulder);
		Elbow.writeMicroseconds(microsElbow);
		Grabber.writeMicroseconds(microsGrabber);
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
		Serial.print(angBase*180/PI);
		Serial.print("   shoulder: ");
		Serial.print(angShoulder*180/PI);
		Serial.print("   elbow: ");
		Serial.print(angElbow*180/PI);
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
		Serial.print("   micros G: ");
		Serial.print(microsGrabber);

		Serial.println();
	}
}

/**
 * @brief q1 servo conversion
 * 
 * @param rad Angle in Radians
 * @return int returns signal to servo 
 */
int s1dm(float rad){//q1 servo conversion
	//y = -13.78x2 + 679.91x + 500
	int ret = (int)((-13.78*SQUARE(rad) + 679.91*rad + 500)+.5);
	ret = (ret<min1)?min1:(ret>max1?max1:ret);

	return ret;
}

/**
 * @brief q2 servo conversion
 * 
 * @param rad Angle in Radians
 * @return int returns signal to servo microseconds
 */
int s2dm(float rad){//q2 servo conversion
	//y = -3.7518x2 + 724.38x + 313.4
	int ret = (int)((-3.7518*SQUARE(rad) + 724.38*rad + 313.4)+.5);
	ret = (ret<min2)?min2:(ret>max2?max2:ret);

	return ret;
}

/**
 * @brief q3 servo conversion
 * 
 * @param rad Angle in Radians
 * @return int returns signal to servo 
 */
int s3dm(float rad){//q3 servo conversion
	//y = -17.586x2 - 602.83x + 1834.7
	//int ret = (int)((-17.586*SQUARE(rad) - 602.83*rad + 1834.7)+.5);
	//y = 58.778x3 - 140.69x2 - 541.24x + 1831.7
	int ret = (int)((58.778*CUBE(rad) - 140.69*SQUARE(rad) - 541.24*rad - 541.24 + 1831.7)+.5);
	ret = (ret<min3)?min3:(ret>max3?max3:ret);
	
	return ret;
}

int s4dm(int openClose){
	return min4 + ((max4-min4)*openClose/100);
}

int s4dm(float rad){
	return min4 + ((max4-min4)*rad/180);
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
	float x, y, z;

	x = point[0];
	y = point[1];
	z = point[2];

	float r = sqrt(SQUARE(x) + SQUARE(y));
	Serial.print("r: ");
	Serial.printf("%.5f", r);
	Serial.print("     ");
	float q1 = atan2(y,x)+PI/2;
	float hp = z - CAL_H_1;
	Serial.print("hp: ");
	Serial.print(hp);
	Serial.print(" ");
	float S = sqrt(SQUARE(r) + SQUARE(hp));
	float q3_ = acos(2-(SQUARE(hp)+SQUARE(r))/SQUARE(l));

	float alpha = atan2(hp,r);
	float beta = (PI-q3_)/2;
	float q2 = PI - alpha - beta;
	float q3 = q2-q3_;

	base = q1;		//in radians
	shoulder = q2;	//in radians
	elbow = q3;		//in radians

	return true;
}

/**
 * Executes a command based on the given character input.
 *
 * @param b the character input representing the command
 * @param p a pointer to a float value
 *
 * @return void
 *
 * @throws None
 */
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

void doCommand2(char b, float* p, int* step, int* openClose){// by cartesian
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
		p[0] = (p[0] - c_step < -60) ? 60 : p[0]-c_step;
	}
	if (b == 'y'){
		p[0] = (p[0] + c_step > 150) ? 150 : p[0]+c_step;
	}

	if (b == 'j'){
		p[1] = (p[1] - c_step < -160) ? -160 : p[1]-c_step;
	}
	if (b == 'u'){
		p[1] = (p[1] + c_step > 160) ? 160 : p[1]+c_step;
	}

	if (b == 'k'){ // Z
		p[2] = (p[2] - c_step < -50) ? -50 : p[2]-c_step;
	}
	if (b == 'i'){
		p[2] = (p[2] + c_step > 200) ? 200 : p[2]+c_step;
	}

	if(b == 'o'){
		*openClose = (*openClose + c_step > 100) ? 100 : *openClose+c_step;
	}

	if(b == 'l'){
		*openClose = (*openClose - c_step < 0) ? 0 : *openClose-c_step;
	}

	if (b == '1'){
		//pp.setxyz(140, 0, 140);
		point[0]=85;
		point[1]=-40;
		point[2]=70;
	}
	if (b == '2'){
		//pp.setxyz(140, -50, 140);
		point[0]=100;
		point[1]=-40;
		point[2]=70;
	}
	if (b == '3'){
		//pp.setxyz(140, 50, 140);
		point[0]=100;
		point[1]=40;
		point[2]=70;
	}
	if (b == '4'){
		//pp.setxyz(140, 0, 140);
		point[0]=85;
		point[1]=40;
		point[2]=70;
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

	if (b == 'l'){
		microsGrabber = (microsGrabber - c_step < 500) ? 500 : microsGrabber-c_step;
	}
	if (b == 'o'){
		microsGrabber = (microsGrabber + c_step > 2500) ? 2500 : microsGrabber+c_step;
	}

	if (b == '1'){
		microsElbow = min3;
		microsShoulder = (min2+max2)/2;
		microsBase = (min1+max1)/2;
	}
	if (b == '2'){
		microsElbow = max3;
		microsShoulder = (min2+max2)/2;
		microsBase = (min1+max1)/2;
	}
}