#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>
#include "commands.h"
#include "ARMcalib.h"
#include "Position.h"

//#define stdPosition

//#define CTRL_BY_MICROS
//#define CTRL_BY_ANGLE
#define CTRL_BY_CARSTESIAN

#define TOF
//#define TCS

#define SQUARE(x) (x*x)
#define CUBE(x) (x*x*x)
#define mm(x) (x/1000)

#define STEP 1
#define SERVO1_PIN 9	//BASE
#define SERVO2_PIN 10	//SHOULDER
#define SERVO3_PIN 11	//ELBOW
#define SERVO4_PIN 12	//GRABBER

#define TOF_SCL_PIN 17
#define TOF_SDA_PIN 16

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
#ifdef TOF
VL53L0X tof;
float distance, prev_distance;
#endif

#ifdef TCS
commands_t serial_commands;
int show_lux;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void process_command(char command, float value)
{
  if (command == 'L')
  { // The 'L' command controls if we calculate the Lux value

    show_lux = value; // L1<enter> to turn on Lux calculation, L0<enter> to switch off
  }
  else if (command == 'o')
  { // The 'o' command ...

  } // Put here more commands...
}
#endif

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
float speed = 20;

Position p(0,0,0);
float targetPos[3];
float currPos[3];

int s1dm(float rad);
int s2dm(float rad);
int s3dm(float rad);
int s4dm(int openClose);

float* moveToTarget(float* currP, float* targetP, float speed, int cylclePeriod, float& base, float& shoulder, float& elbow);
bool inverseKinematics(float *point, float& base, float& shoulder, float& elbow);
void doCommand1(char b, float* p);
void doCommand2(char b, float* p, int* step, int* openClose);
void doCommand3(char b, float* p, int* step);

void setup(){
	interval = 40;

	Serial.begin(115200);
	#ifdef TCS
	serial_commands.init(process_command);

	Wire.setSDA(4); // Connect TCS34725 SDA to gpio 8
	Wire.setSCL(5); // Connect TCS34725 SCL to gpio 9
	Wire.begin();

	while (!tcs.begin())
	{
		Serial.println("No TCS34725 found ... check your connections");
		delay(500);
	}

	Serial.println("Found sensor");
	#endif

	#ifdef TOF
	Wire.setSDA(TOF_SDA_PIN);
  	Wire.setSCL(TOF_SCL_PIN);
	Wire.begin();
	tof.setTimeout(500);
	while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  	}
	tof.startReadRangeMillimeters();
	#endif

	pinMode(LED_BUILTIN, OUTPUT);

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
	targetPos[0]=85;
	targetPos[1]=0;
	targetPos[2]=70;
	inverseKinematics(targetPos, angBase, angShoulder, angElbow);
	currPos[0]=targetPos[0];
	currPos[1]=targetPos[1];
	currPos[2]=targetPos[2];

	thB = 90;
	thS = 90;
	thE = 90;

	cmd = 90;
}

void loop(){
	uint8_t b;

	if(Serial.available()){
		b = Serial.read();
		#ifdef TCS
		serial_commands.process_char(b);
		#endif
		#ifndef TCS
		#ifdef CTRL_BY_CARSTESIAN
      	doCommand2(b, targetPos, &c_step, &openClose);
		#endif
		#ifdef CTRL_BY_ANGLE
		doCommand1(b, targetPos);
		#endif
		#ifdef CTRL_BY_MICROS
		doCommand3(b, targetPos, &c_step);
		#endif
		#endif
	}


	unsigned long now = millis();
	if (now - last_cycle > interval)
	{
		last_cycle = now;
		
		//	Read Inputs
		#ifdef TOF
		if (tof.readRangeAvailable()) {
      		prev_distance = distance;
      		distance = tof.readRangeMillimeters();
			tof.startReadRangeMillimeters();
    	}	
		#endif

		#ifdef TCS
		uint16_t r, g, b, c, colorTemp, lux;
		getRawData_noDelay(&r, &g, &b, &c);
		if (show_lux)
			lux = tcs.calculateLux(r, g, b);

			Serial.print("Color Temp: ");
			Serial.print(colorTemp, DEC);
			Serial.print(" K - ");
		if (show_lux)
			Serial.print("Lux: ");
			Serial.print(lux, DEC);
			Serial.print(" - ");
			Serial.print("R: ");
			Serial.print(r, DEC);
			Serial.print(" ");
			Serial.print("G: ");
			Serial.print(g, DEC);
			Serial.print(" ");
			Serial.print("B: ");
			Serial.print(b, DEC);
			Serial.print(" ");
			Serial.print("C: ");
			Serial.print(c, DEC);
			Serial.print(" ");
			Serial.print(" Command: ");
			Serial.print(serial_commands.command);
		#endif
		
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
		//inverseKinematics(targetPos, angBase, angShoulder, angElbow);
		moveToTarget(currPos, targetPos, speed, interval, angBase, angShoulder, angElbow);		

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
		#ifdef TOF
		Serial.print("   distance: ");
		Serial.print(distance, 3);
		#endif
		#ifdef CTRL_BY_CARSTESIAN
		Serial.print("   Speed: ");
		Serial.print(speed);
		Serial.print("   currX: ");
		Serial.print(currPos[0]);
		Serial.print("   currY: ");
		Serial.print(currPos[1]);
		Serial.print("   currZ: ");
		Serial.print(currPos[2]);

		Serial.print("   targetX: ");
		Serial.print(targetPos[0]);
		Serial.print("   targetY: ");
		Serial.print(targetPos[1]);
		Serial.print("   targetZ: ");
		Serial.print(targetPos[2]);

		Serial.print("   base: ");
		Serial.print(angBase*180/PI);
		Serial.print("   shoulder: ");
		Serial.print(angShoulder*180/PI);
		Serial.print("   elbow: ");
		Serial.print(angElbow*180/PI);
		#endif

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
	//y = 58.778x3 - 140.69x2 - 541.24x + 1831.7
	//int ret = (int)((58.778*CUBE(rad) - 140.69*SQUARE(rad) - 541.24*rad + 1831.7)+.5);
	//y = -150.86x3 + 318.31x2 - 835.73x + 1852.5
	int ret = (int)((-150.86*CUBE(rad) + 318*SQUARE(rad) - 835.73*rad + 1852.5)+.5);
	ret = (ret<min3)?min3:(ret>max3?max3:ret);
	
	return ret;
}

int s4dm(int openClose){
	return min4 + ((max4-min4)*openClose/100);
}

float map(float input, int min_in, int max_in, int min_out, int max_out){
	return max_out + ((max_in - input)/(max_in-min_in))*max_out;
}

float pitagoras(float a, float b, float c){
	return sqrt(SQUARE(a)+SQUARE(b)+SQUARE(c));
}

/**
 * Calculate the inverse kinematics for a robotic arm.
 *
 * @param point array of three float values representing the target point
 * @param base reference to a float to store the base angle in radians
 * @param shoulder reference to a float to store the shoulder angle in radians
 * @param elbow reference to a float to store the elbow angle in radians
 *
 * @return true if the inverse kinematics calculation is successful, false otherwise
 *
 * @throws None
 */
bool inverseKinematics(float *point, float& base, float& shoulder, float& elbow){
	//float qa = atan((p[0]-CAL_L_0)/-p[1]);
	float l = CAL_L_2;
	float x, y, z;

	x = point[0];
	y = point[1];
	z = point[2]; // max 130
	if(z < CAL_Z_BOT_CAP){
		return false;
	}

	float r = sqrt(SQUARE(x) + SQUARE(y));
	if(r <CAL_Hub_Diameter/2) {
		Serial.println("Point is too close to the hub");
		r  = CAL_Hub_Diameter/2;
	}
	float q1 = atan2(y,x)+PI/2;
	float hp = z - CAL_H_1;
	float HIP = sqrt(SQUARE(r) + SQUARE(hp));
	if(HIP > CAL_SUP_CAP){
		Serial.println("Point is too far from the hub");
		HIP = CAL_SUP_CAP;
	}
	//float q3_ = acos(2-(SQUARE(hp)+SQUARE(r))/SQUARE(l));
	float q3_ = acos(1-SQUARE(HIP)*0.5/SQUARE(l));

	float alpha = atan2(hp,r);
	float beta = (PI-q3_)/2;
	float q2 = (PI - alpha - beta);
	float q3 = q2-q3_;

	base = q1;		//in radians
	shoulder = q2;	//in radians
	elbow = q3;		//in radians

	return true;
}

float* moveToTarget(float* currP, float* targetP, float speed, int cylclePeriod, float& base, float& shoulder, float& elbow){
	if(currP[0] < targetP[0]+POS_MARGIN && currP[0] > targetP[0]-POS_MARGIN){
		if(currP[1] < targetP[1]+POS_MARGIN && currP[1] > targetP[1]-POS_MARGIN){
			if(currP[2] < targetP[2]+POS_MARGIN && currP[2] > targetP[2]-POS_MARGIN){
				return currP;
			}
		}
	}
	float sVec[3] = {targetP[0] - currP[0], targetP[1] - currP[1], targetP[2] - currP[2]};
	//Serial.printf("SVec: %.6f %.6f %.6f", sVec[0], sVec[1], sVec[2]);
	float sMod = pitagoras(sVec[0], sVec[1], sVec[2]);
	//Serial.printf("   SMod: %.6f", sMod);
	float sUnit[3] = {sVec[0]/sMod, sVec[1]/sMod, sVec[2]/sMod};
	//Serial.printf("   SUnit: %.6f %.6f %.6f", sUnit[0], sUnit[1], sUnit[2]);

	speed = (speed*powf(sMod*0.1, 1.8) > 150) ? 150 : speed*powf(sMod*0.1, 1.8);

	currP[0] = sUnit[0]*speed*1e-3*cylclePeriod + currP[0];
	currP[1] = sUnit[1]*speed*1e-3*cylclePeriod + currP[1];
	currP[2] = sUnit[2]*speed*1e-3*cylclePeriod + currP[2];

	inverseKinematics(currP, base, shoulder, elbow);
	return currP;
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

	if (b == 'z'){
		speed = (speed + c_step*10 > 150)? 150 : speed+c_step;
	}
	if (b == 'x'){
		speed = (speed - c_step*10 < 10)? 10 : speed-c_step;
	}

	if (b == 'h' || b == 's'){
		p[0] = (p[0] - c_step < -60) ? -60 : p[0]-c_step;
	}
	if (b == 'y' || b == 'w'){
		p[0] = (p[0] + c_step > 150) ? 150 : p[0]+c_step;
	}

	if (b == 'j' || b == 'd'){
		p[1] = (p[1] - c_step < -160) ? -160 : p[1]-c_step;
	}
	if (b == 'u' || b == 'a'){
		p[1] = (p[1] + c_step > 160) ? 160 : p[1]+c_step;
	}

	if (b == 'k' || b == 'f'){ // Z
		p[2] = (p[2] - c_step < -50) ? -50 : p[2]-c_step;
	}
	if (b == 'i' || b == 'r'){
		p[2] = (p[2] + c_step > 200) ? 200 : p[2]+c_step;
	}

	if(b == 'o' || b == 'q'){
		*openClose = (*openClose + c_step > 100) ? 100 : *openClose+c_step;
	}

	if(b == 'l' || b == 'e'){
		*openClose = (*openClose - c_step < 0) ? 0 : *openClose-c_step;
	}

	if (b == '1'){
		//pp.setxyz(140, 0, 140);
		p[0]=36;
		p[1]=-40;
		p[2]=70;
	}
	if (b == '2'){
		//pp.setxyz(140, -50, 140);
		p[0]=125;
		p[1]=-40;
		p[2]=70;
	}
	if (b == '3'){
		//pp.setxyz(140, 50, 140);
		p[0]=125;
		p[1]=40;
		p[2]=70;
	}
	if (b == '4'){
		//pp.setxyz(140, 0, 140);
		p[0]=36;
		p[1]=40;
		p[2]=70;
	}

	if (b == '5'){
		//pp.setxyz(140, 0, 140);
		p[0]=90;
		p[1]=90;
		p[2]=74;
	}
	if (b == '6'){
		//pp.setxyz(140, 0, 140);
		p[0]=90;
		p[1]=0;
		p[2]=74;
	}
	if (b == '7'){
		//pp.setxyz(140, 0, 140);
		p[0]=90;
		p[1]=-90;
		p[2]=74;
	}

	#ifdef TOF
	if (b == 'm'){
		//pp.setxyz(140, 0, 140);
		p[0]=distance-R_GRABBER_OFFSET;
		p[1]=0;
		p[2]=74;
	}
	#endif
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