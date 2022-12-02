#include <ESP32Servo.h>

const int SERVO_AMOUNT = 6;
const int MIN_MICROSECONDS = 550; // By the book is supposed to be 500, the values being set are a result of experimental tests from the author of the library
const int MAX_MICROSECONDS = 2350; // By the book is supposed to be 2500, the values being set are a result of experimental tests from the author of the library
const int PWM_FREQUENCY = 50;

// create four servo objects 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo_list[6] = {servo1, servo2, servo3, servo4, servo5, servo6};

const char PWM_PINS_AVAILABLE[] = { 12, 13, 14, 15, 16, 17 };

void setup() {
	Serial.begin(9600);

	for (int servo_index=0; servo_index<6; servo_index++){
		servo_list[servo_index].setPeriodHertz(PWM_FREQUENCY);
		servo_list[servo_index].attach(PWM_PINS_AVAILABLE[servo_index], MIN_MICROSECONDS, MAX_MICROSECONDS);
	}
}

void loop() {
	// If serial instruction is available
		// Check that instruction is valid
		int q[6] = {30, 60, 90, 120, 150, 180};
		// Iterate through each value in the q array and assign it to the right servo
		for(int i=0; i<SERVO_AMOUNT; i++){
			int angle = q[i];
			servo_list[i].write(angle);
			Serial.print("Servo #");
			Serial.print(i);
			Serial.print(" set to ");
			Serial.print(angle);
			Serial.println();
		}
}


/* THIS SECTION IS MODIFIED CODE IN AN ATTEMPT TO MAKE IT MORE DYNAMIC (TAKE ROBOT JOINTS AND TYPES AND ALL OTHER VARIABLES OF THE SYSTEM FROM A SERIAL INSTRUCTION)
CODE ABOVE IS MOSTLY HARDCODED TO GET IT TO WORK :)

#include <ESP32Servo.h>

const int MAX_SERVO_AMOUNT = 6;
const char PWM_PINS_AVAILABLE[] = { 12, 13, 14, 15, 16, 17 }; // PWM pins in ESP32 that will be used to communicate with the servoes
const int MIN_MICROSECONDS = 500;
const int MAX_MICROSECONDS = 2500;
const int PWM_FREQUENCY = 50;

Servo* servo_list[MAX_SERVO_AMOUNT];

int array_length(Servo* servo_array[]){
	int array_length = sizeof(servo_array) / sizeof(Servo*);
	return array_length;
}

bool register_servos(int servo_amount){

	if (servo_amount > MAX_SERVO_AMOUNT){ return false; }

	for (int i=0; i<servo_amount; i++){
		// Create a servo object
		Servo* s = new Servo;

		// Set up servo variables
		s->setPeriodHertz(PWM_FREQUENCY);
		s->attach(PWM_PINS_AVAILABLE[i], MIN_MICROSECONDS, MAX_MICROSECONDS);

		// Append it to the list of servoes
		servo_list[i] = s;

		// Serial message of feedback for what was done
		Serial.print("Servo #");
		Serial.print(i+1);
		Serial.print(" registered");
		Serial.println();
	}
	return true;
}

// create four servo objects 
// Servo servo1;
// Servo servo2;
// Servo servo3;
// Servo servo4;
// Servo servo5;

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
// int servo1Pin = 15;
// int servo2Pin = 16;

int angles[13] = {0, 30, 60, 90, 120, 150, 180, 150, 120, 90, 60, 30, 0};
int position_delay = 1000;

void setup() {
	Serial.begin(9600);
	// servo1.setPeriodHertz(50);      // Standard 50hz servo
	// servo2.setPeriodHertz(50);      // Standard 50hz servo
	// servo1.attach(servo1Pin, MIN_MICROSECONDS, MAX_MICROSECONDS);
	// servo2.attach(servo2Pin, MIN_MICROSECONDS, MAX_MICROSECONDS);
	register_servos(2); // This number comes from a serial command given by the computer
}

void loop() {
	// =================== SERVO #1 SWEEP ===================
	// for (int i=0; i < 13; i++) {
	// 	servo1.write(angles[i]);
	// 	delay(position_delay);
	// }

	// =================== SERVO #2 SWEEP ===================
	// for (int i=0; i < 13; i++) {
	// 	servo2.write(angles[i]);
	// 	delay(position_delay);
	// }

	for (int angle_index=0; angle_index<13; angle_index++){
		for (int servo_index=0; servo_index<2; servo_index++){
			servo_list[servo_index]->write(angles[angle_index]);
		}
		delay(position_delay);
	}

	delay(1000);
}

*/