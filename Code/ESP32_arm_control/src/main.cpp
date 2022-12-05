#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_AMOUNT 5

const int Q_LENGH = SERVO_AMOUNT;
const int MIN_MICROSECONDS = 550; // By the book is supposed to be 500, the values being set are a result of experimental tests from the author of the library
const int MAX_MICROSECONDS = 2350; // By the book is supposed to be 2500, the values being set are a result of experimental tests from the author of the library
const int PWM_FREQUENCY = 50;
char c;
int q[Q_LENGH];

// create four servo objects 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo_list[6] = {servo1, servo2, servo3, servo4, servo5};

const char PWM_PINS_AVAILABLE[] = { 12, 13, 14, 15, 16, 17 };

void setup() {
  // Initialize serial communication
	Serial.begin(9600);
	while (!Serial);
	Serial.println("---> ESP32 Communication Initalized <---");

	// Empty serial buffer from any garbage
	while (Serial.available() > 0){
		int value = Serial.read();
	}

  // Initialize servos
	for (int servo_index=0; servo_index<SERVO_AMOUNT; servo_index++){
		servo_list[servo_index].setPeriodHertz(PWM_FREQUENCY);
		servo_list[servo_index].attach(PWM_PINS_AVAILABLE[servo_index], MIN_MICROSECONDS, MAX_MICROSECONDS);
	}
}

void loop() {
	
	if (Serial.available() >= Q_LENGH){
		for (int i=0; i < Q_LENGH; i++) {
			c = Serial.read(); // Read stuff in buffer
			int joint_value = (int)c; // Cast byte to its integer decimal value
			q[i] = joint_value; // Record value in the internal array for q
		}

		// Set servos to the values received and report value written to them to the serial port
		Serial.print("ESP32 current q: ");
		for (int i=0; i < Q_LENGH; i++){
			int angle = q[i];
			servo_list[i].write(angle); // Write angle to servo
			Serial.println(angle); // Write servo angle to the serial port
		}
	}
}