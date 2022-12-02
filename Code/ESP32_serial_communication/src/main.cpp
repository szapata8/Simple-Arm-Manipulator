#include <Arduino.h>

#define Q_LENGH 5

char c;
int q[Q_LENGH];

void setup() {
	Serial.begin(9600);
	while (!Serial);
	Serial.println("---> ESP32 Communication Initalized <---");

	// Empty serial buffer from any garbage
	while (Serial.available() > 0){
		int value = Serial.read();
	}
	delay(500);
}

void loop() {
	if (Serial.available() == Q_LENGH){
		for (int i=0; i < Q_LENGH; i++) {
			c = Serial.read(); // Read stuff in buffer
			int joint_value = (int)c; // Cast byte to its integer decimal value
			q[i] = joint_value; // Record value in the internal array for q
		}

		// Echo values received
		Serial.print("ESP32 current q: ");
		for (int val=0; val < Q_LENGH; val++){
			Serial.print(q[val]);
		}
	}
}
