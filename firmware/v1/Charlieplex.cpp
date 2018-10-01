// Modified from [https://playground.arduino.cc/Code/Charlieplex]

#include "Charlieplex.h"

Charlieplex::Charlieplex(byte* userPins, byte numberOfUserPins) {
	pins = userPins;
	numberOfPins = numberOfUserPins;
	clear();
}

void Charlieplex::write(charliePin pin, bool state) {
	if (state) {
		setHigh(pin.vcc);
		setLow(pin.gnd);
	} else {
		setZ(pin.vcc);
		setZ(pin.gnd);
	}
}

void Charlieplex::writePwm(charliePin pin, byte dutyCycle) {
	if (dutyCycle) {
		setPwm(pin.vcc, dutyCycle);
		setLow(pin.gnd);
	} else {
		setZ(pin.vcc);
		setZ(pin.gnd);
	}
}

//set a pin HIGH
void Charlieplex::setHigh(byte pin) {
	pinMode(pins[pin], OUTPUT);
	digitalWrite(pins[pin], HIGH);
}

void Charlieplex::setPwm(byte pin, byte dutyCycle) {
	pinMode(pins[pin], OUTPUT);
	analogWrite(pins[pin], dutyCycle);
}

//set a pin LOW
void Charlieplex::setLow(byte pin) {
	pinMode(pins[pin], OUTPUT);
	digitalWrite(pins[pin], LOW);
}

//set a pin to High-Z
void Charlieplex::setZ(byte pin) {
	pinMode(pins[pin], INPUT);
	digitalWrite(pins[pin], LOW);
}

//set all as input
void Charlieplex::clear() {
	for (byte i = 0; i < numberOfPins; i++) {
		setZ(i);
	}
}
