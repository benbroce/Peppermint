// Modified from [https://playground.arduino.cc/Code/Charlieplex]

#include "Charlieplex.h"

// Constructs a charlieplexed matrix
Charlieplex::Charlieplex(byte* userPins, byte numberOfUserPins) {
	pins = userPins;
	numberOfPins = numberOfUserPins;
	clear();
}

// Turns a specific LED on or off
void Charlieplex::write(charlieLED pin, bool state) {
	if (state) {
		setHigh(pin.vcc);
		setLow(pin.gnd);
	} else {
		setZ(pin.vcc);
		setZ(pin.gnd);
	}
}

// Pulses a specific LED at a given duty cycle
void Charlieplex::writePwm(charlieLED pin, byte dutyCycle) {
	if (dutyCycle) {
		setPwm(pin.vcc, dutyCycle);
		setLow(pin.gnd);
	} else {
		setZ(pin.vcc);
		setZ(pin.gnd);
	}
}

// Set all to High-Z
void Charlieplex::clear() {
	for (byte i = 0; i < numberOfPins; i++) {
		setZ(i);
	}
}

// Set a pin HIGH
void Charlieplex::setHigh(byte pin) {
	pinMode(pins[pin], OUTPUT);
	digitalWrite(pins[pin], HIGH);
}

// Pulse a pin w/ PWM
void Charlieplex::setPwm(byte pin, byte dutyCycle) {
	pinMode(pins[pin], OUTPUT);
	analogWrite(pins[pin], dutyCycle);
}

// Set a pin LOW
void Charlieplex::setLow(byte pin) {
	pinMode(pins[pin], OUTPUT);
	digitalWrite(pins[pin], LOW);
}

// Set a pin to High-Z (set to INPUT w/o pullup)
void Charlieplex::setZ(byte pin) {
	pinMode(pins[pin], INPUT);
	digitalWrite(pins[pin], LOW);
}
